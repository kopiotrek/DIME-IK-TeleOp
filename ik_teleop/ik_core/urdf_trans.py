import rospy
from xml.etree import ElementTree as ET
import numpy as np

def parse_origin(element):
    """
    Parses the origin element of a joint/link in the URDF.

    Parameters:
        element (ElementTree.Element): The origin element.

    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    xyz = element.attrib.get('xyz', '0 0 0').split()
    rpy = element.attrib.get('rpy', '0 0 0').split()

    # Translation matrix
    translation = np.eye(4)
    translation[:3, 3] = list(map(float, xyz))

    # Rotation matrices
    roll, pitch, yaw = map(float, rpy)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    rotation = Rz @ Ry @ Rx

    # Combine rotation and translation
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation[:3, 3]

    return transform

def get_transform_from_urdf(urdf_file, start_joint, end_joint):
    """
    Reads a URDF file and computes the transform from the parent of start_joint to end_joint.

    Parameters:
        urdf_file (str): Path to the URDF file.
        start_joint (str): Name of the starting joint (e.g., 'joint_0.0').
        end_joint (str): Name of the ending joint (e.g., 'joint_3.0_tip').

    Returns:
        np.ndarray: 4x4 transformation matrix from start_joint to end_joint.
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Build joint map
    joint_map = {}
    for joint in root.findall('joint'):
        joint_name = joint.attrib['name']
        origin = joint.find('origin')
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']

        joint_map[joint_name] = {
            'origin': parse_origin(origin),
            'parent': parent,
            'child': child
        }

    # Find the chain of joints from start_joint to end_joint
    chain = []
    current_joint = start_joint
    while True:
        if current_joint not in joint_map:
            raise ValueError(f"Joint {current_joint} not found in URDF")
        chain.append(current_joint)
        child_link = joint_map[current_joint]['child']

        # Check if the child link is the parent link of the end_joint
        if any(joint.attrib['name'] == end_joint and joint.find('parent').attrib['link'] == child_link for joint in root.findall('joint')):
            chain.append(end_joint)
            break

        # Find the next joint connected to the child link
        next_joint = None
        for joint in root.findall('joint'):
            if joint.find('parent').attrib['link'] == child_link:
                next_joint = joint.attrib['name']
                break

        if not next_joint:
            raise ValueError(f"Cannot find a joint connecting link {child_link} to the chain")

        current_joint = next_joint

    # Compute the transformation matrix
    transform = np.eye(4)
    for joint in chain:
        transform = transform @ joint_map[joint]['origin']

    return transform

if __name__ == "__main__":
    rospy.init_node("urdf_transform_reader")

    # Path to your URDF file
    urdf_file_path = "/home/mcw/RPL/DIME-Controllers/src/Allegro-Hand-Controller-DIME/src/allegro_hand_description/urdf/allegro_hand_description_right.urdf"

    # Specify the joints
    start_joint = "joint_3.0"
    end_joint = "joint_3.0_tip"

    try:
        transform_matrix = get_transform_from_urdf(urdf_file_path, start_joint, end_joint)
        rospy.loginfo(f"Transform from {start_joint} to {end_joint}:\n{transform_matrix}")
    except Exception as e:
        rospy.logerr(f"Failed to compute transform: {e}")