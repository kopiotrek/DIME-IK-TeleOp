import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import minimize

# Load the URDF file
from ik_teleop.teleop_utils.files import *

# Load the URDF
urdf_path = get_path_in_package("robot/assets/thumb.urdf")
tree = ET.parse(urdf_path)
root = tree.getroot()

# Initialize lists to store joint parameters
joint_names = []
joint_types = []
parents = []
children = []
axes = []
origins = []

# Iterate through all joints
for joint in root.findall('joint'):
    name = joint.get('name')
    joint_type = joint.get('type')
    parent = joint.find('parent').get('link')
    child = joint.find('child').get('link')
    axis_elem = joint.find('axis')
    if axis_elem is not None:
        axis = [float(x) for x in axis_elem.get('xyz').split()]
    else:
        axis = [0, 0, 0]  # Default axis
    origin_elem = joint.find('origin')
    if origin_elem is not None:
        xyz = [float(x) for x in origin_elem.get('xyz', '0 0 0').split()]
        rpy = [float(x) for x in origin_elem.get('rpy', '0 0 0').split()]
    else:
        xyz = [0, 0, 0]
        rpy = [0, 0, 0]

    # Store the extracted information
    joint_names.append(name)
    joint_types.append(joint_type)
    parents.append(parent)
    children.append(child)
    axes.append(axis)
    origins.append({'xyz': xyz, 'rpy': rpy})

def compute_dh_parameters(origins, axes):
    dh_params = []

    for i in range(len(origins)):
        xyz = origins[i]['xyz']
        rpy = origins[i]['rpy']
        axis = axes[i]

        # Simplified computation (needs proper implementation)
        alpha = rpy[0]
        a = xyz[0]
        d = xyz[2]
        theta = 0

        dh_params.append({'alpha': alpha, 'a': a, 'd': d, 'theta': theta})
    
    print (dh_params)
    return dh_params


# Compute DH parameters
dh_params = compute_dh_parameters(origins, axes)