import os
import sys
import numpy as np

try:
    import rospy
except ModuleNotFoundError:
    print('Module rospy not found!\n'\
    'Check packages if you are using MediapipeJointsPublisher')

try:
    from std_msgs.msg import Float64MultiArray
except ModuleNotFoundError:
    print('Module std_msgs.msg NOT FOUND!! '\
    'Check packages if using robot hardware.')
    
POSE_COORD_TOPIC = '/transformed_mediapipe_joint_coords'

class BoundCalibrator(object):
    def __init__(self, storage_dir = os.getcwd()):
        try:
            rospy.init_node("hand_bound_calibration")
            rospy.Subscriber(POSE_COORD_TOPIC, Float64MultiArray, self._callback, queue_size = 1)

        except:
            pass

        self.hand_coords = None

        self.storage_path = os.path.join(storage_dir, "bound_data", "calibrated_values.npy")

    def _callback(self, coord_data):
        self.hand_coords = np.array(list(coord_data.data)).reshape(11, 2)
    
    def set_hand_coords(self, coords):
        self.hand_coords = coords

    def calibrate(self):
        # sys.stdin = open(0)

        print("Starting calibration\n")
        
        # Getting Index finger bounds
        print("Getting the vertical bounds for the Index finger.")
        index_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Index finger.")
        index_bounds[0][1] = self.hand_coords[7][1]   # Getting the y coordinate of the index tip position
        print("Registered upper bound is {}.\n".format(index_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Index finger.")
        index_bounds[0][0] = self.hand_coords[7][1]    # Getting the y coordinate of the index tip position
        print("Registered lower bound is {}.\n".format(index_bounds[0][0]))

        # Getting Middle finger bounds
        print("Getting the vertical bounds for the Middle finger.")
        middle_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Middle finger.")
        middle_bounds[0][1] = self.hand_coords[8][1]   # Getting the y coordinate of the middle tip position
        print("Registered upper bound is {}.\n".format(middle_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Middle finger.")
        middle_bounds[0][0] = self.hand_coords[8][1]   # Getting the y coordinate of the middle tip position
        print("Registered lower bound is {}.\n".format(middle_bounds[0][0]))

        # Getting Ring finger bounds
        print("Getting the vertical bounds for the Ring finger.")
        ring_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Ring finger.")
        ring_bounds[0][1] = self.hand_coords[9][1]     # Getting the y coordinate of the ring tip position
        print("Registered upper bound is {}.\n".format(ring_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Ring finger.")
        ring_bounds[0][0] = self.hand_coords[9][1]     # Getting the y coordinate of the ring tip position
        print("Registered lower bound is {}.\n".format(ring_bounds[0][0]))

        # Getting Pinky finger bounds
        print("Getting the vertical bounds for the Pinky finger.")
        pinky_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Pinky finger.")
        pinky_bounds[0][1] = self.hand_coords[10][1]    # Getting the y coordinate of the pinky tip position
        print("Registered upper bound is {}.\n".format(pinky_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Pinky finger.")
        pinky_bounds[0][0] = self.hand_coords[10][1]    # Getting the y coordinate of the pinky tip position
        print("Registered lower bound is {}.\n".format(pinky_bounds[0][0]))

        # Getting Thumb finger bounds
        print("Getting the quadrilateral bounds for the Thumb finger.")
        thumb_bounds = np.zeros((4, 2))

        register = input("Press Enter to register the upper right bound for the Thumb finger.")
        thumb_bounds[0][0] = self.hand_coords[6][0]
        thumb_bounds[0][1] = self.hand_coords[6][1]
        
        print("Registered upper right bound is {}.\n".format(thumb_bounds[3]))
        
        register = input("Press Enter to register the lower right bound for the Thumb finger.")
        thumb_bounds[1][0] = self.hand_coords[6][0]
        thumb_bounds[1][1] = self.hand_coords[6][1]
        
        print("Registered lower right bound is {}.\n".format(thumb_bounds[2]))

        register = input("Press Enter to register the lower left bound for the Thumb finger.")
        thumb_bounds[2][0] = self.hand_coords[6][0]
        thumb_bounds[2][1] = self.hand_coords[6][1]

        print("Registered lower left bound is {}.\n".format(thumb_bounds[1]))

        register = input("Press Enter to register the upper left bound for the Thumb finger.")
        thumb_bounds[3][0] = self.hand_coords[6][0]
        thumb_bounds[3][1] = self.hand_coords[6][1]

        print("Registered upper left bound is {}.\n".format(thumb_bounds[0]))

        print("Registered bounds: \nIndex finger bounds: {}\nMiddle finger bounds: {}\nRing finger bounds: {} \nPinky finger bounds: {} \nThumb finger bounds: {}\n".format(index_bounds, middle_bounds, ring_bounds, pinky_bounds, thumb_bounds))

        self.calibrated_bounds = np.vstack([index_bounds, middle_bounds, ring_bounds, pinky_bounds, thumb_bounds])

        np.save(self.storage_path, self.calibrated_bounds)

    def set_process_queue(self, q):
        self.queue = q
    
    def _get_queue_coords(self):
        attempts = 0
        max_attempts = 5

        while(attempts < max_attempts):
            joints =  self.queue.get()
            if joints is not None:
                self.hand_coords= joints
                return
            else:
                attempts += 1
                time.sleep(5)
        print('No hand detected')
        import sys
        sys.exit(0)

            

    def process_calibrate(self, queue):
        self.set_process_queue(queue)
        print("Starting calibration\n")
        
        # Getting Index finger bounds
        print("Getting the vertical bounds for the Index finger.")
        index_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Index finger.")
        self._get_queue_coords()
        index_bounds[0][1] = self.hand_coords[7][1]   # Getting the y coordinate of the index tip position
        print("Registered upper bound is {}.\n".format(index_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Index finger.")
        self._get_queue_coords()
        index_bounds[0][0] = self.hand_coords[7][1]    # Getting the y coordinate of the index tip position
        print("Registered lower bound is {}.\n".format(index_bounds[0][0]))

        # Getting Middle finger bounds
        print("Getting the vertical bounds for the Middle finger.")
        middle_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Middle finger.")
        self._get_queue_coords()
        middle_bounds[0][1] = self.hand_coords[8][1]   # Getting the y coordinate of the middle tip position
        print("Registered upper bound is {}.\n".format(middle_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Middle finger.")
        self._get_queue_coords()
        middle_bounds[0][0] = self.hand_coords[8][1]   # Getting the y coordinate of the middle tip position
        print("Registered lower bound is {}.\n".format(middle_bounds[0][0]))

        # Getting Ring finger bounds
        print("Getting the vertical bounds for the Ring finger.")
        ring_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Ring finger.")
        self._get_queue_coords()
        ring_bounds[0][1] = self.hand_coords[9][1]     # Getting the y coordinate of the ring tip position
        print("Registered upper bound is {}.\n".format(ring_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Ring finger.")
        self._get_queue_coords()
        ring_bounds[0][0] = self.hand_coords[9][1]     # Getting the y coordinate of the ring tip position
        print("Registered lower bound is {}.\n".format(ring_bounds[0][0]))

        # Getting Pinky finger bounds
        print("Getting the vertical bounds for the Pinky finger.")
        pinky_bounds = np.zeros((1, 2))

        register = input("Press Enter to register the upper bound for the Pinky finger.")
        self._get_queue_coords()
        pinky_bounds[0][1] = self.hand_coords[10][1]    # Getting the y coordinate of the pinky tip position
        print("Registered upper bound is {}.\n".format(pinky_bounds[0][1]))

        register = input("Press Enter to register the lower bound for the Pinky finger.")
        self._get_queue_coords()
        pinky_bounds[0][0] = self.hand_coords[10][1]    # Getting the y coordinate of the pinky tip position
        print("Registered lower bound is {}.\n".format(pinky_bounds[0][0]))

        # Getting Thumb finger bounds
        print("Getting the quadrilateral bounds for the Thumb finger.")
        thumb_bounds = np.zeros((4, 2))

        register = input("Press Enter to register the upper right bound for the Thumb finger.")
        self._get_queue_coords()
        thumb_bounds[0][0] = self.hand_coords[6][0]
        thumb_bounds[0][1] = self.hand_coords[6][1]
        
        print("Registered upper right bound is {}.\n".format(thumb_bounds[3]))
        
        register = input("Press Enter to register the lower right bound for the Thumb finger.")
        self._get_queue_coords()
        thumb_bounds[1][0] = self.hand_coords[6][0]
        thumb_bounds[1][1] = self.hand_coords[6][1]
        
        print("Registered lower right bound is {}.\n".format(thumb_bounds[2]))

        register = input("Press Enter to register the lower left bound for the Thumb finger.")
        self._get_queue_coords()
        thumb_bounds[2][0] = self.hand_coords[6][0]
        thumb_bounds[2][1] = self.hand_coords[6][1]

        print("Registered lower left bound is {}.\n".format(thumb_bounds[1]))

        register = input("Press Enter to register the upper left bound for the Thumb finger.")
        self._get_queue_coords()
        thumb_bounds[3][0] = self.hand_coords[6][0]
        thumb_bounds[3][1] = self.hand_coords[6][1]

        print("Registered upper left bound is {}.\n".format(thumb_bounds[0]))

        print("Registered bounds: \nIndex finger bounds: {}\nMiddle finger bounds: {}\nRing finger bounds: {} \nPinky finger bounds: {} \nThumb finger bounds: {}\n".format(index_bounds, middle_bounds, ring_bounds, pinky_bounds, thumb_bounds))

        self.calibrated_bounds = np.vstack([index_bounds, middle_bounds, ring_bounds, pinky_bounds, thumb_bounds])

        np.save(self.storage_path, self.calibrated_bounds)


    def check_and_load_calibration_file(self):
        sys.stdin = open(0)

        if os.path.exists(self.storage_path):
            print("\nCalibration file already exists. Do you want to create a new one? Press y for Yes else press Enter")
            use_calibration_file = input()

            if use_calibration_file == "y":
                self.calibrate()
            else:
                self.calibrated_bounds = np.load(self.storage_path)

        else:
            print("\nNo calibration file found.\n")
            self.calibrate()

        return self.calibrated_bounds
    
'''
For running on a linux machine w/o ros/rospy set up.
Used to calibrate cameras for simulation demos.
'''

CAM_SERIAL_NUM = '023322062089'
def run_detect(queue):
    print('Detector process starting...')
    detector = MediapipeJoints(cam_serial_num=CAM_SERIAL_NUM)
    detector.queue = queue
    
    detector.detect()

def run_calibrate_cb(hand_coords,calibrator):
    calibrator.process_calibrate(hand_coords)
    

            
if(__name__=="__main__"):
    from multiprocessing import Process, Queue
    from ik_teleop.teleop_utils.hand_detector import MediapipeJoints
    import time 

    coord_q = Queue(1)
    calibrator = BoundCalibrator(storage_dir = os.getcwd())
    
    det_process = Process(target = run_detect, args=(coord_q,))
    det_process.start()

    # calib_update_process = Process(target = run_calibrate_cb, args=(coord_q,calibrator))
    # calib_update_process.start()
    calibrator.process_calibrate(coord_q)

    
    
