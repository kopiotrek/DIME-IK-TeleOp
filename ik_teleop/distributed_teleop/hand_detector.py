from ik_teleop.teleop_utils.hand_detector import MediapipeJoints

def detector():
    mp_detector = MediapipeJoints(display_image = False)
    mp_detector.detect()

if __name__ == '__main__':
    print("***************************************************************\n     Starting detection process \n***************************************************************")
    print("\nHand detection process started!\n")
    detector()
