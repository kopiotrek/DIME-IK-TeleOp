from ik_teleop import finger_controller
import concurrent.futures
                   
if __name__ == '__main__':
    print('Started Allegro Hand controller')
    with concurrent.futures.ProcessPoolExecutor() as executor:
        finger_types = ['index','middle','ring','thumb']
        results = executor.map(finger_controller.control_finger, finger_types)

    print(results)