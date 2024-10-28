import cv2
import mediapipe as mp

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)

# Initialize MediaPipe drawing utils
mp_drawing = mp.solutions.drawing_utils

# Function to process the video frame and draw hand landmarks
def process_frame(frame):
    # Convert the frame to RGB as mediapipe works with RGB images
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame and detect hands
    result = hands.process(rgb_frame)

    # If hands are detected, draw the landmarks
    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            # Draw hand landmarks on the original frame
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    
    return frame

# Function to read video and apply hand tracking
def hand_tracking_from_video(video_path):
    # Open the video file
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video file.")
        return

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Process the frame to detect and draw hand pose
        frame = process_frame(frame)

        # Show the processed frame
        cv2.imshow('Hand Pose Tracking', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close windows
    cap.release()
    cv2.destroyAllWindows()

# Main execution
if __name__ == "__main__":
    # Provide the path to the video file
    video_file_path = '/home/vm/RPL/DIME-IK-TeleOp/ik_teleop/teleop_utils/hand_recordings/attempt2_succesful/output_cap0.avi'
    
    # Call the function to track hands from the video file
    hand_tracking_from_video(video_file_path)
