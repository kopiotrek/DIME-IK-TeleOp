# Standard imports
import numpy as np

# Image based imports
import cv2

def perform_perspective_transformation(input_coordinate, mediapipe_bound, allegro_bound, allegro_height):
    # Get the perspective transformation matrix
    transformation_matrix = cv2.getPerspectiveTransform(np.float32(mediapipe_bound), np.float32(allegro_bound))

    # Multiply the transformation matrix with the input coordinate to get the transformed coordinate
    transformed_coordinate = np.matmul(np.array(transformation_matrix), np.array([input_coordinate[0], input_coordinate[1], 1]))

    # Normalize the transformed coordinate using the z value
    transformed_coordinate = transformed_coordinate / transformed_coordinate[-1]

    allegro_coordinate = [
        allegro_height,
        transformed_coordinate[0],
        transformed_coordinate[1]
    ]

    return allegro_coordinate