import numpy as np
import cv2

def normalize_vector(vector):
    return vector / np.linalg.norm(vector)

def moving_average(vector, moving_average_queue, limit):
    moving_average_queue.append(vector)

    if len(moving_average_queue) > limit:
        moving_average_queue.pop(0)

    mean_vector = np.mean(moving_average_queue, axis = 0)
    return mean_vector

def get_distance(start_vector, end_vector):
    return np.linalg.norm(end_vector - start_vector)

def linear_transform(curr_val, source_bound, target_bound):
    multiplier = (target_bound[1] - target_bound[0]) / (source_bound[1] - source_bound[0])
    target_val = ((curr_val - source_bound[0]) * multiplier) + target_bound[0]
    return target_val

def perspective_transform(input_coordinates, given_bound, target_bound):
    transformation_matrix = cv2.getPerspectiveTransform(np.float32(given_bound), np.float32(target_bound))
    transformed_coordinate = np.matmul(np.array(transformation_matrix), np.array([input_coordinates[0], input_coordinates[1], 1]))
    transformed_coordinate = transformed_coordinate / transformed_coordinate[-1]

    return transformed_coordinate[0], transformed_coordinate[1]

def calculate_angle(coord_1, coord_2, coord_3):
    vector_1 = coord_2 - coord_1
    vector_2 = coord_3 - coord_2

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

# def calculate_angle(a, b, c):       
#     print("a",a)
#     print("b",b)
#     print("c",c)

#     v1 = np.array([ a[0] - b[0], a[1] - b[1], a[2] - b[2] ])
#     v2 = np.array([ c[0] - b[0], c[1] - b[1], c[2] - b[2] ])

#     v1mag = np.sqrt([ v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] ])
#     v1norm = np.array([ v1[0] / v1mag, v1[1] / v1mag, v1[2] / v1mag ])

#     v2mag = np.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])
#     v2norm = np.array([ v2[0] / v2mag, v2[1] / v2mag, v2[2] / v2mag ])
#     res = v1norm[0] * v2norm[0] + v1norm[1] * v2norm[1] + v1norm[2] * v2norm[2]
#     angle_rad = np.arccos(res)

#     return angle_rad

def coord_in_bound(bound, coord):
    bound_points = np.array(bound, dtype=np.float32).reshape(-1, 2)
    return cv2.pointPolygonTest(bound_points, tuple(coord), False)    