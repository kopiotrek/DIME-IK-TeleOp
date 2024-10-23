import numpy as np

# def angle_2p_3d(a, b, c):       

#     v1 = np.array([ a[0] - b[0], a[1] - b[1], a[2] - b[2] ])
#     v2 = np.array([ c[0] - b[0], c[1] - b[1], c[2] - b[2] ])

#     v1mag = np.sqrt([ v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] ])
#     v1norm = np.array([ v1[0] / v1mag, v1[1] / v1mag, v1[2] / v1mag ])

#     v2mag = np.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])
#     v2norm = np.array([ v2[0] / v2mag, v2[1] / v2mag, v2[2] / v2mag ])
#     res = v1norm[0] * v2norm[0] + v1norm[1] * v2norm[1] + v1norm[2] * v2norm[2]
#     angle_rad = np.arccos(res)

#     return angle_rad

def calculate_angle(coord_1, coord_2, coord_3):
    vector_1 = coord_2 - coord_1
    vector_2 = coord_3 - coord_2

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

p1 = np.array([0., 0., 0.,])
p2 = np.array([-4.17426701,  9.63163421,  0.2807703 ])
p3 = np.array([-3.59644271,  8.54840575,  0.20037295])

angle= calculate_angle(p1, p2, p3)
# angle= angle_2p_3d(p1, p2, p3)

print("angle: ", angle)
    