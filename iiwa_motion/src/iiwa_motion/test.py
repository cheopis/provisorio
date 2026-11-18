from math import cos, sin, pi
import numpy as np

def get_rotation_matrix(orientation):
    rot_x = orientation[0]
    rot_y = orientation[1]
    rot_z = orientation[2]

    Rx = np.array([
        [1, 0, 0],
        [0, cos(rot_x), -sin(rot_x)],
        [0, sin(rot_x), cos(rot_x)]
    ])
    
    Ry = np.array([
        [cos(rot_y), 0, sin(rot_y)],
        [0, 1, 0],
        [-sin(rot_y), 0, cos(rot_y)]
    ])

    Rz = np.array([
        [cos(rot_z), -sin(rot_z), 0],
        [sin(rot_z), cos(rot_z), 0],
        [0, 0, 1]
    ])

    R = Rx @ Ry @ Rz
    print(R)

    return R


R = get_rotation_matrix([0,0,-pi/2])

M = [0.5, -0.2, 0.4]@R

print(M)

base = [-0.85514, 1.02705, 0.31063]

start_p = [0.5, -0.2, 0.4]
end_p = [0.5, 0.2, 0.6]

print(np.array(end_p@R)+np.array(base))
