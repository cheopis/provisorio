import numpy as np
from math import degrees, cos, sin
from time import sleep
from iiwa_motion.iiwa_kinematics import KukaIIWA14Kinematics

class TrajectoryEngine:
    """
    Handles Cartesian path generation and IK mapping for the IIWA 14.
    """
    def __init__(self):
        self.kinematics = KukaIIWA14Kinematics()
        self.client = None

    def start_client(self, client):
        self.client = client

    def generate_straight_line(self, start_xyz, end_xyz, steps, q_initial, orientation = None):
        """
        Generates a sequence of joint angles for a straight Cartesian line.
        """

        # if not self.client:
        #     return None, f"Client not inicialized"
        
        joint_path = []
        q_current = np.array(q_initial)

        for i in range(steps + 1):
            alpha = i / float(steps)
            # Linear interpolation of position
            interp_pos = [
                start_xyz[j] + (end_xyz[j] - start_xyz[j]) * alpha 
                for j in range(3)
            ]

            # Define target matrix (keeping orientation constant/neutral for the line)
            target_T = np.eye(4)
            target_T[0:3, 3] = interp_pos
            # Neutral orientation: Flange pointing down
            if orientation:
                target_T[0:3, 0:3] = self.get_rotation_matrix(orientation)
            else:
                # (p)
                target_T[0:3, 0:3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

            # Solve IK using the previous point as the seed for continuity
            q_sol, success = self.kinematics.inverse_kinematics(target_T, q_guess=q_current)
            
            if not success:
                return None, f"IK failed at step {i}"
            
            joint_path.append(q_sol.tolist())
            q_current = q_sol # Update seed for next step

        return joint_path, "Success"
    
    def get_rotation_matrix(self, orientation):
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

        return R
    
    def follow_path(self, start, end):
        if not self.client:
            return None, f"Client not inicialized"
        
        init = False

        while not init:
            current_q = self.client.JointPosition[0]
            if current_q[0] is None:
                pass
            else: 
                init = True

        path, success = self.generate_straight_line(start, end, 100, current_q)

        if not path:
            return f"IK failed, cannot follow path"

        for pose in path:
            for i in range(0,len(pose)):
                pose[i] = degrees(pose[i])
            self.move_robot_joint(self.client, f"{' '.join(map(str, pose))}")
            if i==0:
                sleep(10)

    def move_robot_joint(self, position):
        #print(f"Sending position {position}")
        self.client.send_command(f'setPosition {position}')
        sleep(0.1)

        for i in range(100):
            if self.client.isFinished:
                return
            if i == 99:
                print("Max tries achieved, getting next point")
                return
            print(f"Slept for {i} seconds")


