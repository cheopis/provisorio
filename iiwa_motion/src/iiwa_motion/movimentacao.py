#!/usr/bin/env python3
import numpy as np

from settings import max_t1_speed, max_t2_speed, max_auto_speed, medium_auto_speed, slow_auto_speed
from time import time, sleep
from datetime import datetime
from statistics import mean, median, stdev
from math import degrees, radians, pi
import sys

from gap_detection.msg import GapProfile

sys.path.insert(1, '/home/ariele/flymov/catkin_ws/src/iafa_sealing/iiwa_motion/nodes')
from kuka_ros_node import *
from iiwa_motion.iiwa_controller import TrajectoryEngine
import moveit_planner

def set_joint_velocity(client, value):
    ''' Set the joint velocity value from 0.0 to 1.0'''

    print(f"Sending joint velocity {value}")
    client.send_command(f'setJointVelocity {value}')
    sleep(1)

def set_cartesian_velocity(client, value):
    ''' Set the cartesian velocity value in mm/s'''

    print(f"Sending cartesian velocity {value}")
    client.send_command(f'setCartVelocity {value}')
    sleep(1)
    
def move_robot_joint(client, position):
    #print(f"Sending position {position}")
    client.send_command(f'setPosition {position}')
    sleep(0.1)

    for i in range(100):
        # print(f'Is Finished? {client.isFinished[0]}')
        # print(client.JointVelocity) //Corrigir!
        
        #sleep(1)
        if client.isFinished:
            return
        if i == 99:
            #print("Max tries achieved, getting next point")
            return
        #print(f"Slept for {i} seconds")
    #print('Finished')

def move_robot_xyzabc(client, position): #funcao para Exame
    print(f"Sending position {position}")
    client.send_command(f'setPositionXYZABC {position}')
    #client.send_command(f'MoveXYZABC {position}')
    sleep(1)

    for i in range(100):
        print(f'Is Finished? {client.isFinished}')
        # print(client.JointVelocity) #Corrigir!
        sleep(1)
        if client.isFinished[0]:
            break
        if i == 99:
            print("Max tries achieved")
            sys.exit()
        print(f"Slept for {i} seconds")

    print('Finished')

def rotational_cycle(client, repetitions=10):
    for i in range(repetitions):
        move_robot_joint(client, '0 -15 00 75 90 30 0')
        move_robot_joint(client, '0 -15 00 75 90 -30 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def linear_cycle(client, repetitions=10):
    for i in range(repetitions):
        move_robot_xyzabc(client, '-660 50 800 - - - ptp')
        move_robot_xyzabc(client, '-660 50 750 - - - ptp')
        move_robot_xyzabc(client, '-660 0 750 - - - ptp')
        move_robot_xyzabc(client, '-660 0 800 - - - ptp')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def linear_cycle_y(client, repetitions=10):
    move_robot_joint(client, '0 -15 0 75 0 0 0')
    for i in range(repetitions):
        move_robot_xyzabc(client, '-660 0 750 - - - ptp')
        move_robot_xyzabc(client, '-560 0 750 - - - ptp')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def rotational_cycle_velocity(client, repetitions=10):
    max_auto_speed(client)

    for i in range(repetitions):
        move_robot_joint(client, '0 -15 -90 75 0 0 0')
        move_robot_joint(client, '0 -15 90 75 0 0 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

    sleep(10)
    medium_auto_speed(client)

    for i in range(repetitions):
        move_robot_joint(client, '0 -15 -90 75 0 0 0')
        move_robot_joint(client, '0 -15 90 75 0 0 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

    sleep(10)
    slow_auto_speed(client)

    for i in range(repetitions):
        move_robot_joint(client, '0 -15 -90 75 0 0 0')
        move_robot_joint(client, '0 -15 90 75 0 0 0')
        print(f'-----Repetition {i+1} finished-----')
    print('End of repetitions')

def cartesian_movement(client, position, velocity, orientation):
    print(f"Sending position {position} with {orientation} and velocity {velocity}")
    client.send_command(f'setCartesianPosition {position} {velocity} {orientation}')
    #client.send_command(f'MoveXYZABC {position}')
    #sleep(1)

    for i in range(100):
        #print(f'Is Finished? {client.isFinished}')
        # print(client.JointVelocity) #Corrigir!
        # sleep(1)
        if client.isFinished[0]:
            break
        if i == 99:
            print("Max tries achieved")
            sys.exit()
        print(f"Slept for {i} seconds")

    print('Finished')

client = kuka_iiwa_ros_node()  # Making a connection object.

# Adicionando monitoramento de login e endereço
while not client.isReady:
    print("Waiting for the client to connect...")
    sleep(1)

print(f"Client connected: {client.isReady}")
#print(f"Server address: {client.server_address}")

if client.OperationMode[0] == 'T1':
    max_t1_speed(client)
elif client.OperationMode[0] == 'T2':
    max_t2_speed(client)
elif client.OperationMode[0] == 'AUT':
    slow_auto_speed(client)

print('The robot is in', client.OperationMode[0], 'mode.')

def print_tool_position(client):
    rate = rospy.Rate(1)  # 1 Hz (imprime a cada segundo)
    while not rospy.is_shutdown():
        if client.ToolPosition is not None:
            print("Último valor de ToolPosition:", client.ToolPosition)
        else:
            print("Aguardando dados de ToolPosition...")
        rate.sleep()

def gap_callback(profile):
    global area
    area = profile.depth * profile.current_width

def salvar_linha(file, tempo: float) -> None:
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    with open(file, "a", encoding="utf-8") as f:
        f.write(f"{tempo:.6f}\n")

rospy.Subscriber(
        '/gap_detection/profile',
        GapProfile,
        gap_callback
    )

#client.send_command('setJointJerk 0.5')
#client.send_command('setJointVelocity 1.0')

# move_robot_joint(client, '0 0 0 0 0 0 0')
# sleep(5)

#move_robot_joint(client, '-90.01 0.06 -0.02 -89.70 -0.09 90.26 0')

# Points passed here as requested

T_br = np.array([[0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]])

# position = client.ToolPosition[0][:3]      
# start_p = np.array([y*0.001 for y in position])
# end_p = np.array([x+y for (x,y) in zip(start_p, [0,0,-0.3])])
file = datetime.now().strftime("benchmark_%Y-%m-%d_%H-%M-%S.log")

base = np.array([-0.85514, 1.3735, 0.31063])

# start_p = np.array([-0.35, 1.37, 1.2]) - base
# end_p = np.array([-0.35, 1.37, 0.5]) - base

#q = [pi/2, 0, pi/2]

start_p = np.array([-0.32, 1.3735, 1.2]) - base
end_p = np.array([-0.32, 1.3735, 0.6]) - base

q = [0, pi/2, -pi]

# start_p = [0.5, -0.2, 0.4]
# end_p = [0.5, 0.2, 0.6]
# q = None

#moveit_planner.go_to(start_p, q)

init = False

while not init:
    current_q = client.JointPosition[0]
    if current_q[0] is None:
        pass
    else: 
        init = True

engine = TrajectoryEngine()

path, status = engine.generate_straight_line(start_p, end_p, 100, current_q, q)


print(status)

n = 1
flow = 0.00002

for pose in path:
    inicio = time()
    for i in range(0,len(pose)):
        pose[i] = degrees(pose[i])
    vel = 0.4
    move_robot_joint(client, f"{' '.join(map(str, pose))}")

    fim = time()
    elapsed = fim - inicio
    salvar_linha(file, elapsed)



# cartesian_movement(client, f"{' '.join(map(str,end_p@T_br))}", 
#                             0.1, 
#                             f"{' '.join(map(str,q))}")

# path = moveit_planner.move_arm(poses, q)

# for pose in path:
#     print(pose)
#     move_robot_joint(client, f'{pose}')

def moveit_movement():
    # Go to initial pose
    move_robot_joint(client, '0 0 0 0 0 0 0')
    sleep(5)
    #move_robot_joint(client, '-90.01 0.06 -0.02 -89.70 -0.09 90.26 0')

    # Compute path
    robot_height = 0.6
    poses = [[0.7, -0.2, 1.06-robot_height], [0.7, 0.2, 1.06-robot_height]]


    path = moveit_planner.move_arm(poses)
    print('go')

    # Move
    for pose in path[-40:]:
        move_robot_joint(client, f'{pose}')
        #client.send_command(f'setPosition {pose}')
        # get picture
        # inference
        
        # take more pictures
        # check the failure
        
        # estimate position


# # save data

if __name__ == "__main__":
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Gap Detection finalizado")