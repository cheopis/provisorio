#!/usr/bin/env python3

import sys
import copy
import rospy
import roscpp
import time
import tf2_ros
import numpy as np
import moveit_commander

from math import pi, degrees
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler

import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

from save_file import *
# from movimentacao import move_robot_joint

GROUP_NAME_ARM = "manipulator"
GRIPPER_FRAME = "iiwa_link_ee"
ROBOT_DESCRIPTION = '/iiwa/robot_description'

def go_to(move_group, target_pose,q):

    if len(q) == 3:
        print(q)
        q = quaternion_from_euler(q[0],q[1],q[2])

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = target_pose[0]
    pose_goal.position.y = target_pose[1]
    pose_goal.position.z = target_pose[2]
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    move_group.set_pose_target(pose_goal)
    
    move_group.go(wait=True)

    joints = move_group.get_joint_value_target()
    # 
    # move_group.stop()
    # move_group.clear_pose_targets()

def carthesian_plan(group,target_pose, q):
    waypoints = []
    wpose = group.get_current_pose().pose

    
    for pose in target_pose:
        wpose.position.x = pose[0]
        wpose.position.y = pose[1]
        wpose.position.z = pose[2]
        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]

        waypoints.append(copy.deepcopy(wpose))

    plan, f = group.compute_cartesian_path(waypoints, 0.01)

    #group.execute(plan, wait=True)
    #group.clear_pose_targets()
    return plan 

def move_arm(target_pose, q =[pi, 0, 0]):
    rospy.sleep(1.0)

    start = time.time()
    
    robot = moveit_commander.RobotCommander(ROBOT_DESCRIPTION)
    scene = moveit_commander.PlanningSceneInterface('/iiwa')
    
    group = MoveGroupCommander(name=GROUP_NAME_ARM, robot_description=ROBOT_DESCRIPTION, ns='/iiwa')
    group.set_end_effector_link(GRIPPER_FRAME)
    group.set_planning_time(30.0)

    #atach_box(robot, scene)
    # attached_obj = scene.get_attached_objects(["pistol"])
    #is attached = len(attached_obj.keys()) > 0
    
    display_trajectory_publisher = rospy.Publisher('/iiwa/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    rospy.sleep(1.0)

    q = quaternion_from_euler(q[0], q[1], q[2])
    

    #go_to(group, target_pose[0], q)
    
    plan = carthesian_plan(group, target_pose, q)
    
    path = []
    for pos in plan.joint_trajectory.points:
        s = ' '.join([str(degrees(a)) for a in pos.positions])
        path.append(s)

    # path = []
    # for pos in plan:
    # 	s = ' '.join([str(degrees(a)) for a in pos])
    # 	path.append(s)
    
    return path

def atach_box(robot, scene):

    #create box
    box = geometry_msgs.msg.PoseStamped()
    box.header.frame_id = GRIPPER_FRAME
    box.pose.orientation.w = 1.0
    box.pose.position.z = 0.15

    box_name = 'pistol'
    
    # create box in scene
    scene.add_box(box_name, box, size=(0.11, 0.11, 0.3))

    touch_link = robot.get_link_names(group=GROUP_NAME_ARM)
    scene.attach_box(GRIPPER_FRAME, box_name, touch_links=touch_link)

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("iiwa_move_test")
    
    robot_height = 0.6
    
    poses = [[0.7, -0.1, 1.07-robot_height],[0.7, 0.1, 1.07-robot_height]]
    
    path = move_arm(poses)
    
    save_path_all_formats(path)