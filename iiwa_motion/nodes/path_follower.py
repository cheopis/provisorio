#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger

def run_mission():
    rospy.init_node('iiwa_mission_client')

    # controller = 
    # # Points passed here as requested
    # start_point = [0.5, -0.2, 0.4]
    end_point = [0.0, 0.0, -0.1]

    # # Set parameters on the server so the node can read them
    # rospy.set_param('/iiwa_control/start_point', start_point)
    node = '/kuka_iiwa_client_25'
    rospy.set_param(node + '/end_point', end_point)

    rospy.wait_for_service(node + '/execute_linear_motion')
    try:
        move_service = rospy.ServiceProxy(node + '/execute_linear_motion', Trigger)
        rospy.loginfo("Requesting motion...")
        response = move_service()
        rospy.loginfo(f"Service result: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    run_mission()