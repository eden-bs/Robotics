#!/usr/bin/env python3

import sys
sys.path.append('/home/robotics1/catkin_ws/src/py-code')
sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Pose
from move_to_goal_service import movebase_client
from robot_pose_service import gps_location
from turtlebot3_gazebo.srv import PickObjectsNearby


# Import the functions provided in the question
from environment_functions import spawn_model, delete_model, initialize_environment, create_scene

import time

def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist

def pick_object(object_name, object_position):
    print('Trying to pick up: ' + object_name)
    me_pose = gps_location()
    object_x = object_position[0]
    object_y = object_position[1]   
    dist = distance(me_pose[0], me_pose[1], object_x, object_y)
    
    # TODO isEmpty = fcn_that_checks_that_nothing_is_in_the_knapsack() 

    if dist < 0.35:
        delete_model(object_name)
        time.sleep(1)
        spawn_model(name=object_name, spawn_location=[-2.40,-1.50,1.0]) # Put in knapsack
        time.sleep(1)
        print('...successfully.')
        return True
    else: 
        print('...unsuccessfully. Need to be closer to the object to pick it')
        return False

def handle_pick_object_nearby(req):
    object_name = req.object_name
    object_location = req.object_location

    # Call the pick_object function with the provided object name and location
    success = pick_object(object_name, object_location)

    # Return success as True if picking is successful
    return success

def pick_object_nearby_server():
    rospy.init_node('pick_object_nearby_server')
    s = rospy.Service('pick_object_nearby', PickObjectsNearby, handle_pick_object_nearby)
    print("Ready to pick objects nearby.")
    rospy.spin()

if __name__ == "__main__":
    pick_object_nearby_server()
