#!/usr/bin/env python3

import sys
sys.path.append('/home/robotics1/catkin_ws/src/py-code')

sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Pose
from move_to_goal_service import movebase_client
from robot_pose_service import gps_location
from turtlebot3_gazebo.srv import PlaceObjects


# Import the functions provided in the question
from environment_functions import spawn_model, delete_model, initialize_environment, create_scene

def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist

def place_object(object_name, place_location):
    # delete selected object from bag and place it in gazebo
    me_pose = gps_location()
    dist2 = distance(me_pose[0], me_pose[1], place_location[0], place_location[1])


    if dist2<.35:
        delete_model(object_name)
        spawn_model(name=object_name, spawn_location=place_. location)
        print('Placed the object')
        return True
    else: 
        print('Need to be closer to the location to place the object (and NOT on it!)') 
        return False

def handle_place_object(req):
    object_name = req.object_name
    place_location = req.place_location

    # Call the place_object function with the provided object name and location
    success = place_object(object_name, place_location)

    # Return success as True if placing is successful
    return success

def place_object_server():
    rospy.init_node('place_object_server')
    s = rospy.Service('place_object', PlaceObjects, handle_place_object)
    print("Ready to place objects.")
    rospy.spin()

if __name__ == "__main__":
    try:
        place_object_server()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted by user!")