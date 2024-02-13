#!/usr/bin/env python3

import sys
sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.msg import ModelStates
from turtlebot3_gazebo.srv import SenseObjects

def find_objects():
    # request from Gazebo the global pose of all objects
    rospy.loginfo("Requesting Global Object Poses from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose)  - 3 # ignore: [ground_plane, room1, turtlebot3_burger]    	  	   	
    print('I found ' +str(number_of_objects) +' Objects')
    print(model_state.name[3:])
    return model_state.name[3:], model_state.pose[3:]

def handle_sense_objects(req):
    # Call the find_objects function to get the names and poses of objects
    object_names, object_poses = find_objects()
    object_positions_x = [pose.position.x for pose in object_poses]
    object_positions_y = [pose.position.y for pose in object_poses]
    # Return the names and poses of objects as a response
    return object_names, object_positions_x, object_positions_y

def sense_objects_server():
    rospy.init_node('sense_objects_server')
    s = rospy.Service('sense_objects', SenseObjects, handle_sense_objects)
    print("Ready to sense objects.")
    rospy.spin()

if __name__ == "__main__":
    sense_objects_server()
