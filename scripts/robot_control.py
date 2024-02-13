#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
import random
import copy
import time
import sys
sys.path.append('/home/robotics1/catkin_ws/src/py-code')
sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
from geometry_msgs.msg import Point, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates, ModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from environment_functions import spawn_model, delete_model, initialize_environment, create_scene

global knapsack_isEmpty
global knapsack_object
knapsack_object = "empty"
knapsack_isEmpty = True

def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist


def movebase_client(x,y,w=1.0):
    #moves the robot collision free to a x,y,theta pose (must be valid/reachable in the map)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = w
    print("goal sent:")
    print(goal)
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def gps_location():
    # request a GPS like pose information from the Gazebo server
    rospy.loginfo("Requesting Global Robot Pose from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    me_pose = Pose()
    me_pose = model_state.pose[2]
    me_pose_angles = euler_from_quaternion([me_pose.orientation.x, me_pose.orientation.y, me_pose.orientation.z, me_pose.orientation.w])
    print('My pose is (x,y,theta): ')
    print(me_pose.position.x, me_pose.position.y, me_pose_angles[2])
    return me_pose.position.x, me_pose.position.y, me_pose_angles[2]


def find_objects():
    # request from Gazebo the global pose of all objects
    rospy.loginfo("Requesting Global Object Poses from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose)  - 3 # ignore: [ground_plane, room1, turtlebot3_burger]    	  	   	
    print('I found ' +str(number_of_objects) +' Objects')
    print(model_state.name[3:])
    return model_state.name[3:], model_state.pose[3:]


def pick_object(object_name, object_position):
    global knapsack_isEmpty
    print('Trying to pick up: ' + object_name)
    me_pose = gps_location()
    object_x = object_position[0]
    object_y = object_position[1]	
    dist = distance(me_pose[0],me_pose[1],object_x,object_y)
    isEmpty = knapsack_isEmpty
    
    if dist <.35 and isEmpty:
        
        delete_model(object_name)
        time.sleep(1)
        spawn_model(name=object_name, spawn_location=[-2.41,-1.53,0.2]) #put in knapsack
        time.sleep(1)
        print('...successfully.')
        knapsack_isEmpty = False
    else: 
        print('...unsuccessfully. Need to be closer to the object to pick it')
        
def isPicked(object_name):
    global knapsack_object,knapsack_isEmpty
    if knapsack_object == object_name:
        return True
    else:
        return False

def place_object(object_name, place_location):
    global knapsack_isEmpty
    global knapsack_object
    # delete selected object from bag and place it in gazebo
    me_pose = gps_location()
    dist2 = distance(me_pose[0], me_pose[1], place_location[0], place_location[1])

    is_picked = isPicked(object_name)
    if not is_picked: 
        print('Object is not with me...')
        return False
    if dist2<.35:
        delete_model(object_name)
        spawn_model(name=object_name, spawn_location=place_location)
        print('Placed the object')
        knapsack_isEmpty = True
        knapsack_object = "empty"

        return True
    else: 
        print('Need to be closer to the location to place the object (and NOT on it!)') 
        return False




if __name__ == '__main__':
    # set up environment

    # initialize_environment()
    # create_scene()
    # print("my location:",gps_location())
    # objects = find_objects()
    # object_name = objects[0][0]
    # object_pose = objects[1][0]
    # print("going to some red ball")
    # movebase_client(object_pose.position.x,object_pose.position.y)
    # # print("picking the ball to knapsack")
    # # pick_object(object_name, [object_pose.position.x,object_pose.position.y])
    # # print("knapsack_is_empty,knapsack_object",knapsack_isEmpty,knapsack_object)
    # # print("going to put it in spot")
    # # movebase_client(0.0,0.0)
    # # print("placing the ball")
    # # place_object(object_name, [0.0,0.0])
    # # print("knapsack_is_empty,knapsack_object",knapsack_isEmpty,knapsack_object)



    print("object name",object_name)
    # print("object position",object_pose)

    print(knapsack_isEmpty)
    print(knapsack_object)
    print("done")

