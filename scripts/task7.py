#!/usr/bin/env python3
import sys
import time
sys.path.append('/home/robotics1/catkin_ws/src/py-code')

sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
import rospy
from environment_functions import create_scene,goal_checker
from move_to_goal_service import movebase_client
from pick_objects_nearby_service import pick_object
from place_objects_service import place_object
from robot_pose_service import gps_location
from sense_objects_service import find_objects
# from goal_checker_service import goal_checker
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Point
from turtlebot3_gazebo.srv import SenseObjects, SenseObjectsRequest, RobotPose, MoveToGoal, MoveToGoalRequest,PickObjectsNearby,PlaceObjects
import math


def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist

def get_robot_pose():
    rospy.wait_for_service('/robot_pose')
    try:
        robot_pose_proxy = rospy.ServiceProxy('/robot_pose', RobotPose)
        response = robot_pose_proxy()
        return response
    except rospy.ServiceException as e:
        print("Service call failed:", e)


def index_of_min_distance(distances):
    min_dist = float('inf')
    min_dist_index = -1
    for i in range(len(distances)):
        if distances[i] != -1 and distances[i] < min_dist:
            min_dist = distances[i]
            min_dist_index = i
    return min_dist_index

def navigate_to_goal(object_name, object_x, object_y):
    rospy.loginfo("Navigating to {} located at ({}, {}).".format(object_name, object_x, object_y))

    # Create a MoveToGoal service client
    rospy.wait_for_service('/navigate_to_point')
    try:
        navigate_to_point_proxy = rospy.ServiceProxy('/navigate_to_point', MoveToGoal)

        goal_x = object_x
        goal_y = object_y
        # radians_theta = calculate_theta(robot_x,robot_y,goal_x,goal_y)

        
        # Call the MoveToGoal service
        success = movebase_client(goal_x,goal_y)
        # success = navigate_to_point_proxy(goal_x,goal_y,radians_theta)

        if success:
            rospy.loginfo("Navigation to {} successful.".format(object_name))
            return True
        else:
            rospy.logerr("Failed to navigate to {}.".format(object_name))
            return False
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def pick_up_ball(ball_name, ball_x,ball_y):
    rospy.wait_for_service('/pick_object_nearby')
    try:
        # Create a service proxy
        pick_object_proxy = rospy.ServiceProxy('/pick_object_nearby', PickObjectsNearby)
        
        # Call the service to pick up the ball
        response = pick_object_proxy(object_name=ball_name, object_location=[ball_x,ball_y])
        
        if response.success:
            return True
            rospy.loginfo("Successfully picked up the ball: {}".format(ball_name))
        else:
            rospy.logerr("Failed to pick up the ball: {}".format(ball_name))
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))


def place_object(object_name, place_location):
    rospy.wait_for_service('/place_object')
    try:
        place_object_proxy = rospy.ServiceProxy('/place_object', PlaceObjects)
        response = place_object_proxy(object_name=object_name, place_location=place_location)


        if response.success:
            rospy.loginfo("Object {} placed successfully at location: {}".format(object_name, place_location))
            return True
        else:
            rospy.logerr("Failed to place object {} at location: {}".format(object_name, place_location))
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def task_execution():
    rospy.init_node('task_executor')
    collected_balls = []
    create_scene()
    time.sleep(2)

    while not rospy.is_shutdown():

        # Call SenseObjects service to get the current positions of all red balls
        rospy.wait_for_service('/sense_objects')
        eps = 0.03
        sense_objects_proxy = rospy.ServiceProxy('/sense_objects', SenseObjects)
        sense_objects_request = SenseObjectsRequest()
        sense_objects_response = sense_objects_proxy(sense_objects_request)

        object_names = sense_objects_response.object_names
        ball_positions_x = sense_objects_response.object_positions_x
        ball_positions_y = sense_objects_response.object_positions_y
        distances = [0] * len(object_names)

        robot_pose = get_robot_pose()
        robot_x = robot_pose.x
        robot_y = robot_pose.y

        for i in range(len(object_names)):
            if(object_names[i] == 'blue_cube'):
                distances[i] = -1
                continue
            else:
                ball_x = ball_positions_x[i]
                ball_y = ball_positions_y[i]
                distances[i] = distance(ball_x,ball_y,robot_x,robot_y)
        for j in range(4):
            min_dist_index = index_of_min_distance(distances)
            if min_dist_index == -1:
                break
            ball_name = object_names[min_dist_index]
            ball_x = ball_positions_x[min_dist_index]
            ball_y = ball_positions_y[min_dist_index]
            print("closest ball info:",ball_name,ball_x,ball_y)
            # ball_x - 0.01 so the robot will not push the ball when arriving
            flag = navigate_to_goal(ball_name,ball_x-0.1,ball_y)
            if not flag:
                rospy.signal_shutdown("couldnt navigate to the ball")
                break
            time.sleep(3)
            flag = pick_up_ball(ball_name,ball_x,ball_y)
            if not flag:
                rospy.signal_shutdown("couldnt pick up the ball")
                break
            time.sleep(3)
                

            #go to place location
            flag = navigate_to_goal(ball_name,-3.23,0.595)
            if not flag:
                rospy.signal_shutdown("couldnt go back to place")
                break
            time.sleep(3)

            flag = place_object(ball_name,[-3.25,0.42,0.1])

            if not flag:
                rospy.signal_shutdown("couldnt place ball in place")
                break
            time.sleep(3)
            collected_balls.append(ball_name)
            distances[min_dist_index] = -1
        if goal_checker(collected_balls):
            print("All the red balls were collected")
        else:
            print("Not all the balls were collected")

        break
            

if __name__ == "__main__":
    try:
        task_execution()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted by user!")