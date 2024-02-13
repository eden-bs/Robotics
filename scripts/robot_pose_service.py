#!/usr/bin/env python3

import sys
sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from turtlebot3_gazebo.srv import RobotPose

def gps_location():
    # request a GPS like pose information from the Gazebo server
    rospy.loginfo("Requesting Global Robot Pose from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    me_pose = Pose()
    me_pose = model_state.pose[2]
    me_pose_angles = euler_from_quaternion([me_pose.orientation.x, me_pose.orientation.y, me_pose.orientation.z, me_pose.orientation.w])
    # print('My pose is (x,y,theta): ')
    print(me_pose.position.x, me_pose.position.y, me_pose_angles[2])
    return me_pose.position.x, me_pose.position.y, me_pose_angles[2]

def handle_robot_pose(req):
    # Call the gps_location function to get the robot's current pose
    x, y, theta = gps_location()


    # Return the current pose as a response
    return x,y,theta

def robot_pose_server():
    rospy.init_node('robot_pose_server')
    s = rospy.Service('robot_pose', RobotPose, handle_robot_pose)
    print("Ready to provide robot pose.")
    rospy.spin()

if __name__ == "__main__":
    robot_pose_server()
