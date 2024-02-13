#!/usr/bin/env python3

import sys
sys.path.append('/home/robotics1/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
from turtlebot3_gazebo.srv import MoveToGoal
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  


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
 eturn False

def handle_navigate_to_point(req):
    # Extract x, y, and theta from the request
    x = req.x
    y = req.y
    theta = req.theta

    # Call the movebase_client function with the provided coordinates
    success = movebase_client(x, y, theta)

    # Return success status
    return {'success': success}

def navigate_to_point_server():
    rospy.init_node('navigate_to_point_server')
    rospy.loginfo("Ready to navigate to a point.")
    rospy.Service('navigate_to_point', MoveToGoal, handle_navigate_to_point)
    rospy.spin()

if __name__ == "__main__":
    try:
        navigate_to_point_server()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted by user!")