#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    # Initialize the ROS node
    rospy.init_node('move_base_goal_publisher')

    # Create a publisher for the move base goal topic
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Create a PoseStamped message
    goal_msg = PoseStamped()

    # Fill in the desired position and orientation of the goal
    goal_msg.header.frame_id = 'map'  # Set the frame ID
    goal_msg.pose.position.x = 0.0    # Set the desired x-coordinate
    goal_msg.pose.position.y = 0.0    # Set the desired y-coordinate
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.w = 1.0  # Set the orientation (quaternion)

    # Publish the message repeatedly (if needed)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        goal_publisher.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
