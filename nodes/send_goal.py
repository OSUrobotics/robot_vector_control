#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Empty

goal_pub = None
goal = None

def int_pt_cb(msg):
    global goal
    goal = PoseStamped()
    goal.header = msg.header
    goal.pose.position = msg.point
    goal.pose.orientation.w = 1
    

def click_cb(msg):
    if goal:
        goal_pub.publish(goal)

if __name__ == '__main__':
    rospy.init_node('publish_goal_from_click')
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped)
    rospy.Subscriber('intersected_point', PointStamped, int_pt_cb)
    rospy.Subscriber('click', Empty, click_cb)
    rospy.spin()