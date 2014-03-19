#!/usr/bin/env python
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from robot_vector_control.marker_helpers import make6DofMarker
from geometry_msgs.msg import PoseStamped
import tf

translation = None
rotation = None

def processFeedback(feedback):
    global translation
    global rotation
    p = feedback.pose.position
    r = feedback.pose.orientation
    translation = p.x, p.y, p.z
    rotation = r.x, r.y, r.z, r.w

if __name__=="__main__":
    rospy.init_node("simple_marker")
    
    tfb = tf.TransformBroadcaster()
    pose_pub = rospy.Publisher('head_pose', PoseStamped)

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")
    parent_frame_id = '/odom'
    server.insert(make6DofMarker(parent_frame_id, fixed=True), processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    pose = PoseStamped()
    pose.header.frame_id = 'face_detection'
    pose.pose.orientation.w = 1

    tf_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if translation and rotation:
            stamp = rospy.Time.now()
            tfb.sendTransform(translation, rotation, stamp, 'face_detection', parent_frame_id)
            pose.header.stamp = stamp
            pose_pub.publish(pose)
        tf_rate.sleep()