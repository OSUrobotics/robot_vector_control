#!/usr/bin/env python
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from robot_vector_control.marker_helpers import make6DofMarker
from geometry_msgs.msg import PoseStamped
import tf
import sys

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
    if len(sys.argv) >= 3:
        rospy.init_node("simple_marker")
        publish_pose = rospy.get_param('~publish_pose', True)
        tfb = tf.TransformBroadcaster()
        pose_pub = rospy.Publisher('head_pose', PoseStamped)

        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("simple_marker")
        parent_frame_id = sys.argv[1]
        server.insert(make6DofMarker(parent_frame_id, fixed=True), processFeedback)

        # 'commit' changes and send to all clients
        server.applyChanges()

        pose = PoseStamped()
        pose.header.frame_id = sys.argv[2]
        pose.pose.orientation.w = 1

        tf_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if translation and rotation:
                stamp = rospy.Time.now()
                tfb.sendTransform(translation, rotation, stamp, 'face_detection', parent_frame_id)
                if publish_pose:
                    pose.header.stamp = stamp
                    pose_pub.publish(pose)
            tf_rate.sleep()
    else:
        print 'Usage: interactive_vector_marker.py [parent frame id] [child frame id]'