#!/usr/bin/env python

import math

import rospy

from geometry_msgs.msg import Twist, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers

rospy.init_node("follow_ar_tag")

follow_id = rospy.get_param("~follow_id", 0)
marker_timeout = rospy.Duration(rospy.get_param("~marker_timeout", 0.5))
min_ang_vel = rospy.get_param("~min_ang_vel", 0.1)
max_ang_vel = rospy.get_param("~max_ang_vel", 1.0)
angle_min = rospy.get_param("~angle_min", 0.1)
angle_max = rospy.get_param("~angle_max", 0.7)

last_ts = rospy.Time()
last_position = Vector3()
twist_cmd = Twist()


def ar_pose_callback(msg):
    global last_ts, last_position
    for marker in msg.markers:
        if marker.id != follow_id:
            continue
        if marker.header.stamp < last_ts:
            rospy.logwarn_throttle(3.0, "Got marker position with an older timestamp")
            continue

        last_ts = marker.header.stamp
        last_position = marker.pose.pose.position


cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
ar_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_pose_callback)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if last_ts + marker_timeout < rospy.get_rostime():
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
    else:
        # Get the angle to the marker
        angle = math.atan(last_position.y / last_position.x)

        # Check direction
        if angle < 0.0:
            dir = -1.0
            angle *= -1.0
        else:
            dir = 1.0

        # Check angle threshold
        if angle < angle_min:
            ang_cmd = 0.0
        else:
            if angle > angle_max:
                angle = angle_max

            angle_scaled = (angle - angle_min) / (angle_max - angle_min)
            ang_cmd = min_ang_vel + angle_scaled * (max_ang_vel - min_ang_vel)

        twist_cmd.angular.z = dir * ang_cmd

    cmd_vel_pub.publish(twist_cmd)
    rate.sleep()
