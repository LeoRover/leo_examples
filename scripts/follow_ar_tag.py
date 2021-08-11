#!/usr/bin/env python

import math

import rospy

from geometry_msgs.msg import Twist, TwistStamped, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers


class ARTagFollower:
    def __init__(self):
        self.last_ts = rospy.Time()
        self.last_position = Vector3()
        self.twist_cmd = Twist()

        self.get_parameters()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.ar_pose_sub = rospy.Subscriber(
            "ar_pose_marker", AlvarMarkers, self.callback_ar_pose
        )
        self.wheel_odom_sub = rospy.Subscriber(
            "wheel_odom", TwistStamped, self.callback_wheel_odom
        )

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_cmd()
            self.cmd_vel_pub.publish(self.twist_cmd)
            rate.sleep()

    def update_cmd(self):
        # Check for a timeout
        if self.last_ts + self.marker_timeout < rospy.get_rostime():
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = 0.0
            return

        # Get the angle to the marker
        angle = math.atan(self.last_position.y / self.last_position.x)

        # Check direction
        if angle < 0.0:
            dir = -1.0
            angle *= -1.0
        else:
            dir = 1.0

        # Check angle threshold
        if angle < self.angle_min:
            ang_cmd = 0.0
        else:
            if angle > self.angle_max:
                angle = self.angle_max

            angle_scaled = (angle - self.angle_min) / (self.angle_max - self.angle_min)
            ang_cmd = self.min_ang_vel + angle_scaled * (
                self.max_ang_vel - self.min_ang_vel
            )

        self.twist_cmd.angular.z = dir * ang_cmd

    def get_parameters(self):
        self.follow_id = rospy.get_param("~follow_id", 0)
        self.marker_timeout = rospy.Duration(rospy.get_param("~marker_timeout", 0.5))
        self.min_ang_vel = rospy.get_param("~min_ang_vel", 0.1)
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 1.0)
        self.angle_min = rospy.get_param("~angle_min", 0.1)
        self.angle_max = rospy.get_param("~angle_max", 0.7)

    def callback_ar_pose(self, msg):
        for marker in msg.markers:
            if marker.id != self.follow_id:
                continue
            if marker.header.stamp < self.last_ts:
                rospy.logwarn_throttle(
                    3.0, "Got marker position with an older timestamp"
                )
                continue

            self.last_ts = marker.header.stamp
            self.last_position = marker.pose.pose.position

    def callback_wheel_odom(self, msg):
        pass


if __name__ == "__main__":
    rospy.init_node("follow_ar_tag")
    ar_tag_follower = ARTagFollower()
    ar_tag_follower.run()
