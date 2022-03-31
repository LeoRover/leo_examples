#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
import cv_bridge
import datetime

import cv2


class DataSaver():
    def __init__(self):
        self.pkg_path = self.get_package_path()
        self.get_params()
        self.bridge = cv_bridge.CvBridge()
        
        self.end_time = rospy.Time.now() + rospy.Duration.from_sec(self.duration)
       
        date = datetime.datetime.now()
        self.base_name = "%s%s%s%s%s-img" % (date.day, date.month, date.year, date.hour, date.minute)


        self.path = self.pkg_path + "/" + self.output_dir + "/"

        self.label_file = open(self.path +  "labels.txt", "a+")
       
        self.counter = 0
        
        self.video_sub = rospy.Subscriber(self.video_topic, Image, self.data_callback)
        self.vel_sub = rospy.Subscriber(self.vel_topic, TwistStamped, self.vel_callback)


    def data_callback(self, data: Image):
        if self.end_time <= rospy.Time.now():
            rospy.signal_shutdown("Saved enough data. Finnishing node")

        rospy.loginfo("got")
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #cv2.imwrite(filename=self.base_name + str(self.counter) + ".jpg", img=cv_img, params=[cv2.IMWRITE_JPEG_QUALITY, 100])
        self.counter += 1
        

    def vel_callback(self, data:TwistStamped):
        self.last_msg = data

    def get_package_path(self):
        packages = rospkg.get_ros_paths()
        for package in packages:
            if "leo_example_line_follower" in package:
                rospy.loginfo("Found path")
                return package
        
        rospy.logerr("Couldn't get path for leo_example_line_follower package")
        exit(0)

    def get_params(self):
        self.duration = rospy.get_param("duration", 20.0)
        self.video_topic = rospy.get_param("video_topic", "camera/image_raw")
        self.vel_topic = rospy.get_param("velocity_topic", "cmd_vel")        
        self.output_dir = rospy.get_param("output", "data")







if __name__ == "__main__":
    rospy.init_node("data_saver")
    saver = DataSaver()
    rospy.spin()
    