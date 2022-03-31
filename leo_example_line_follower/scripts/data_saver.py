#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import cv_bridge

import sys
import argparse
import datetime
from pathlib import Path



class DataSaver():
    def __init__(self, duration, video_topic, vel_topic, output_dir):
        self.duration = duration
        self.video_topic = video_topic
        self.vel_topic = vel_topic
        self.output_dir = output_dir

        self.bridge = cv_bridge.CvBridge()
        
        self.ready = False
        self.end_time = 0#rospy.Time.now() + rospy.Duration.from_sec(self.duration)
       

        rospy.loginfo("Making directory for saved images (if it doesn't exist).")
        self.path = "./" + self.output_dir
        Path(self.path).mkdir(parents=True, exist_ok=True)
        
        date = datetime.datetime.now()
        self.base_name = self.path + "/%s%s%s%s%s-img" % (date.day, date.month, date.year, date.hour, date.minute)

        rospy.loginfo("Opening label file (creating if deons't exist).")
        self.label_file = open(self.path +  "/labels.txt", "a+")
       
        self.counter = 0
        
        self.video_sub = rospy.Subscriber(self.video_topic, Image, self.data_callback)
        self.vel_sub = rospy.Subscriber(self.vel_topic, Twist, self.vel_callback)


    def data_callback(self, data: Image):
        if not self.ready:
            rospy.loginfo("Wating for twist msg.")
            return

        if self.end_time <= rospy.Time.now():
            rospy.loginfo("Saved enough data. Finishing node.")
            self.label_file.close()
            rospy.signal_shutdown("Saved enough data. Finishing node.")

        
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        img_name = self.base_name + str(self.counter) + ".jpg"
        rospy.loginfo("{}:{}".format(img_name, self.label))
        #cv2.imwrite(filename=img_name, img=cv_img, params=[cv2.IMWRITE_JPEG_QUALITY, 100])
        #self.label_file.write("{}:{}".format(img_name, self.label))
        self.counter += 1
        

    def vel_callback(self, data:Twist):
        if not self.ready:
            self.ready = True
            self.end_time = rospy.Time.now() + rospy.Duration.from_sec(self.duration)
        self.label = (data.linear.x, data.angular.z)


def add_arguments(parser: argparse.ArgumentParser):
    parser.add_argument("duration", default=20.0, type=float, nargs="?", metavar="N", help="duration of recording the data")
    parser.add_argument("-v", "--vid", default="camera/image_raw", type=str, required=False, dest="video_topic", help="name of the topic with view from robot")
    parser.add_argument("-V", "--vel", default="cmd_vel", type=str, required=False, nargs="?", dest="vel_topic", help="name of the velocity topic")
    parser.add_argument("-o", "--output", default="data", type=str, required=False, nargs="?", dest="output_dir", help="name of an output directory for saved images")


if __name__ == "__main__":
    rospy.init_node("data_saver")

    parser = argparse.ArgumentParser(description="Record video from rover as images.")
    add_arguments(parser)
   
    rosargv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(rosargv[1:])

    saver = DataSaver(args.duration, args.video_topic, args.vel_topic, args.output_dir)
    rospy.spin()
    