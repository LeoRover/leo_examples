#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import cv_bridge

import sys
import argparse

import tflite_runtime.interpreter as tflite
import numpy as np


class LineFollower:
    def __init__(self, modelPath=None, velocityTopic=None, videoTopic=None):
        self.bridge = cv_bridge.CvBridge()

        try:
            self.interpreter = tflite.Interpreter(model_path=modelPath)
            self.interpreter.allocate_tensors()
        except ValueError as e:
            rospy.logerr("Couldnt load tflite model: %s" % (modelPath))
            return

        self.get_params()

        self.vel_pub = rospy.Publisher(velocityTopic, Twist, queue_size=10)
        
        self.mask_pub = None
        if self.pub_mask:
            self.mask_pub = rospy.Publisher("color_mask", Image, queue_size=1)

        self.video_sub = rospy.Subscriber(videoTopic, Image, self.video_callback)

    def get_params(self):
        hue_min = rospy.get_param("~hue_min", default=0)
        hue_max = rospy.get_param("~hue_max", default=179)
        sat_min = rospy.get_param("~sat_min", default=0)
        sat_max = rospy.get_param("~sat_max", default=255)
        val_min = rospy.get_param("~val_min", default=0)
        val_max = rospy.get_param("~val_max", default=255)

        self.pub_mask = rospy.get_param("~pub_mask", default=False)

        self.lower = (hue_min, sat_min, val_min)
        self.upper = (hue_max, sat_max, val_max)

    def video_callback(self, data: Image):
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        processed_img = self.preprocess_img(cv_img)

        steering = self.get_steering(processed_img)
        
        if self.pub_mask:
            self.publish_mask(processed_img)

        rospy.logdebug("steering: %f, %f" % (steering[0], steering[1]))
        
        self.publish_vel(steering)

    def publish_vel(self, steering):
        vel_msg = Twist()
        vel_msg.linear.x = steering[0]
        vel_msg.angular.z = steering[1]

        self.vel_pub.publish(vel_msg)

    def publish_mask(self, mask):
        mask *= 255.0
        img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="32FC1")
        self.mask_pub.publish(img_msg)

    def get_steering(self, img):
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()
        self.interpreter.allocate_tensors()

        # providing input
        self.interpreter.set_tensor(input_details[0]["index"], [img])
        # running interferance
        self.interpreter.invoke()
        
        # getting answer
        linear_x = self.interpreter.get_tensor(output_details[0]["index"])[0][0]
        angular_z = self.interpreter.get_tensor(output_details[1]["index"])[0][0]
        rospy.logdebug("pred = (%f, %f)" % (linear_x, angular_z))
        
        return linear_x, angular_z

    def preprocess_img(self, img):
        # changing BGR to HSV
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # cropping img
        crop_img = hsv_img[200 : hsv_img.shape[0], :]
        # getting color mask
        blue_color = cv2.inRange(crop_img, self.lower, self.upper)
        # converting int balues to float
        float_img = blue_color.astype(np.float32)
        # resizing
        resized_img = cv2.resize(float_img, (160, 120))
        # normalize
        final_img = resized_img / 255.0

        return final_img[:, :, np.newaxis]


def add_argparse_arguments(parser: argparse.ArgumentParser):
    parser.add_argument(
        "-p",
        "--path",
        default="~/Downloads/saved_model.tflite",
        type=str,
        required=True,
        dest="model_path",
        help="path to the saved CNN model",
    )
    parser.add_argument(
        "-v",
        "--velocity",
        default="cmd_vel",
        type=str,
        required=False,
        dest="velocity_topic",
        help="name of the topic with Twist messages for rover",
    )
    parser.add_argument(
        "-c",
        "--camera",
        default="camera/image_raw",
        type=str,
        required=False,
        dest="image_topic",
        help="name of the topic with Image messages from rover",
    )


if __name__ == "__main__":
    rospy.init_node("line_follower")

    parser = argparse.ArgumentParser(description="Record video from rover as images.")
    add_argparse_arguments(parser)

    rosargv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(rosargv[1:])

    follower = LineFollower(args.model_path, args.velocity_topic, args.image_topic)
    rospy.spin()
