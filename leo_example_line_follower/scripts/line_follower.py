#!/usr/bin/env python
from re import S
import sys
import argparse

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from leo_example_line_follower.cfg import ColorMaskConfig
import cv2
import cv_bridge

import tflite_runtime.interpreter as tflite
import numpy as np


class LineFollower:
    color_mask_config = None

    def __init__(self, modelPath=None, velocityTopic=None, videoTopic=None):
        self.bridge = cv_bridge.CvBridge()

        try:
            self.interpreter = tflite.Interpreter(model_path=modelPath)
            self.interpreter.allocate_tensors()
        except ValueError as e:
            rospy.logerr("Couldnt load tflite model: %s" % (modelPath))
            return

        self.mask_func = self.simple_mask
        
        self.srv = Server(ColorMaskConfig, self.param_callback)

        self.pub_mask = rospy.get_param("~pub_mask", default=False)

        self.vel_pub = rospy.Publisher(velocityTopic, Twist, queue_size=10)

        self.mask_pub = None
        if self.pub_mask:
            self.mask_pub = rospy.Publisher("color_mask", Image, queue_size=1)

        self.video_sub = rospy.Subscriber(videoTopic, Image, self.video_callback)

    def simple_mask(self, img):
        color_mask = cv2.inRange(
            img,
            (
                self.color_mask_config.hue_min,
                self.color_mask_config.sat_min,
                self.color_mask_config.val_min,
            ),
            (
                self.color_mask_config.hue_max,
                self.color_mask_config.sat_max,
                self.color_mask_config.val_max,
            ),
        )
        return color_mask

    def double_range_mask(self, img):
        # hue_min > hue_max, so there are two masks:
        # 1) 0-hue_max
        # 2) hue_min - 180
        lower_mask = cv2.inRange(
            img,
            (
                0,
                self.color_mask_config.sat_min,
                self.color_mask_config.val_min,
            ),
            (
                self.color_mask_config.hue_max,
                self.color_mask_config.sat_max,
                self.color_mask_config.val_max,
            ),
        )

        upper_mask = cv2.inRange(
            img,
            (
                self.color_mask_config.hue_min,
                self.color_mask_config.sat_min,
                self.color_mask_config.val_min,
            ),
            (
                179,
                self.color_mask_config.sat_max,
                self.color_mask_config.val_max,
            ),
        )

        final_mask = lower_mask + upper_mask

        return final_mask
    
    
    def param_callback(self, config, level):
        self.color_mask_config = config
        self.mask_func = (
            self.simple_mask
            if config.hue_min < config.hue_max
            else self.double_range_mask
        )


        return config

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
        color_mask = self.mask_func(crop_img)
        # converting int balues to float
        float_img = color_mask.astype(np.float32)
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
