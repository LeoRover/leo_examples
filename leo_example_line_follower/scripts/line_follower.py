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

class LineFollower():
    def __init__(self, modelPath=None, velocityTopic=None, videoTopic=None):
        self.bridge = cv_bridge.CvBridge()
        
        try:
            self.interpreter = tflite.Interpreter(model_path=modelPath)
            self.interpreter.allocate_tensors()
        except ValueError as e:
            rospy.logerr("Couldnt load tflite model")
            return


        self.vel_pub = rospy.Publisher(velocityTopic, Twist, queue_size=10)

        self.video_sub = rospy.Subscriber(videoTopic, Image, self.video_callback)
        

    def video_callback(self, data:Image):
        rospy.loginfo("got new image")
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        processed_img = self.preprocess_img(cv_img)
        
        #cv2.imshow("current_processed_frame", processed_img)
        #cv2.waitKey(1)

        steering = self.get_steering(processed_img)
        rospy.loginfo("steering: %f, %f" %(steering[0], steering[1]))
        self.publish_vel(steering)

    def debug(self, img):
        msg_img = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.debug_pub.publish(msg_img)

    def publish_vel(self, steering):
        vel_msg = Twist()
        vel_msg.linear.x = steering[0]
        vel_msg.angular.z = steering[1]
        
        self.vel_pub.publish(vel_msg)

    def get_steering(self, img):
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()
        self.interpreter.allocate_tensors()

        # providing input
        self.interpreter.set_tensor(input_details[0]['index'], [img])
        # running interferance
        self.interpreter.invoke()
        #getting answer
        linear_x = self.interpreter.get_tensor(output_details[0]['index'])[0][0]
        angular_z = self.interpreter.get_tensor(output_details[1]['index'])[0][0]
        rospy.loginfo("pred = (%f, %f)" % (linear_x, angular_z))
        return linear_x, angular_z

    def preprocess_img(self, img):
        # changing BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32)
        # cropping image
        img = img[320:img.shape[0], 65:img.shape[1]-90]
        # normalize
        img /= 255.0
        # resizingS
        img = cv2.resize(img, (160, 120))
        
        return img


def add_argparse_arguments(parser: argparse.ArgumentParser):
    parser.add_argument("-p", "--path", default="~/Downloads/saved_model.tflite", type=str, required=True, dest="model_path", help="path to the saved CNN model")
    parser.add_argument("-v", "--velocity", default="cmd_vel", type=str, required=False, dest="velocity_topic", help="name of the topic with Twist messages for rover")
    parser.add_argument("-c", "--camera", default="camera/image_raw", type=str, required=False, dest="image_topic", help="name of the topic with Image messages from rover")


if __name__ == "__main__":
    rospy.init_node("line_follower")

    parser = argparse.ArgumentParser(description="Record video from rover as images.")
    add_argparse_arguments(parser)
   
    rosargv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(rosargv[1:])

    follower = LineFollower(args.model_path, args.velocity_topic, args.image_topic)
    rospy.spin()