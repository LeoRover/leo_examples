#!/usr/bin/env python
import copy
import sys
import argparse

import rospy
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from leo_example_object_detection.cfg import ConfidenceConfig

import cv2
import cv_bridge

import tflite_runtime.interpreter as tflite


class ObjectDetector:
    confidence_cfg = None

    def __init__(self, modelPath=None, video_topic=None, labels_file=None):
        self.bridge = cv_bridge.CvBridge()

        try:
            self.interpreter = tflite.Interpreter(model_path=modelPath)
            self.interpreter.allocate_tensors()
        except ValueError as e:
            rospy.logerr("Couldnt load tflite model: %s" % (modelPath))
            return

        self.read_labels(labels_file)

        self.srv = Server(ConfidenceConfig, self.param_callback)

        self.detection_pub = rospy.Publisher(
            "detections/compressed", CompressedImage, queue_size=10
        )

        self.video_sub = rospy.Subscriber(
            video_topic, Image, self.video_callback, queue_size=1
        )

        rospy.loginfo("Starting node")

    def read_labels(self, file):
        self.labels = []
        with open(file, "r") as f:
            lines = f.readlines()
            for line in lines:
                self.labels.append(line.strip())

    def param_callback(self, config, level):
        self.confidence_cfg = config
        return config

    def video_callback(self, data: Image):
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
        processed_img = self.preprocess(cv_img)
        detections = self.detect_objects(processed_img)
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(detections)
            self.detection_pub.publish(msg)
            rospy.loginfo("published")
        except cv_bridge.CvBridgeError() as e:
            rospy.logerror("duppa")

    def preprocess(self, img):
        cpy = copy.deepcopy(img)
        rgb = cv2.cvtColor(cpy, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (300, 300))

        return resized

    def detect_objects(self, img):
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()
        self.interpreter.allocate_tensors()

        # providing input
        self.interpreter.set_tensor(input_details[0]["index"], [img])
        # running interferance
        self.interpreter.invoke()

        # getting answer
        boxes = self.interpreter.get_tensor(output_details[0]["index"])[0]
        labels = self.interpreter.get_tensor(output_details[1]["index"])[0]
        confidence = self.interpreter.get_tensor(output_details[2]["index"])[0]
        img = self.draw_detections(img, boxes, labels, confidence)
        return img

    def draw_detections(self, img, boxes, labels, confidence):
        for box, label, conf in zip(boxes, labels, confidence):
            if int(conf * 100) - self.confidence_cfg.confidence > 0:
                top = int(box[0] * 300)
                left = int(box[1] * 300)
                bottom = int(box[2] * 300)
                right = int(box[3] * 300)

                # border around the detection
                img = cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)

                text_size, _ = cv2.getTextSize(
                    self.labels[int(label)], cv2.FONT_HERSHEY_COMPLEX, 0.5, 1 
                )

                text_w, text_h = text_size
                # border behind label
                cv2.rectangle(
                    img,
                    (left, top + 2),
                    (left + text_w, top + 2 + text_h),
                    (0, 0, 255),
                    -1,
                )

                img = cv2.putText(
                    img,
                    self.labels[int(label)],
                    (left, top + text_h - 1),
                    cv2.FONT_HERSHEY_COMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )

        return img


def add_argparse_arguments(parser: argparse.ArgumentParser):
    parser.add_argument(
        "-p",
        "--path",
        type=str,
        required=True,
        dest="model_path",
        help="path to the object detection model",
    )
    parser.add_argument(
        "-l",
        "--labels",
        type=str,
        required=True,
        dest="labels_path",
        help="name of the file with labels for detected objects",
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
    rospy.init_node("object_detector")

    parser = argparse.ArgumentParser(
        description="Detect objects seen in rover's camera"
    )
    add_argparse_arguments(parser)

    rosargv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(rosargv[1:])

    follower = ObjectDetector(args.model_path, args.image_topic, args.labels_path)
    rospy.spin()
