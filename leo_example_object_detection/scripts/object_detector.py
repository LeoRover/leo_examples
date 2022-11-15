#!/usr/bin/env python3
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
        except ValueError as e:
            rospy.logerr("Could not load tflite model: %s" % (modelPath))
            raise

        input_details = self.interpreter.get_input_details()
        self.input_shape = tuple(input_details[0]["shape"][1:3].tolist())
        self.interpreter.allocate_tensors()

        self.read_labels(labels_file)
        self.build_color_dict()
        self.scales = False

        self.srv = Server(ConfidenceConfig, self.dynamic_param_callback)

        self.detection_pub = rospy.Publisher(
            "detections/compressed", CompressedImage, queue_size=10
        )

        self.video_sub = rospy.Subscriber(
            video_topic, Image, self.video_callback, queue_size=1
        )

        rospy.loginfo("Starting node")

    def get_scales(self, img):
        self.final_height = img.shape[0]
        self.final_width = img.shape[1]

        self.scale_x = self.final_width / self.input_shape[0]
        self.scale_y = self.final_height / self.input_shape[1]

        self.scales = True

    def translate_point(self, point):
        x_translated = point[0] * self.scale_x
        y_translated = point[1] * self.scale_y

        return (int(x_translated), int(y_translated))

    def read_labels(self, file):
        self.labels = []
        with open(file, "r") as f:
            lines = f.readlines()
            for line in lines:
                self.labels.append(line.strip())

    def build_color_dict(self):
        self.label_colors = dict()
        labels = rospy.get_param("~labels")
        colors = rospy.get_param("~colors")

        for label, color in zip(labels, colors):
            self.label_colors[label] = tuple(color)

    def dynamic_param_callback(self, config, level):
        self.confidence_cfg = config
        return config

    def video_callback(self, data: Image):
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        if not self.scales:
            self.get_scales(cv_img)

        processed_img = self.preprocess(cv_img)
        boxes, labels, confidences = self.detect_objects(processed_img)
        final_boxes = self.get_boxes(boxes, labels, confidences)
        final_img = self.draw_detections(cv_img, final_boxes)

        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(final_img, "jpeg")
            self.detection_pub.publish(msg)
        except cv_bridge.CvBridgeError() as e:
            rospy.logerror(e)

    def preprocess(self, img):
        cpy = copy.deepcopy(img)
        rgb = cv2.cvtColor(cpy, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, self.input_shape)

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
        # img = self.draw_detections(img, boxes, labels, confidence)
        return boxes, labels, confidence

    def get_boxes(self, boxes, labels, confidence):
        final_boxes = []
        for box, label, conf in zip(boxes, labels, confidence):
            if int(conf * 100) - self.confidence_cfg.confidence > 0:
                top = int(box[0] * self.input_shape[1])
                left = int(box[1] * self.input_shape[0])
                bottom = int(box[2] * self.input_shape[1])
                right = int(box[3] * self.input_shape[0])

                color = self.label_colors.get(self.labels[int(label)], (0, 0, 102))

                text = self.labels[int(label)] + " " + str(round(conf * 100, 2)) + "%"

                final_start_point = self.translate_point((left, top))
                final_end_point = self.translate_point((right, bottom))

                final_boxes.append((final_start_point, final_end_point, text, color))

        return final_boxes

    def draw_detections(self, img, final_boxes):
        for start, end, text, color in final_boxes:
            # main border
            img = cv2.rectangle(img, start, end, color, 2)

            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_COMPLEX, 1, 1)
            text_w, text_h = text_size

            # text background
            cv2.rectangle(
                img,
                (start[0], start[1] + 2),
                (start[0] + text_w, start[1] + 2 + text_h),
                color,
                -1,
            )

            img = cv2.putText(
                img,
                text,
                (start[0], start[1] + text_h - 1),
                cv2.FONT_HERSHEY_COMPLEX,
                1,
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
