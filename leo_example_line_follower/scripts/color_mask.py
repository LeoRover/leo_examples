#!/usr/bin/env python
import copy

import rospy
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from leo_example_line_follower.cfg import ColorMaskConfig

import cv2
import cv_bridge


class ColorMask:
    color_mask_config = None

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.mask_func = self.simple_mask

        self.srv = Server(ColorMaskConfig, self.param_callback)

        self.mask_pub = rospy.Publisher("color_mask", Image, queue_size=1)

        self.catched_color_pub = rospy.Publisher(
            "catched_colors/compressed", CompressedImage, queue_size=1
        )

        self.video_sub = rospy.Subscriber(
            "camera/image_raw", Image, self.video_callback
        )

        rospy.loginfo("Starting color_mask_finder node")

    def param_callback(self, config, level):
        self.color_mask_config = config
        self.mask_func = (
            self.simple_mask
            if config.hue_min < config.hue_max
            else self.double_range_mask
        )

        return config

    def video_callback(self, data: Image):
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv_img = cv_img[200 : cv_img.shape[0], :]
        cv_img = cv2.resize(cv_img, (160, 120))

        color_mask = self.get_mask(cv_img)
        catched_colors = get_colors_from_mask(color_mask, cv_img)

        self.publish_imgs(color_mask, catched_colors)

    def get_mask(self, img):
        copy_img = copy.deepcopy(img)
        hsv_img = cv2.cvtColor(copy_img, cv2.COLOR_BGR2HSV)
        mask = self.mask_func(hsv_img)

        return mask

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

    def publish_imgs(self, mask, colors):
        mask_to_ros = self.bridge.cv2_to_imgmsg(mask, encoding="8UC1")
        colors_to_ros = self.bridge.cv2_to_compressed_imgmsg(colors)

        self.mask_pub.publish(mask_to_ros)
        self.catched_color_pub.publish(colors_to_ros)

    def print_vals(self):
        print("Your chosen hsv bounds - copy them to correct yaml file")
        print("hue_min: %d" % (self.color_mask_config.hue_min))
        print("hue_max: %d" % (self.color_mask_config.hue_max))
        print("sat_min: %d" % (self.color_mask_config.sat_min))
        print("sat_max: %d" % (self.color_mask_config.sat_max))
        print("val_min: %d" % (self.color_mask_config.val_min))
        print("val_max: %d" % (self.color_mask_config.val_max))


def get_colors_from_mask(mask, img):
    copy_img = copy.deepcopy(img)
    catched_colors = cv2.bitwise_and(copy_img, copy_img, mask=mask)

    return catched_colors


if __name__ == "__main__":
    rospy.init_node("color_mask_finder")
    cm = ColorMask()
    rospy.on_shutdown(cm.print_vals)
    rospy.spin()
