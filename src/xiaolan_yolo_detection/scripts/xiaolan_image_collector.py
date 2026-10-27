#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class XiaolanImageCollector:

    def __init__(self):

        rospy.init_node(
            "xiaolan_image_collector",
            anonymous=False
        )

        ####################################################
        # Parameters
        ####################################################

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/camera/color/image_raw"
        )

        self.save_dir = os.path.expanduser(
            rospy.get_param(
                "~save_dir",
                "~/xiaolan_dataset/raw"
            )
        )

        self.image_prefix = rospy.get_param(
            "~image_prefix",
            "xiaolan"
        )

        self.start_index = int(
            rospy.get_param(
                "~start_index",
                -1
            )
        )

        ####################################################
        # Init
        ####################################################

        os.makedirs(
            self.save_dir,
            exist_ok=True
        )

        self.bridge = CvBridge()

        self.latest_image = None

        self.saved_count = 0

        ####################################################
        # Determine next image index
        ####################################################

        if self.start_index >= 0:

            self.image_index = self.start_index

        else:

            self.image_index = (
                self.find_next_index()
            )

        ####################################################
        # Subscriber
        ####################################################

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        ####################################################
        # Log
        ####################################################

        rospy.loginfo(
            "=========================================="
        )

        rospy.loginfo(
            "Xiaolan Image Collector"
        )

        rospy.loginfo(
            "Image topic: %s",
            self.image_topic
        )

        rospy.loginfo(
            "Save directory: %s",
            self.save_dir
        )

        rospy.loginfo(
            "Start index: %d",
            self.image_index
        )

        rospy.loginfo(
            "SPACE : save image"
        )

        rospy.loginfo(
            "Q/ESC : quit"
        )

        rospy.loginfo(
            "=========================================="
        )

    ########################################################
    # Find next available index
    ########################################################

    def find_next_index(self):

        max_index = -1

        if not os.path.exists(
            self.save_dir
        ):

            return 0

        for filename in os.listdir(
            self.save_dir
        ):

            if not filename.startswith(
                self.image_prefix + "_"
            ):

                continue

            if not filename.lower().endswith(
                (
                    ".jpg",
                    ".jpeg",
                    ".png"
                )
            ):

                continue

            name_without_ext = os.path.splitext(
                filename
            )[0]

            parts = name_without_ext.split(
                "_"
            )

            if len(parts) < 2:

                continue

            try:

                index = int(
                    parts[-1]
                )

            except ValueError:

                continue

            if index > max_index:

                max_index = index

        return max_index + 1

    ########################################################
    # Image callback
    ########################################################

    def image_callback(
        self,
        msg
    ):

        try:

            image = (
                self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8"
                )
            )

            if (
                image is None
                or image.size == 0
            ):

                return

            self.latest_image = image.copy()

        except Exception as e:

            rospy.logerr_throttle(
                2.0,
                "Image conversion failed: %s",
                str(e)
            )

    ########################################################
    # Save current image
    ########################################################

    def save_image(self):

        if self.latest_image is None:

            rospy.logwarn(
                "No RGB image received yet."
            )

            return

        filename = (
            f"{self.image_prefix}_"
            f"{self.image_index:06d}.jpg"
        )

        path = os.path.join(
            self.save_dir,
            filename
        )

        success = cv2.imwrite(
            path,
            self.latest_image,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                95
            ]
        )

        if not success:

            rospy.logerr(
                "Failed to save image: %s",
                path
            )

            return

        self.saved_count += 1

        rospy.loginfo(
            "Saved [%d] %s",
            self.saved_count,
            path
        )

        self.image_index += 1

    ########################################################
    # Draw UI
    ########################################################

    def draw_overlay(
        self,
        image
    ):

        display = image.copy()

        height, width = (
            display.shape[:2]
        )

        ####################################################
        # Main text
        ####################################################

        cv2.putText(
            display,
            "Xiaolan Dataset Collector",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            f"Saved this session: {self.saved_count}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            display,
            f"Next index: {self.image_index:06d}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            display,
            "SPACE: Save",
            (20, height - 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            display,
            "Q / ESC: Quit",
            (20, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        ####################################################
        # Camera center
        ####################################################

        center_x = width // 2
        center_y = height // 2

        cv2.drawMarker(
            display,
            (
                center_x,
                center_y
            ),
            (0, 255, 0),
            cv2.MARKER_CROSS,
            20,
            2
        )

        return display

    ########################################################
    # Main loop
    ########################################################

    def run(self):

        rate = rospy.Rate(
            30
        )

        window_name = (
            "Xiaolan Image Collector"
        )

        cv2.namedWindow(
            window_name,
            cv2.WINDOW_NORMAL
        )

        while not rospy.is_shutdown():

            if self.latest_image is None:

                rate.sleep()

                continue

            display = self.draw_overlay(
                self.latest_image
            )

            cv2.imshow(
                window_name,
                display
            )

            key = cv2.waitKey(
                1
            ) & 0xFF

            ################################################
            # SPACE
            ################################################

            if key == 32:

                self.save_image()

            ################################################
            # Q / ESC
            ################################################

            elif (
                key == ord("q")
                or
                key == 27
            ):

                rospy.loginfo(
                    "Quit image collector."
                )

                break

            rate.sleep()

        cv2.destroyAllWindows()


############################################################
# Main
############################################################

if __name__ == "__main__":

    try:

        collector = (
            XiaolanImageCollector()
        )

        collector.run()

    except rospy.ROSInterruptException:

        pass
