#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge
import cv2
import os
import threading
import re
import numpy as np

class ImagePublisherWithCameraInfo:
    def __init__(self, folder_path, topic_name="/camera/image_raw", rate_hz=0.2):
        self.folder_path = folder_path
        self.topic_name = topic_name
        self.rate_hz = rate_hz
        self.bridge = CvBridge()

        # Setup publishers
        self.pub = rospy.Publisher(self.topic_name, Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)

        # Configure camera info from calibration parameters
        self.camera_info = self.setup_camera_info()

        # 启动服务
        self.service = rospy.Service("/camera/set_camera_info", SetCameraInfo, self.set_camera_info_callback)
        rospy.loginfo("Service /camera/set_camera_info is ready.")

    def setup_camera_info(self):
        """Setup camera info message with the calibration data."""
        camera_info_msg = CameraInfo()

        # Image dimensions
        camera_info_msg.width = 2160
        camera_info_msg.height = 2880

        # Distortion parameters (D)
        camera_info_msg.distortion_model = "fisheye"
        camera_info_msg.D = [0.5382011858036907, -1.860775747877836, 4.32070554055795, -2.028846754921608]

        # Camera matrix (K)
        camera_info_msg.K = [
            1358.3275464606425, 4.887936572731081, 1089.4802196489652,
            0.000000, 1491.8391465377379, 1495.9056125766886,
            0.000000, 0.000000, 1.000000
        ]

        # Rectification matrix (R)
        camera_info_msg.R = [
            1.000000, 0.000000, 0.000000,
            0.000000, 1.000000, 0.000000,
            0.000000, 0.000000, 1.000000
        ]

        # Projection matrix (P)
        camera_info_msg.P = [
            1358.3275464606425, 4.887936572731081, 1089.4802196489652, 0.000000,
            0.000000, 1491.8391465377379, 1495.9056125766886, 0.000000,
            0.000000, 0.000000, 1.000000, 0.000000
        ]

        return camera_info_msg

    def set_camera_info_callback(self, req):
        rospy.loginfo("Received camera info (fake accept).")
        return SetCameraInfoResponse(success=True, status_message="Camera info saved (not really).")

    def _extract_number(self, filename):
        """Extract number from filename for numerical sorting."""
        numbers = re.findall(r'\d+', filename)
        if numbers:
            return int(numbers[0])  # Return the first number found
        return 0  # Return 0 if no number found

    def start_publishing(self):
        # Get all image files
        images = [img for img in os.listdir(self.folder_path) if img.lower().endswith(('.png', '.jpg', '.jpeg'))]
        # Sort by numerical value in filenames
        images.sort(key=self._extract_number)

        rate = rospy.Rate(self.rate_hz)

        for img_name in images:
            if rospy.is_shutdown():
                break
            img_path = os.path.join(self.folder_path, img_name)
            cv_img = cv2.imread(img_path)
            if cv_img is None:
                rospy.logwarn(f"Failed to load image: {img_path}")
                continue

            # Get current timestamp for both messages
            timestamp = rospy.Time.now()

            # Publish image
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = "camera_frame"
            self.pub.publish(img_msg)

            # Publish camera info
            self.camera_info.header.stamp = timestamp
            self.camera_info.header.frame_id = "camera_frame"
            self.camera_info_pub.publish(self.camera_info)

            rospy.loginfo(f"Published: {img_name} with camera info")
            rate.sleep()

        rospy.loginfo("Finished publishing all images.")

if __name__ == "__main__":
    rospy.init_node("image_publisher_with_camera_info")
    folder = rospy.get_param("~image_folder", "/home/eric/Desktop/0417_1030_15")

    # Get rate parameter and ensure it's a float
    rate_param = rospy.get_param("~rate_hz", 5)
    if isinstance(rate_param, (int, float)):
        rate_hz = float(rate_param)
    else:
        rate_hz = 5.0  # Default to 5 Hz if parameter has an unexpected type
        rospy.logwarn(f"Invalid rate parameter: {rate_param}, using default: {rate_hz} Hz")

    node = ImagePublisherWithCameraInfo(folder_path=folder, rate_hz=rate_hz)

    # 用线程跑发布逻辑，这样不会阻塞服务
    threading.Thread(target=node.start_publishing).start()

    rospy.spin()
