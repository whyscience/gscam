#!/usr/bin/env python

import rospy
import cv2
import os
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge, CvBridgeError
import yaml

class IPCamNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ip_cam_node', anonymous=True)

        # Parameters
        self.video_url = rospy.get_param('~video_url', 'http://192.168.10.144:4747/video')
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        self.image_topic = rospy.get_param('~image_topic', '/ip_cam/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/ip_cam/camera_info')
        self.calibration_file = rospy.get_param('~calibration_file', os.path.expanduser('~/.ros/camera_info/ip_cam.yaml'))

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create publishers
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10)

        # Initialize camera info
        self.camera_info = CameraInfo()
        self.load_camera_info()

        # Create service
        self.set_camera_info_srv = rospy.Service('/camera/set_camera_info', SetCameraInfo, self.handle_set_camera_info)

        # Initialize video capture
        self.cap = cv2.VideoCapture(self.video_url)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open video stream: %s", self.video_url)
            return

        # Set publishing rate (e.g., 30 Hz)
        self.rate = rospy.Rate(30)

    def load_camera_info(self):
        """Load camera info from calibration file if it exists."""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    calib_data = yaml.safe_load(f)
                    if calib_data:
                        self.camera_info = CameraInfo()
                        self.camera_info.header.frame_id = self.frame_id
                        self.camera_info.width = calib_data.get('image_width', 0)
                        self.camera_info.height = calib_data.get('image_height', 0)
                        self.camera_info.K = calib_data.get('camera_matrix', {}).get('data', [0]*9)
                        self.camera_info.D = calib_data.get('distortion_coefficients', {}).get('data', [0]*5)
                        self.camera_info.R = calib_data.get('rectification_matrix', {}).get('data', [0]*9)
                        self.camera_info.P = calib_data.get('projection_matrix', {}).get('data', [0]*12)
                        self.camera_info.distortion_model = calib_data.get('distortion_model', 'plumb_bob')
                        rospy.loginfo("Loaded camera info from %s", self.calibration_file)
            except Exception as e:
                rospy.logerr("Failed to load camera info: %s", e)

    def handle_set_camera_info(self, req):
        """Handle SetCameraInfo service request."""
        try:
            # Update camera info
            self.camera_info = req.camera_info
            self.camera_info.header.frame_id = self.frame_id

            # Save to file
            calib_data = {
                'image_width': self.camera_info.width,
                'image_height': self.camera_info.height,
                'camera_name': rospy.get_name(),
                'distortion_model': self.camera_info.distortion_model,
                'distortion_coefficients': {'data': self.camera_info.D},
                'camera_matrix': {'data': self.camera_info.K},
                'rectification_matrix': {'data': self.camera_info.R},
                'projection_matrix': {'data': self.camera_info.P}
            }

            # Ensure directory exists
            calib_dir = os.path.dirname(self.calibration_file)
            if not os.path.exists(calib_dir):
                os.makedirs(calib_dir)

            # Write to YAML file
            with open(self.calibration_file, 'w') as f:
                yaml.safe_dump(calib_data, f)

            rospy.loginfo("Saved camera info to %s", self.calibration_file)
            return SetCameraInfoResponse(success=True, status_message="Camera info set and saved successfully.")

        except Exception as e:
            rospy.logerr("Failed to set camera info: %s", e)
            return SetCameraInfoResponse(success=False, status_message=str(e))

    def run(self):
        while not rospy.is_shutdown():
            try:
                # Read frame from video stream
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logwarn("Failed to grab frame")
                    continue

                # Convert OpenCV image to ROS Image message
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.header.frame_id = self.frame_id
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: %s", e)
                    continue

                # Update and publish camera info
                self.camera_info.header.stamp = image_msg.header.stamp
                self.camera_info_pub.publish(self.camera_info)

                # Publish the image
                self.image_pub.publish(image_msg)

            except Exception as e:
                rospy.logerr("Error processing frame: %s", e)

            self.rate.sleep()

        # Release the capture when done
        self.cap.release()

if __name__ == '__main__':
    try:
        node = IPCamNode()
        node.run()
    except rospy.ROSInterruptException:
        pass