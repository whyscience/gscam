#!/usr/bin/env python
import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from optparse import OptionParser
from camera_calibration.calibrator import MonoCalibrator
from camera_calibration.parsing import parse_calibration_pattern
from camera_calibration.calibrator import Patterns
import message_filters

class LocalImageCalibrator:
    def __init__(self, image_folder, pattern, size, square):
        self.image_folder = image_folder
        self.pattern, self.rows, self.cols = parse_calibration_pattern(pattern, size)
        self.square = square
        self.calibrator = MonoCalibrator((self.rows, self.cols), self.square, self.pattern)
        self.bridge = CvBridge()

    def run(self):
        images = sorted([f for f in os.listdir(self.image_folder) if f.lower().endswith(('.jpg', '.jpeg', '.png'))])
        for img_file in images:
            img_path = os.path.join(self.image_folder, img_file)
            cv_img = cv2.imread(img_path)
            if cv_img is None:
                rospy.logwarn(f"Failed to load image {img_path}")
                continue

            rospy.loginfo(f"Processing {img_path}")
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            self.calibrator.handle_image(gray)

        if len(self.calibrator.good_corners) == 0:
            rospy.logerr("No corners were found in any image.")
            return

        self.calibrator.calibrate()
        self.calibrator.report()
        self.calibrator.write_camera_info("file:///tmp/camera_calibration.yaml")
        rospy.loginfo("Calibration saved to /tmp/camera_calibration.yaml")


def main():
    parser = OptionParser()
    parser.add_option("--folder", dest="image_folder", help="Folder with calibration images")
    parser.add_option("--size", dest="size", help="Pattern size, e.g. 8x6", default="8x6")
    parser.add_option("--square", dest="square", type="float", help="Square size in meters", default=0.024)
    parser.add_option("--pattern", dest="pattern", help="Calibration pattern: chessboard, circles, acircles", default="chessboard")

    (options, args) = parser.parse_args()

    if not options.image_folder:
        print("You must specify --folder pointing to the image directory")
        return

    rospy.init_node("local_image_calibrator", anonymous=True)
    calibrator = LocalImageCalibrator(
        image_folder=options.image_folder,
        pattern=options.pattern,
        size=options.size,
        square=options.square
    )
    calibrator.run()

if __name__ == '__main__':
    main()
