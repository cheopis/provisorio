#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from line_detection.line_detector import LineDetector

from line_detection.msg import Line2D, Lines

class LineDetectionNode:
    def __init__(self):
        rospy.init_node('line_detection_node', anonymous=True)

        # ===================================================================
        # CONFIGURATION
        # ===================================================================
        self.detector = LineDetector()  # Line detector class
        self.bridge = CvBridge()        # Package to convert image to cv2 data

        # ===================================================================
        # PUBLISHERS
        # ===================================================================
        self.lines_pub = rospy.Publisher(
            '/lines_detection/lines',
            Lines,
            queue_size=10
        )

        self.img_pub = rospy.Publisher(
            '/lines_detection/img_detected',
            Image,
            queue_size=10
        )

        # ===================================================================
        # SUBSCRIBERS
        # ===================================================================
        self.camera_sub = rospy.Subscriber(
            #"/camera/depth/image_raw",
            #"/camera/depth/image_rect_raw",
            "/camera/color/image_raw", 
            Image,
            self.camera_callback)

        
        rospy.loginfo("Line Detection Node Ready.")

    def camera_callback(self, data):
        # Converty the msg received from camera to opencv format
        try:
            #cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)

        # Process image to obtain the lines
        detection, img = self.detector.detect_lines(cv_image)

        # Generate and publish the detection in a topic
        self._publish_line_msg(detection)
        #self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        #self._publish_image(img)


    def _publish_line_msg(self, detection):
        # Creating lines msg
        lines_msg = Lines()
        # Creating Header
        lines_msg.header = Header(stamp=rospy.Time.now(), frame_id="camera")
        # Generating the data with all the lines detected in Line2D format. 
        detected = []
        for line in detection:
            line_msg = Line2D()
            line_msg.x1 = line[0][0]
            line_msg.y1 = line[0][1]
            line_msg.x2 = line[1][0]
            line_msg.y2 = line[1][1]

            line_msg.theta = line[2]

            line_msg.center_x = line[3][0]
            line_msg.center_y = line[3][1]

            detected.append(line_msg)
        lines_msg.lines = detected

        # Publish lines detection msg
        self.lines_pub.publish(lines_msg)

    def _publish_image(self,cv_image):
        # Creating image msg
        img_msg = Image()

        # Creating Header
        img_msg.header = header = Header(stamp=rospy.Time.now(), frame_id="camera")

        # Creating msg body
        # Img parameters
        img_msg.height = 0
        img_msg.width = 0

        # Msg parameters
        img_msg.encoding = None
        img_msg.is_bigendian = None
        img_msg.step = None

        # Img data
        img_msg.data = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

if __name__ == '__main__':
    node = LineDetectionNode()
    rospy.spin()