#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image
from apc_msgs.srv import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from apc_tools import Image_Subscriber
'''Show the current image, allow the user to draw a bounding box
'''

class Simulated_DPM(object):
    def __init__(self):
        if not rospy.has_param("/kinect2/serial_number"):
            rospy.set_param("/kinect2/serial_number", '196605135147')
            rospy.logerr("SETTING A FAKE KINECT SERIAL")

        rospy.init_node('simulated_dpm')
        cv2.namedWindow("kinect_view")
        cv2.setMouseCallback("kinect_view", self.mouse_call)
        self.im_sub = Image_Subscriber('/kinect2/mono/image', self.got_image)

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.is_done = False
        self.image = None

        self.state = 0
        self.states = ['bottom_right', 'top_right', 'top_left', 'bottom_left']
        self.corners = []

    def draw_corners(self, image):
        for corner in self.corners:
            cv2.circle(image, (corner[0], corner[1]), 5, (0, 0, 255))

    def got_image(self, msg):
        self.image = msg

    def mouse_call(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.is_done:
                self.corners.append(np.array([x, y]))
                self.state += 1
                if self.state >= len(self.states):
                    self.is_done = True
                    self.state = 0
                    # self.corners = []

        if event == cv2.EVENT_RBUTTONDOWN:
            pass

    def run(self):
        rate = rospy.Rate(10)
        while (not rospy.is_shutdown()):

            if self.image is None:
                continue

            image = np.copy(self.image)
            state = self.states[self.state].replace('_', ' ')
            cv2.putText(image, 'Click the {} corner'.format(state), (10, 500), self.font, 4, (255, 100, 80), 2)
            self.draw_corners(image)

            if self.is_done:
                cv2.polylines(image, np.int32([self.corners]), True, (0, 255, 0), 6)

            cv2.imshow("kinect_view", image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                rate.sleep()


if __name__ == '__main__':

    sdpm = Simulated_DPM()
    sdpm.run()