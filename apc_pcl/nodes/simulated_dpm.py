#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image
from apc_msgs.srv import *
from apc_msgs.msg import DPMObject

import cv2
import numpy as np
from apc_tools import get_image_msg
'''Show the current image, allow the user to draw a bounding box
'''

class Simulated_DPM(object):
    def __init__(self):
        if not rospy.has_param("/kinect2/serial_number"):
            rospy.set_param("/kinect2/serial_number", '503233542542')
            rospy.logerr("SETTING A FAKE KINECT SERIAL")

        rospy.init_node('simulated_dpm')
        rospy.Service('/run_dpm_simulated', SegmentImage, self.run_dpm)

        cv2.namedWindow("kinect_view")
        cv2.setMouseCallback("kinect_view", self.mouse_call)

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.is_done = False
        self.image = None

        self.state = 0
        self.states = ['bottom_right', 'top_right', 'top_left', 'bottom_left']
        self.corners = []

    def draw_corners(self, image):
        for corner in self.corners:
            cv2.circle(image, (corner[0], corner[1]), 5, (0, 0, 255))

    def run_dpm(self, srv):
        self.image = get_image_msg(srv.image)
        print 'Executing on image of size {}'.format(self.image.shape)
        self.target_object = srv.object_id
        self.run()
        x, y, w, h = cv2.boundingRect(np.array([np.array(self.corners)]))
        self.corners = []
        self.state = 0
        self.is_done = False
        self.image = None
        return SegmentImageResponse(
            x=x,
            y=y,
            height=h,
            width=w,
            success=True,
        )
        

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

        if event == cv2.EVENT_RBUTTONDOWN:
            pass

    def run(self):
        rate = rospy.Rate(10)
        done = False
        cv2.namedWindow("kinect_view")
        cv2.setMouseCallback("kinect_view", self.mouse_call)

        while (not rospy.is_shutdown() and not done):

            if self.image is None:
                continue

            image = np.copy(self.image)
            state = self.states[self.state].replace('_', ' ')
            cv2.putText(image, 'Click the {}'.format(self.target_object), (10, self.image.shape[1] - 100), self.font, 1, (255, 100, 80), 2)
            self.draw_corners(image)

            if self.is_done:
                cv2.polylines(image, np.int32([self.corners]), True, (0, 255, 0), 6)
                done = True
                print 'DONE'

            cv2.imshow("kinect_view", image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                rate.sleep()

            if done:
                cv2.destroyWindow("kinect_view")


if __name__ == '__main__':

    sdpm = Simulated_DPM()
    rospy.spin()