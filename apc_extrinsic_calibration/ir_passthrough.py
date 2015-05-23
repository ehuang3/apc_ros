#!/usr/bin/python
from __future__ import division
from apc_tools import Image_Subscriber, Image_Publisher
import cv2
import numpy as np
import rospy

class IR_Pass(object):
    def __init__(self, in_topic, out_topic):
        rospy.init_node('ir_passthrough')
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.im_sub = Image_Subscriber(in_topic, self.got_image, encoding='16UC1')
        self.im_pub = Image_Publisher(out_topic, encoding='mono8')

    def got_image(self, img):
        cv2.imshow("Image", img)
        self.im_pub.publish(img / (2**8))

        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            exit()


if __name__ == '__main__':
    ipr = IR_Pass(
        in_topic='/kinect2/ir/image',
        out_topic='/kinect2/ir/image_valid'
    )
    rospy.spin()