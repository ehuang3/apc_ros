#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image
from apc_msgs.srv import *
from apc_tools import Image_Subscriber, Bin_Segmenter
import numpy as np
import cv2
import os
import sys
from time import time
fpath = os.path.dirname(os.path.realpath(__file__))

class Proxy_Subscriber(object):
    def __init__(self, resolution='high', rate=0.1):
        self.rate = rate
        self.resolution = resolution

        self.pcl_sub = rospy.Subscriber(
            'kinect2/depth_{}res/points'.format(self.resolution),
            PointCloud2,
            self.cloud_cb,
            queue_size=1
        )

        self.cloud = None
        rospy.logwarn('----waiting for icp service----')
        rospy.wait_for_service('/runICP')
        rospy.logwarn('----found icp service----')
        self.pcl_proxy = rospy.ServiceProxy('/runICP', RunICP)
        self.last_time = time()

    def cloud_cb(self, msg):
        self.cloud = msg
        got_time = time()
        if (got_time - self.last_time) > (1 / self.rate):
            self.pcl_proxy(image(), self.cloud)
            self.last_time = time()


if __name__ == '__main__':
    rospy.init_node('cloud_test_node')

    if not rospy.has_param("/kinect2/serial_number"):
        # rospy.set_param("/kinect2/serial_number", '196605135147')
        rospy.logwarn("Could not find kinect serial on /kinect2/serial_number, guessing a random one: 196605135147")
        kinect_serial = '503233542542'
    else:
        kinect_serial = rospy.get_param("/kinect2/serial_number")

    binseg = Bin_Segmenter(kinect_serial)
    binseg.draw_bin()

    ps = Proxy_Subscriber()

    rospy.spin()