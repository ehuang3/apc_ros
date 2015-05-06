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

class Vision_Server(object):
    def __init__(self):
        rospy.init_node('vision_server')
        rospy.Service('run_vision', RunVision, self.run_vision)

    def run_vision(self, srv):
        print 'Running vision' 
        print 'Got camera ID {}'.format(camer_id)
        for number, _bin in enumerate(srv.bins):
            print '{}: Name: {}'.format(number, _bin)
            position = _bin.bin_pose.position
            size = _bin.bin_size
            print 'Position: ({}, {}, {})\n Size: ({}, {}, {})'.format(
                position.x,
                position.y,
                position.z,
                size.x,
                size.y,
                size.z,
            )
            print 'Contains:'
            for _object in _bin.object_list:
                print '\tID: {} Key: {}'.format(_object.object_id, _object.object_key)

        return RunVisionResponse


if __name__ == '__main__':
    sdpm = Vision_Server()
    rospy.spin()