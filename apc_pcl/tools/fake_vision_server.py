#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped
from apc_msgs.srv import *
from apc_msgs.msg import BinState, ObjectState, DPMObject
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rospy.numpy_msg import numpy_msg
from apc_tools import (Image_Subscriber, Bin_Segmenter, xyzharray, xyzarray, xyzwarray, pqfrompose, pose_from_matrix,
    make_image_msg, load_background, path_to_root)
import tf
import os
import rosbag


class Fake_Vision(object):
    def __init__(self):
        rospy.init_node('fake_vision')
        rospy.Service('run_vision', RunVision, self.run_vision)

    def run_vision(self, srv):
        rospy.loginfo("Resending fake ground truth")
        return RunVisionResponse(srv.ground_truth_bins)
        

if __name__ == '__main__':
    fv = Fake_Vision()
    rospy.spin()
