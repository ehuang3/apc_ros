#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from apc_msgs.srv import *
from apc_msgs.msg import BinState
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rospy.numpy_msg import numpy_msg
from apc_tools import path_to_root
import tf
import rosbag
import os


class Background_Recorder(object):
    def __init__(self):
        '''Wait until we have both an image and a point cloud, then save it
        (Along with a proper transformation)
        '''
        rospy.init_node('background_recorder')
        self.store_path = os.path.join(path_to_root(), 'apc_pcl', 'calibration')
        self.pcl_sub = rospy.Subscriber('/kinect2/depth_highres/points', PointCloud2, self.got_pcl)
        self.image_sub = rospy.Subscriber('/camera/depth/cloud_image', Image, self.got_image)
        self.Listener = tf.TransformListener()
        self.Transformer = tf.TransformerROS(True, rospy.Duration(10.0))

        self.shelf_srv = rospy.Service('get_shelf', GetShelf, self.got_shelf)

        self.pcl = None
        self.image = None
        self.shelf_pose = None
        self.run_loop()

    def run_loop(self):
        r = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            if ((self.pcl is not None) and (self.image is not None) and (self.shelf_pose is not None)):
                self.pcl_sub.unregister()
                self.image_sub.unregister()
                path = os.path.join(self.store_path, 'background.bag')
                print 'Recording background to {}'.format(path)
                bag = rosbag.Bag(path, 'w')
                bag.write('background_cloud', self.pcl)
                bag.write('background_image', self.image)
                bag.write('background_pose', self.shelf_pose)
                bag.close()
                return
            else:
                r.sleep()

    def get_transform(self):
        target_frame = "kinect2_rgb_optical_frame";
        source_frame = "crichton_origin";
        m = self.Listener.lookupTransform(target_frame, source_frame, rospy.Time.now())
        transform = self.Transformer.fromTranslationRotation(*m)
        return transform

    def got_shelf(self, srv):
        print 'Got shelf pose'
        self.shelf_pose = srv.pose_shelf_world
        return GetShelfResponse()

    def got_pcl(self, msg):
        print 'Got cloud'
        self.pcl = msg

    def got_image(self, msg):
        print 'Got image'
        self.image = msg

if __name__ == '__main__':
    br = Background_Recorder()
