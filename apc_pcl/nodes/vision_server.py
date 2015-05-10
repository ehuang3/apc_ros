#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from apc_msgs.srv import *
from apc_msgs.msg import BinState
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from apc_tools import Image_Subscriber, Bin_Segmenter, xyzharray, xyzarray, xyzwarray, make_image_msg
import tf
'''Show the current image, allow the user to draw a bounding box
'''

class Vision_Server(object):
    def __init__(self):
        rospy.init_node('vision_server')
        self.im_sub = Image_Subscriber('/camera/depth/cloud_image', self.got_image)        
        self.image, self.show_image = None, None
        self.segmented = None

        self.cloud_sub = rospy.Subscriber('/kinect2_cool/depth_lowres/points/', PointCloud2, self.got_cloud)

        self.Listener = tf.TransformListener()
        self.Transformer = tf.TransformerROS(True, rospy.Duration(10.0))        
        rospy.sleep(4.0)
        rospy.loginfo('----ready---')

        self.camera_name_mapping = {
            'kinect_lower': '503233542542',
            'primesense': 'primesense',
        }

        rospy.Service('run_vision', RunVision, self.run_vision)
        # cv2.namedWindow("display")
        cv2.namedWindow("sub_segment")

        self.point_pub = rospy.Publisher('test_points', PointStamped, queue_size=20)
        self.bin_pub = rospy.Publisher('bin_points', PointStamped, queue_size=40)

        dpm_server = 'run_dpm_simulated'
        # dpm_server = '/run_dpm_4'
        rospy.logwarn("Looking for DPM server {}".format(dpm_server))

        self.dpm_proxy = rospy.ServiceProxy(dpm_server, RunDPM)
        self.frustum_proxy = rospy.ServiceProxy('cull_frustum', GetCloudFrustum)
        self.background_cull_proxy = rospy.ServiceProxy('cull_background', CullCloudBackground)

        self.view_loop()

    def got_image(self, msg):
        '''Temporary, while we are getting images ourselves'''
        self.image = msg

    def got_cloud(self, msg):
        self.cloud = msg

    def view_loop(self):
        while(not rospy.is_shutdown()):
            if self.show_image is None:
                continue
            # cv2.imshow("display", self.show_image)

            if self.segmented is not None:
                cv2.imshow("sub_segment", self.segmented)

            key = cv2.waitKey(10) & 0xff

            if key == ord('q'):
                break

    def publish_pt(self, point, frame='crichton_origin'):
        point = point[:3]
        self.point_pub.publish(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=frame,
            ),
            point=Point(*point),
        )

    def publish_bin_pt(self, point, frame='crichton_origin'):
        point = point[:3]
        self.bin_pub.publish(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=frame,
            ),
            point=Point(*point),
        )

    def publish_bin(self, _bin, transform=None):
        shelf_world = _bin.pose_shelf_frame
        bin_shelf = _bin.pose_bin_shelf
        shelf_world_tf = self.Transformer.fromTranslationRotation(
            xyzarray(shelf_world.position), xyzwarray(shelf_world.orientation)
        )
        bin_shelf_tf = self.Transformer.fromTranslationRotation(
            xyzarray(bin_shelf.position), xyzwarray(bin_shelf.orientation)
        )
        bin_world_tf = np.dot(shelf_world_tf, bin_shelf_tf)

        cube_points = np.array([
            [-2.0, 1.0, 1.0, 1.0],
            [-0.0, 1.0, 1.0, 1.0],
            [-0.0, -1.0, 1.0, 1.0],
            [-2.0, -1.0, 1.0, 1.0],

            [-2.0, 1.0, -1.0, 1.0],
            [-0.0, 1.0, -1.0, 1.0],
            [-0.0, -1.0, -1.0, 1.0],
            [-2.0, -1.0, -1.0, 1.0],
        ])

        size = np.hstack([0.5 * xyzarray(_bin.bin_size), 1.0])

        for point in cube_points:
            self.publish_bin_pt(np.dot(bin_world_tf, point * size), frame='crichton_origin')

    def bin_to_world(self, bin_shelf, shelf_world):
        shelf_world_tf = self.Transformer.fromTranslationRotation(
            xyzarray(shelf_world.position), xyzwarray(shelf_world.orientation)
        )
        bin_shelf_vector = xyzharray(bin_shelf.position)
        return np.dot(shelf_world_tf, bin_shelf_vector)

    def run_vision(self, srv):
        assert self.image is not None, "Have not yet cached an image"
        self.show_image = np.copy(self.image)
        Bin_Seg = Bin_Segmenter(self.camera_name_mapping[srv.camera_id])

        target_frame = "kinect2_cool_rgb_optical_frame";
        # target_frame = "kinect2_cool_ir_optical_frame";
        source_frame = "crichton_origin";
        m = self.Listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        transform = self.Transformer.fromTranslationRotation(*m)

        rospy.loginfo('Running vision')
        info = ""
        info += '\nGot camera ID {}'.format(srv.camera_id)

        for number, _bin in enumerate(srv.bins):
            bin_size = xyzarray(_bin.bin_size)
            shelf_position = xyzharray(_bin.pose_shelf_frame.position)

            color_map = {
                'bin_G': (0, 255, 0),
                'bin_J': (255, 0, 0),
                'bin_K': (255, 255, 0),
                'bin_H': (255, 255, 0),

            }
            if _bin.bin_name in color_map.keys():
                self.publish_bin(_bin)
                Bin_Seg.draw_bin(self.show_image, _bin, transform)

                if _bin.bin_name == 'bin_G':
                    segmented, (x, y, w, h) = Bin_Seg.segment_bin(self.image, _bin, transform)
                    print segmented.shape
                    assert segmented.shape != (0, 0, 1), "Bin region out of bounds"
                    resp = self.dpm_proxy(
                        make_image_msg(segmented), 
                        ['oreo_mega_stuf'],
                        ['oreo_mega_stuf'],
                    )

                    for obj in resp.detected_objects:
                        print 'Detected {}, box x: {}, y: {}, width: {}, height: {}'.format(
                            obj.object_id, obj.x, 
                            obj.y, obj.height, obj.width
                        )

                        cv2.rectangle(segmented, (obj.x, obj.y), (obj.x + obj.width, obj.y + obj.height), (0, 255, 0), 2)
                    self.segmented = segmented
                    print 'Sending image of size {}'.format(self.image.shape)
                    frustum_cloud = self.frustum_proxy(
                        self.cloud,
                        make_image_msg(self.image),
                        obj.x + x, 
                        obj.y + y, 
                        obj.height, 
                        obj.width
                    )
                    print 'Got frustum back, culling background'
                    self.background_cull_proxy(frustum_cloud.sub_cloud)
                    print 'culled background'

            else:
                print _bin.bin_name
                continue

            # Printing stuff
            info += '\n{}: Name: {}'.format(number, _bin.bin_name)
            position = _bin.pose_bin_shelf.position
            size = _bin.bin_size
            info += '\nPosition: ({}, {}, {})\n Size: ({}, {}, {})'.format(
                position.x,
                position.y,
                position.z,
                size.x,
                size.y,
                size.z,
            )
            info += '\nContains:'
            for _object in _bin.object_list:
                info += '\n\tID: {} Key: {}'.format(_object.object_id, _object.object_key)
            else:
                info += '\n\tNothing'

        rospy.loginfo(info)

        # Then call DPM (Or simulated DPM)
        return RunVisionResponse([BinState()] * 12)


if __name__ == '__main__':
    sdpm = Vision_Server()
    rospy.spin()