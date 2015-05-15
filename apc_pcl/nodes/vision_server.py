#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped
from apc_msgs.srv import *
from apc_msgs.msg import BinState, ObjectState
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rospy.numpy_msg import numpy_msg
from apc_tools import (Image_Subscriber, Bin_Segmenter, xyzharray, xyzarray, xyzwarray, pqfrompose, pose_from_matrix,
    make_image_msg, load_background, path_to_root)
import tf
import os
import rosbag
'''Show the current image, allow the user to draw a bounding box
'''

class Vision_Server(object):
    def __init__(self):
        rospy.init_node('vision_server')

        self.simulate_segmentation = rospy.get_param('simulate_segmentation')
        ## Image and cloud
        self.im_sub = Image_Subscriber('/camera/depth/cloud_image', self.got_image)        
        self.camera_info_sub = rospy.Subscriber('/kinect2/rgb_rect/camera_info', numpy_msg(CameraInfo), self.got_camera_info)
        self.image, self.show_image = None, None
        self.segmented = None

        self.cloud_sub = rospy.Subscriber('/kinect2/depth_highres/points/', PointCloud2, self.got_cloud)

        ## TF
        self.Listener = tf.TransformListener()
        self.Transformer = tf.TransformerROS(True, rospy.Duration(10.0))        
        rospy.loginfo('----ready----')

        # Service inits
        rospy.Service('run_vision', RunVision, self.run_vision)

        self.point_pub = rospy.Publisher('test_points', PointStamped, queue_size=20)
        self.bin_pub = rospy.Publisher('bin_points', PointStamped, queue_size=40)

        if self.simulate_segmentation:
            segmentation_server = 'run_dpm_simulated'
        else:
            segmentation_server = 'segment_image'    

        print "Waiting for segmentation server"
        rospy.wait_for_service(segmentation_server)

        rospy.logwarn("Looking for DPM server {}".format(segmentation_server))
        self.segmentation_proxy = rospy.ServiceProxy(segmentation_server, SegmentImage)
        self.frustum_proxy = rospy.ServiceProxy('cull_frustum', GetCloudFrustum)
        self.background_cull_proxy = rospy.ServiceProxy('cull_background', CullCloudBackground)
        self.backgr_cloud, _, self.backgr_pose = load_background()

        self.target_cloud_proxy = rospy.ServiceProxy('/get_mesh', GetMesh)
        # self.registration_proxy = rospy.ServiceProxy('/shot_detector', shot_detector_srv)
        self.registration_proxy = rospy.ServiceProxy('/apc_object_detection/Shot_detector', shot_detector_srv)

        # Image viewing loop
        self.view_loop()

    def got_image(self, msg):
        '''Temporary, while we are getting images ourselves'''
        self.image = msg

    def got_cloud(self, msg):
        self.cloud = msg

    def got_camera_info(self, msg):
        self.camera_matrix = msg.K.reshape(3, 3)
        self.camera_info_sub.unregister()

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
        print 'publishing a point'
        point = point[:3]
        self.point_pub.publish(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=frame,
            ),
            point=Point(*point),
        )

    def publish_bin_pt(self, point, frame='kinect2_rgb_optical_frame'):
        point = point[:3]
        self.bin_pub.publish(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=frame,
            ),
            point=Point(*point),
        )

    def publish_bin(self, _bin, transform=None, transform_frame='crichton_origin'):
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
            bin_world = np.dot(bin_world_tf, point * size)
            bin_transformed = np.dot(transform, bin_world)
            self.publish_bin_pt(bin_transformed, frame=transform_frame)

    def run_vision(self, srv):
        assert self.image is not None, "Have not yet cached an image"
        self.show_image = np.copy(self.image)
        Bin_Seg = Bin_Segmenter(camera_matrix=self.camera_matrix)

        target_frame = "kinect2_rgb_optical_frame";
        # target_frame = "kinect2_cool_ir_optical_frame";
        source_frame = "crichton_origin";
        now = rospy.Time.now()
        rospy.sleep(0.1)
        # self.Listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(4.0))
        # print 'waited for tf'
        m = self.Listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        transform = self.Transformer.fromTranslationRotation(*m)

        rospy.loginfo('Running vision')
        info = ""
        info += '\nGot camera ID {}'.format(srv.camera_id)

        bin_states = []
        for number, _bin in enumerate(srv.bins):
            bin_state = BinState(
                bin_name=_bin.bin_name
            )
            object_states = []

            bin_size = xyzarray(_bin.bin_size)
            shelf_position = xyzharray(_bin.pose_shelf_frame.position)

            color_map = {
                # 'bin_G': (0, 255, 0),
                'bin_J': (255, 0, 0),
                # 'bin_K': (255, 255, 0),
                # 'bin_H': (255, 255, 0),
            }
            if _bin.bin_name in color_map.keys():
                self.publish_bin(_bin, transform=transform, transform_frame=target_frame)
                Bin_Seg.draw_bin(self.show_image, _bin, transform)
                object_names = ['crayola_64_ct']

                segmented, (x, y, w, h) = Bin_Seg.segment_bin(self.image, _bin, transform)
                assert segmented.shape != (0, 0, 1), "Bin region out of bounds"

                for object_name in object_names:
                    obj = self.segmentation_proxy(
                        make_image_msg(segmented), 
                        object_name,
                        [object_name],
                    )

                    print 'Detected {}, box x: {}, y: {}, width: {}, height: {}'.format(
                        object_name, obj.x,
                        obj.y, obj.height, obj.width
                    )

                    cv2.rectangle(segmented, (obj.x, obj.y), (obj.x + obj.width, obj.y + obj.height), (0, 255, 0), 2)
                    self.segmented = segmented

                    print "Culling background"
                    background_culled = self.background_cull_proxy(self.cloud, self.backgr_cloud, self.backgr_pose, Pose())
                    print "Culled background"

                    print 'Sending image of size {} for frustum culling'.format(self.image.shape)
                    object_alone = self.frustum_proxy(
                        background_culled.cloud,
                        make_image_msg(self.image),
                        obj.x + x,
                        obj.y + y,
                        obj.height,
                        obj.width,
                        True,
                    )
                    print 'Frustum culled'
                    # object_alone = self.background_cull_proxy(self.cloud, self.backgr_cloud, self.backgr_pose, Pose())

                    print 'Getting target cloud'
                    target_cloud = self.target_cloud_proxy(object_name)
                    print 'Got target cloud'

                    print 'Attempting to register'
                    registration = self.registration_proxy(
                        object_alone.sub_cloud, 
                        target_cloud.cloud,
                        object_name
                    )

                    print "Registered object"
                    object_pose = registration.pose
                    self.publish_pt(xyzarray(object_pose.position), frame='kinect2_rgb_optical_frame')
                    print 'Object pose kinect2_rgb_optical_frame', object_pose

                    pq = pqfrompose(object_pose)  # Position, quaternion
                    object_pose_kinect = self.Transformer.fromTranslationRotation(*pq)
                    kinect_world_tf = np.linalg.inv(transform)
                    object_pose_world = np.dot(kinect_world_tf, object_pose_kinect)  # Matrix
                    object_pose_world_msg = pose_from_matrix(object_pose_world)
                    print object_pose_world_msg

                    object_state = ObjectState(
                        object_id=object_name,
                        object_key='',
                        object_pose=object_pose_world_msg,
                    )
                    object_states.append(object_state)

            bin_state.object_list = object_states
            bin_states.append(bin_state)

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
        print 'Sending bin states of length {}'.format(len(bin_states))
        return RunVisionResponse(
            bin_contents=bin_states
        )


if __name__ == '__main__':
    vs = Vision_Server()
    rospy.spin()