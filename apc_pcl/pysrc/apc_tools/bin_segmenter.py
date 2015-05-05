import numpy as np
import cv2
import os
import sys
import rospy
from .utils import path_to_root, path_to_file
from urdf_parser_py.urdf import URDF

fpath = path_to_file(__file__)
root_path = path_to_root()

class Bin_Segmenter(object):
    def __init__(self, kinect_serial):
        '''Provide a service that, given an image and a bin name, will return that bin segmented out of the image
        '''
        self.cam_matrix = self.get_cam_matrix(kinect_serial)
        self.shelf = self.get_shelf_urdf()

    def get_cam_matrix(self, kinect_serial):
        calib_path = os.path.join(root_path, "apc_pcl", "calibration", kinect_serial, "calib_color.yaml")
        print calib_path
        assert os.path.exists(calib_path), "No calibration known for {}, put it in apc_pcl/calibration".format(kinect_serial)
        cam_matrix = np.asarray(
            cv2.cv.Load(calib_path)
        )
        return cam_matrix

    def get_shelf_urdf(self):
        '''Load the urdf xml (SHOULD BE USING ROSPARAM)'''
        f = file(os.path.join(root_path, 'apc_description', 'urdf', 'kiva_pod', 'kiva_pod.urdf'))
        shelf = URDF.from_xml_string(f.read())
        return shelf

    def optical_to_image(self, point, rotation, translation):
        '''Convert a point in the camera optical frame to a pixel coordinate'''
        theta = np.int32(0.0)
        distortion = np.array([0.0, 0.0, 0.0, 0.0], np.float32)

        # 0 distortion coefficients may cause problems
        img_point, _ = cv2.projectPoints(np.array([np.float32(point)]), rotation, translation, self.cam_matrix, distortion)
        return np.int32(img_point[0].flatten())

    def draw_point(self, image, world_point, rotation, translation):
        image_point = self.optical_to_image(world_point, rotation, translation)
        image_pt = (image_point[0], image_point[1])
        cv2.circle(image, image_pt, 5, (0, 0, 255), -1)

    def get_affine_tf(self, triangle):
        # Get the points of the center bin
        np.array([
            [0.0, 0.0, 0.0],
            []
        ], np.int32)
        cv2.getAffine(triangle, )
        return NotImplemented


if __name__ == '__main__':
    from tf.transformations import matrix_from_euler
    def nothing(x):
        pass

    cv2.namedWindow("trackbar")
    cv2.createTrackbar('theta', 'trackbar', 314//2, 314, nothing)
    cv2.createTrackbar('phi', 'trackbar', 314//2, 314, nothing)
    cv2.createTrackbar('point_x', 'trackbar', 0, 100, nothing)
    # cv2.createTrackbar('B', 'image', 0, 255, nothing)
    test_image = cv2.imread(os.path.join(fpath, 'test_image.jpg'))

    bseg = Bin_Segmenter(kinect_serial='196605135147')

    # 24.5 inches forward
    # 6 inches to the side
    fwd = 0.0254 * 24.5
    # side = 0.0254 * 2
    side = 0.0

    while(1):
        image = np.copy(test_image)
        theta = (cv2.getTrackbarPos('theta', 'trackbar') / 100.0) - (np.pi/2)
        phi = (cv2.getTrackbarPos('theta', 'trackbar') / 100.0) - (np.pi/2)

        side = (cv2.getTrackbarPos('point_x', 'trackbar') / 50.0)
        test_point = np.array([side, 0.0, fwd])


        rotation = np.array([0.0, 1.0, 0.0], np.float32) * theta
        translation = np.array([0.0, 0.0, 0.0], np.float32)

        bseg.draw_point(image, test_point, rotation, translation)
        cv2.imshow("image", image)



        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

