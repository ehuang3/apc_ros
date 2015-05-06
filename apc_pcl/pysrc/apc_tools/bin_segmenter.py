import numpy as np
import cv2
import os
import sys
import rospy
from .utils import path_to_root, path_to_file
from urdf_parser_py.urdf import URDF
from tf import transformations
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

    def get_bin_points(self, bin_name):
        '''get the points belonging to the bin in the kiva_pod_origin frame
        I am not doing any interesting math or frame resolution, I don't even handle angles
        '''
        assert bin_name in self.shelf.link_map.keys(), "Bin name {} unknown in URDF".format(bin_name)
        _bin = self.shelf.link_map[bin_name]
        bin_size = np.array(_bin.visual.geometry.size, np.float32)
        bin_joint = self.shelf.joint_map['{}_joint'.format(bin_name)]
        bin_position = np.array(bin_joint.origin.xyz, np.float32)
        return bin_position, bin_size

    def world_to_imframe(self, point):
        '''apply rotation matrix'''
        x, y, z = 0, 1, 2
        imframe_pt = np.zeros(3)
        imframe_pt[x] = point[y]
        imframe_pt[y] = -point[z]
        imframe_pt[z] = -point[x]
        return imframe_pt

    def optical_to_image(self, point, rotation, translation):
        '''Convert a point in the camera optical frame to a pixel coordinate'''
        theta = np.int32(0.0)
        distortion = np.array([0.0, 0.0, 0.0, 0.0], np.float32)

        img_point, _ = cv2.projectPoints(np.array([np.float32(point)]), rotation, translation, self.cam_matrix, distortion)
        return np.int32(img_point[0].flatten())

    def draw_point(self, image, world_point, rotation, translation, color=(0, 0, 255)):
        image_point = self.optical_to_image(world_point, rotation, translation)
        image_pt = (image_point[0], image_point[1])
        cv2.circle(image, image_pt, 5, color, -1)

    def draw_bin(self, image, bin_name, rotation, translation, offset=np.array([0.0, 0.0, 0.0])):
        bin_center, size = bseg.get_bin_points(bin_name)
        imframe_bin_center = bseg.world_to_imframe(bin_center)
        world_center = imframe_bin_center + offset
        bseg.draw_point(image, world_center, rotation, translation)

        square_points = np.array([
            [0.0, 1.0, -1.0],
            [0.0, 1.0, 1.0],
            [0.0, -1.0, 1.0],
            [0.0, -1.0, -1.0],
        ])
        for bin_pt in square_points:
            square_point = (bin_pt * 0.5 * size) + bin_center
            imframe_bin_pt = bseg.world_to_imframe(square_point)
            world_bin_pt = imframe_bin_pt + offset
            bseg.draw_point(image, world_bin_pt, rotation, translation, color=(255, 0, 0))


def visualize():
    '''THIS IS JUST FOR TESTING'''
    from tf.transformations import euler_matrix
    def nothing(x):
        pass

    cv2.namedWindow("trackbar")
    slider_vals = [166, 157, 6, 39, 11]
    cv2.createTrackbar('theta', 'trackbar', slider_vals[0], 314, nothing)
    cv2.createTrackbar('phi', 'trackbar', slider_vals[1], 314, nothing)
    cv2.createTrackbar('point_x', 'trackbar', slider_vals[2], 100, nothing)
    cv2.createTrackbar('point_y', 'trackbar', slider_vals[3], 100, nothing)
    cv2.createTrackbar('point_z', 'trackbar', slider_vals[4], 100, nothing)

    test_image = cv2.imread(os.path.join(fpath, 'test_image.jpg'))

    bseg = Bin_Segmenter(kinect_serial='503233542542')

    # 24.5 inches forward
    # 6 inches to the side
    fwd = 0.0254 * 24.5
    # side = 0.0254 * 2
    side = 0.0

    bins = [
        'bin_A', 'bin_B', 'bin_C', 
        'bin_D', 'bin_E', 'bin_F', 
        'bin_G', 'bin_H', 'bin_I',
        'bin_J', 'bin_K', 'bin_L'
    ]

    while(1):
        image = np.copy(test_image)
        theta = (cv2.getTrackbarPos('theta', 'trackbar') / 100.0) - (np.pi/2)
        phi = (cv2.getTrackbarPos('phi', 'trackbar') / 100.0) - (np.pi/2)
        side = -((cv2.getTrackbarPos('point_x', 'trackbar')) / 50.0) # This is tenths of an inch
        up = ((50 - cv2.getTrackbarPos('point_y', 'trackbar')) / 10.0) # This is fifths of an inch
        fwd = ((cv2.getTrackbarPos('point_z', 'trackbar')) / 10.0)


        slider_vals = [
            cv2.getTrackbarPos('theta', 'trackbar'),
            cv2.getTrackbarPos('phi', 'trackbar'),
            cv2.getTrackbarPos('point_x', 'trackbar'),
            cv2.getTrackbarPos('point_y', 'trackbar'),
            cv2.getTrackbarPos('point_z', 'trackbar'),
        ]

        # theta = -0.0907963267949
        # phi = -0.000796326794896
        # fwd = 1.3
        # side = 0.0
        # up = 1.3

        print 'theta {}, phi {}, fwd {} (m) side {}, up {} (rads, inches)'.format(theta, phi, fwd, side, up)
        offset = np.array([side, up, fwd])

        rotation_mat = euler_matrix(0.0, theta, phi)
        rotation, _ = cv2.Rodrigues(rotation_mat[:3, :3])

        translation = np.array([0.0, 0.0, 0.0], np.float32)

        for _bin in bins:
            bin_pt, size = bseg.get_bin_points(_bin)
            imframe_bin_pt = bseg.world_to_imframe(bin_pt)
            # print imframe_bin_pt + offset
            # bseg.draw_point(image, imframe_bin_pt + offset, rotation, translation)
            bseg.draw_bin(image, _bin, rotation, translation, offset)


        # bseg.draw_point(image, test_point, rotation, translation)
        cv2.imshow("image", image)

        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            print slider_vals
            break


if __name__ == '__main__':
    # visualize()
    bseg = Bin_Segmenter(kinect_serial='503233542542')
    visualize()