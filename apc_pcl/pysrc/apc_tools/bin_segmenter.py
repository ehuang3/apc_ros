import numpy as np
import cv2
import os
import sys
from rospy import Duration
from .utils import path_to_root, path_to_file, xyzharray, xyzarray, xyzwarray
import tf
fpath = path_to_file(__file__)
root_path = path_to_root()

class Bin_Segmenter(object):
    def __init__(self, kinect_serial=None, camera_matrix=None):
        '''Provide a service that, given an image and a bin name, will return that bin segmented out of the image
        '''
        if camera_matrix is not None:
            self.cam_matrix = camera_matrix
        elif kinect_serial is not None:
            raise(Exception("Bin segmenter was not given a camera matrix"))
            self.cam_matrix = self.get_cam_matrix(kinect_serial)
        else:
            raise(Exception("No kinect specified"))

        # self.shelf = self.get_shelf_urdf()
        self.cam_matrix_inv = np.linalg.inv(self.cam_matrix)
        self.Transformer = tf.TransformerROS(True, Duration(10.0))        

    def get_cam_matrix(self, kinect_serial):
        calib_path = os.path.join(root_path, "apc_pcl", "calibration", kinect_serial, "calib_color.yaml")
        assert os.path.exists(calib_path), "No calibration known for {}, put it in apc_pcl/calibration".format(kinect_serial)
        cam_matrix = np.asarray(
            cv2.cv.Load(calib_path)
        )
        return cam_matrix

    def get_unit_vector(self, image_pt):
        ''' get_unit_vector(image_pt, camera_matrix)
        Given a pixel cordinate and a camera matrix, return a unit vector that defines the 
        world-space direction toward that pixel in the focal plane
        '''
        h_image_pt = np.hstack([image_pt[:2], 1.0])
        print h_image_pt
        spatial_vec = np.dot(self.cam_matrix_inv, h_image_pt)
        print spatial_vec
        unit_vec = spatial_vec / np.linalg.norm(spatial_vec)
        return unit_vec

    def optical_to_image(self, point, image_shape, rotation=np.array([0.0, 0.0, 0.0]), translation=np.array([0.0, 0.0, 0.0])):
        '''Convert a point in the camera optical frame to a pixel coordinate'''
        distortion = np.array([0.0, 0.0, 0.0, 0.0], np.float32)
        img_point, _ = cv2.projectPoints(np.array([np.float32(point[:3])]), rotation, translation, self.cam_matrix, distortion)
        image_frame_point = np.int32(img_point[0].flatten())
        if np.any(image_frame_point < (0, 0)) or np.any(image_frame_point > image_shape[:2][::-1]):
            # print "point out of bounds"
            return None
        return image_frame_point

    def get_bin_points(self, _bin, front=False):
        '''Given a bin, compute the bin points in the world frame
        _bin should be a bin message, front is a boolean deciding if you want all 8 points or just the front face
        '''
        shelf_world = _bin.pose_shelf_frame
        bin_shelf = _bin.pose_bin_shelf
        shelf_world_tf = self.Transformer.fromTranslationRotation(
            xyzarray(shelf_world.position), xyzwarray(shelf_world.orientation)
        )
        bin_shelf_tf = self.Transformer.fromTranslationRotation(
            xyzarray(bin_shelf.position), xyzwarray(bin_shelf.orientation)
        )
        bin_world_tf = np.dot(shelf_world_tf, bin_shelf_tf)

        bottom_height = np.array([0.0, 0.0, 0.05, 0.0])
        size = np.hstack([(0.5 * xyzarray(_bin.bin_size)), 1.0])

        # Are we returning just one face?
        if front:
            bin_points = np.array([
                [-0.0, 1.0, 1.0, 1.0] * size,
                [-0.0, -1.0, 1.0, 1.0] * size,    
                [-0.0, 1.0, -1.0, 1.0] * (size - bottom_height),
                [-0.0, -1.0, -1.0, 1.0] * (size - bottom_height),
            ])

        else:
            bin_points = np.array([
                [-2.0, 1.0, 1.0, 1.0] * size,
                [-0.0, 1.0, 1.0, 1.0] * size,
                [-0.0, -1.0, 1.0, 1.0] * size,
                [-2.0, -1.0, 1.0, 1.0] * size,

                [-2.0, 1.0, -1.0, 1.0] * (size - bottom_height),
                [-0.0, 1.0, -1.0, 1.0] * (size - bottom_height),
                [-0.0, -1.0, -1.0, 1.0] * (size - bottom_height),
                [-2.0, -1.0, -1.0, 1.0] * (size - bottom_height),
            ])

        # Comptue bin position in world frame
        for index, point in enumerate(bin_points):
            bin_points[index] = np.dot(bin_world_tf, point)

        return bin_points

    def segment_bin(self, image, _bin, transform):
        bin_world_points = self.get_bin_points(_bin, front=True)

        image_points = np.zeros((4, 2), np.int32)
        for n, world_point in enumerate(bin_world_points):
            camera_point = np.dot(transform, world_point)
            image_point = self.optical_to_image(camera_point, image.shape)

            # If we are out of bounds
            if image_point is None:
                return None

            image_points[n] = image_point
        x, y, w, h = cv2.boundingRect(np.array([image_points]))
        return image[y: y + h, x: x + w, :], (x, y, w, h)


    def draw_point(self, image, camera_point, color=(0, 0, 255)):
        camera_point = camera_point[:3]

        zero = rotation = translation = np.zeros(3, np.float32)
        image_point = self.optical_to_image(camera_point, image.shape)
        if image_point is None:
            return  # Don't draw!
        image_pt = (image_point[0], image_point[1])
        cv2.circle(image, image_pt, 5, color, -1)

    def draw_bin(self, image, _bin, transform):
        shelf_world = _bin.pose_shelf_frame
        bin_shelf = _bin.pose_bin_shelf

        shelf_world_tf = self.Transformer.fromTranslationRotation(
            xyzarray(shelf_world.position), xyzwarray(shelf_world.orientation)
        )

        bin_shelf_tf = self.Transformer.fromTranslationRotation(
            xyzarray(bin_shelf.position), xyzwarray(bin_shelf.orientation)
        )

        # Complete transform from bin to world (crichton origin)
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

        size = np.hstack([(0.5 * xyzarray(_bin.bin_size)) - np.array([0.0, 0.0, 0.05]), 1.0])

        for point in cube_points:
            world_point = np.dot(bin_world_tf, point * size)
            camera_point = np.dot(transform, world_point)
            self.draw_point(image, camera_point, color=(255, 0, 0))

    ''' ////////////// These functions are UNUSED in the final product'''
    def get_shelf_urdf(self):
        '''Load the urdf xml (SHOULD BE USING ROSPARAM)'''
        f = file(os.path.join(root_path, 'apc_description', 'urdf', 'kiva_pod', 'kiva_pod.urdf'))
        shelf = URDF.from_xml_string(f.read())
        return shelf        

    def get_bin_points_urdf(self, bin_name):
        '''get the points belonging to the bin in the kiva_pod_origin frame
        I am not doing any interesting math or frame resolution, I don't even handle angles
        '''
        assert bin_name in self.shelf.link_map.keys(), "Bin name {} unknown in URDF".format(bin_name)
        _bin = self.shelf.link_map[bin_name]
        bin_size = np.array(_bin.visual.geometry.size, np.float32)
        bin_joint = self.shelf.joint_map['{}_joint'.format(bin_name)]
        bin_position = np.array(bin_joint.origin.xyz, np.float32)
        return bin_position, bin_size
    ''' //////////////////////////////////////////////////////////////'''


if __name__ == '__main__':
    matrix = np.array([
        [1045.8557400637853, 0.0, 935.8000074500827],
        [0.0, 1048.1342812101645, 535.4019808653711],
        [0.0, 0.0, 1.0],
    ])

    bs = Bin_Segmenter(camera_matrix=matrix)
    # [1080.0, 1920.0]
    print bs.get_unit_vector([0.0, 0.0])
    # assert False, 'This is not to be run as main'