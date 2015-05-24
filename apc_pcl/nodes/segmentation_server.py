#!/usr/bin/python
import rospy
import pymatlab
from apc_msgs.srv import *
from apc_msgs.msg import DPMObject
from geometry_msgs.msg import Pose
import threading
import os
import cv2
import numpy as np
from datetime import datetime

lock = threading.Lock()

def thread_lock(func):
    '''The matlab server is NOT thread-safe, this makes it safe to run the segmentation server'''
    def locked_function(*args, **kwargs):
        lock.acquire()
        result = func(*args, **kwargs)
        lock.release()
        return result
    return locked_function


class Matlabber(object):
    '''Simple wrapper for pymatlab's matlab session object
    Easily enables:
        - Force semicolon addition
        - Some super simple syntax checking to avoid stupid crashes
        - Logging of the actual /text/ command being sent to Matlab (Based on a settable verbosity)
    '''
    def __init__(self, logging=False):
        self.logging = logging
        self.matlab = pymatlab.session_factory()

    def log(self, func):
        def logged_func(_string):
            if self.logging:
                print '>> Matlabbing:', _string
            if _string.strip()[-1] != ';':
                _string += ';'
            assert _string.count('(') == _string.count(')'), "() mismatch in matlab call"
            assert _string.count('{') == _string.count('}'), "{} mismatch in matlab call"
            return func(_string)
        return logged_func

    def __getattr__(self, name):
        '''This allows us to use Matlabber as a drop-in replacement for pymatlab's tool
            If the function is a run call, log the text sent. It is not practical 
            to print the response, as it is generally a very large image.
        '''
        func = getattr(self.matlab, name)
        if name == 'run':
            return self.log(func)
        else:
            return func


class Object_Segment(object):
    # I made this list by going into the meshes folder and running 
    # ls -l | awk '{print $9}' | c 
    # Where c is an alias for xclip -> copy to clipboard
    
    valid_objects = [
        "champion_copper_plus_spark_plug",
        "cheezit_big_original",
        "crayola_64_ct",
        "dove_beauty_bar",
        "dr_browns_bottle_brush",
        "elmers_washable_no_run_school_glue",
        "expo_dry_erase_board_eraser",
        "feline_greenies_dental_treats",
        "first_years_take_and_toss_straw_cups",
        "genuine_joe_plastic_stir_sticks",
        "highland_6539_self_stick_notes",
        "kong_air_dog_squeakair_tennis_ball",
        "kong_duck_dog_toy",
        "kong_sitting_frog_dog_toy",
        "kygen_squeakin_eggs_plush_puppies",
        "laugh_out_loud_joke_book",
        "mark_twain_huckleberry_finn",
        "mead_index_cards",
        "mommys_helper_outlet_plugs",
        "munchkin_white_hot_duck_bath_toy",
        "one_with_nature_soap_dead_sea_mud",
        "oreo_mega_stuf",
        "paper_mate_12_count_mirado_black_warrior",
        "rollodex_mesh_collection_jumbo_pencil_cup",
        "safety_works_safety_glasses",
        "sharpie_accent_tank_style_highlighters",
        "stanley_66_052",
    ]

    def __init__(self):
        '''
        TODO:
            - Segmentation server
            - Matlab test
            - Easy way to record new data for the histo-matcher

        How to:
            a = np.random.randn(20, 10, 30)
            session.putvalue('a', a)  # Create a variable in the matlab workspace called a
            session.run('f = sqrt('a.^2);')
            f = session.getvalue('f')
        '''

        # Check if we are in training mode
        self.training = rospy.get_param('train_segmentation')
        self.visualize = rospy.get_param('visualize', False)
        self.matlab = Matlabber(logging=True)
        # self.matlab = pymatlab.session_factory()  # This is blocking until we have initialized the session
        from apc_tools import path_to_root
        # We have to do this AFTER we create the matlab instance.

        self.apc_matlab = os.path.join(path_to_root(), 'apc_pcl', 'matsrc')
        self.matlab.run("addpath(genpath('{}'));".format(self.apc_matlab))

        # places 'sets', which contains the training data, in the workspace
        if self.training:
            # Create an empty sets
            self.matlab.run('sets = {}')
        else:
            training_data = os.path.join(self.apc_matlab, "train_data_2.mat")
            self.matlab.run("load('{}');".format(training_data))

        rospy.init_node('segmentation_server')
        rospy.Service('/segment_image', SegmentImage, self.segment_service)

    def list_to_cellarray(self, _list):
        '''Convert a python list to a Matlab-style cell-array in string form
        This enables us to get around 
        '''
        return str(_list).replace('[', '{').replace(']', '}')

    def train_on_image(self, image, object_name, object_list):
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.matlab.putvalue("image", rgb_image)
        self.matlab.putvalue("object_name", object_name)
        self.matlab.run("object_list = {};".format(self.list_to_cellarray(object_list)))
        # self.matlab.run("sets = apc_manual_train_histo(image, object_name, 10, sets)")
        self.matlab.run("sets = apc_record_image(image, object_name, sets)")
        print "Done with training segmentation run on {}".format(object_name)

    def segment_image(self, image, object_name, object_list):
        ''' Apply the histo-seg algorithm to the image using the matlab server

        Notes:
            Histo-seg will return a failure notification if it cannot confidently segment the object
            Objectively, this means that the highest quality match found was below our manually set threshhold

        '''

        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.matlab.putvalue("image", rgb_image)
        self.matlab.putvalue("object_name", object_name)
        self.matlab.run("object_list = {};".format(self.list_to_cellarray(object_list)))
        self.matlab.run("use_sets = apc_pre_train(sets, [object_list, 'background']);")  # Train with background

        if self.visualize:
            visualize = '1'
        else:
            visualize = '0';

        try:
            # I acknowledge that this is a horrifying bastardization of PEP8
            self.matlab.run(
                "eval('" + 
                    "[bounding_boxes, success] = apc_bounding_box(image, object_name, object_list, use_sets, {});" + 
                "');".format(
                visualize
                )
            )

        except RuntimeError as e:
            print e
            return None

        success = self.matlab.getvalue("success")

        if (not success):
            rospy.logerr("Histo-Seg reported acceptability threshhold failure for {}".format(object_name))
            return None

        bounding_boxes = self.matlab.getvalue("bounding_boxes")

        if len(bounding_boxes.shape) == 1:
            bounding_boxes = np.array([bounding_boxes])  # If we get back a one-dimensional array
        self.matlab.run("clear image bounding_boxes object_name object_list use_sets success")
        # x, y, w, h = bounding_box  # Split it all out

        return bounding_boxes  # [(x, y, w, h)...]

    @thread_lock
    def segment_service(self, srv):
        # Check if we have more than 15 windows open, if so, close them all
        self.matlab.run("if length(findall(0, 'type', 'figure')) > 15\nclose all\n end")
        from apc_tools import get_image_msg  # I know this is weird. I promise it was necessary.
        image = get_image_msg(srv.image)
        object_name = srv.object_id

        # Ensure that all of our items are valid
        assert object_name in self.valid_objects, "Object {} is unknown".format(object_name)
        object_list = srv.bin_objects
        for name in object_list:
            assert name in self.valid_objects, "Object {} is unknown".format(name)

        # Check if we're in training mode
        if self.training:
            self.train_on_image(image, object_name, object_list)
            return SegmentImageResponse()

        bounding_boxes = self.segment_image(image, object_name, object_list)
        if bounding_boxes is None:
            return SegmentImageResponse(success=False)

        dpm_objects = []
        for bounding_box in bounding_boxes:
            print bounding_box
            x, y, w, h = bounding_box 
            dpm_objects.append(
                DPMObject(
                    x=x, y=y, height=h, width=w,
                    pose=Pose(),
                    object_id=object_name,
                )
            )

        return SegmentImageResponse(
            found_objects=dpm_objects,
            success=True
        )

    @thread_lock
    def cleanup(self):
        rospy.logwarn("Killing segmentation server")
        def timestamp():
            now = datetime.now()
            return "{}_{}_{}_{}".format(now.month, now.day, now.hour, now.minute)
        if self.training:
            savepath = os.path.join(self.apc_matlab, 'images', 'rec_training_data_{}.mat'.format(timestamp()))
            self.matlab.run("if (length(sets) > 0)\nsave('{}', 'sets');\nend".format(savepath))

        self.matlab.run('close all')


if __name__ == '__main__':
    objseg = Object_Segment()
    rospy.on_shutdown(objseg.cleanup)
    rospy.spin()