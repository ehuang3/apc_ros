#!/usr/bin/python
import rospy
import pymatlab
from apc_msgs.srv import *
import threading
import os
import cv2

lock = threading.Lock()

def thread_lock(func):
    def locked_function(*args, **kwargs):
        lock.acquire()
        result = func(*args, **kwargs)
        lock.release()
        return result
    return locked_function

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

        self.matlab = pymatlab.session_factory()  # This is blocking until we have initialized the session
        from apc_tools import path_to_root
        # We have to do this AFTER we create the matlab instance.

        print "Adding to path"
        apc_matlab = os.path.join(path_to_root(), 'apc_pcl', 'matsrc')
        self.matlab.run("addpath('{}');".format(apc_matlab))

        # places 'sets', which contains the training data, in the workspace
        print 'loading sets'
        training_data = os.path.join(apc_matlab, "train_data.mat")
        self.matlab.run("load('{}');".format(training_data))

        print 'loaded'
        rospy.init_node('segmentation_server')
        rospy.Service('/segment_image', SegmentImage, self.segment_service)

    def list_to_cellarray(self, _list):
        return str(_list).replace('[', '{').replace(']', '}')

    def segment_image(self, image, object_name, object_list):
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        print 'putting image'
        self.matlab.putvalue("image", rgb_image)
        print 'putting object name'
        self.matlab.putvalue("object_name", object_name)

        print 'putting object list', self.list_to_cellarray(object_list)
        print "object_list = {};".format(self.list_to_cellarray(object_list))
        self.matlab.run("object_list = {};".format(self.list_to_cellarray(object_list)))

        print 'running command'
        self.matlab.run("bb = apc_bounding_box(image, object_name, object_list, sets);")
        print 'gettinb bounding box'
        bounding_box = self.matlab.getvalue("bb")
        print 'cleaning up'
        self.matlab.run("clear image bb object_name object_list")
        # x, y, w, h = bounding_box  # Split it all out
        return bounding_box  # x, y, w, h

    @thread_lock
    def segment_service(self, srv):
        from apc_tools import get_image_msg  # I know this is weird. I promise it was necessary.
        image = get_image_msg(srv.image)
        object_name = srv.object_id
        object_list = srv.bin_objects
        x, y, w, h = self.segment_image(image, object_name, object_list)
        assert object_name in self.valid_objects, "Object {} is unknown".format(object_name)
        return SegmentImageResponse(
            x=x, y=y, height=h, width=w,
            success=True
        )


if __name__ == '__main__':
    objseg = Object_Segment()
    rospy.spin()