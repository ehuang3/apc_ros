#!/usr/bin/env python
#
# Copyright (c) 2015, Georgia Tech Research Corporation
# All rights reserved.
#
# Author(s): Eric Huang <ehuang@gatech.edu>
# Georgia Tech Humanoid Robotics Lab
# Under Direction of Prof. Andrea Thomaz <athomaz@cc.gatech.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#     * Redistributions of source code must retain the above
#       copyright notice, this list of conditions and the following
#       disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials
#       provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import rospkg
import rosbag
import argparse
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from apc_msgs.srv import *


def main():
    parser = argparse.ArgumentParser(description='Test RunDPM service')
    parser.add_argument('--image', help='path to image')
    parser.add_argument('--object_id', help='object ID')
    args = parser.parse_args()

    rospy.init_node('test_run_dpm_client')
    rospy.wait_for_service('run_dpm')
    run_dpm = rospy.ServiceProxy('run_dpm', RunDPM)

    image = args.image
    object_id = args.object_id

    bridge = CvBridge()
    cv_img = cv2.imread(image)
    img_msg = bridge.cv2_to_imgmsg(cv_img, "bgr8")

    request = RunDPMRequest()
    request.image = img_msg
    request.target_objects.append(object_id)

    response = run_dpm(request)

    print response

if __name__ == "__main__":
    main()
