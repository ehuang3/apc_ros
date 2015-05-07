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
from apc_scripts import TopicReader
from sensor_msgs.msg import Image
from apc_msgs.srv import GetImage

topic_reader = []

def handle_image_grabber_service(req):
    global topic_reader
    resp = apc_msgs.msg.GetImageResponse()
    resp.image = topic_reader.get_next_message()
    return resp

def main():
    rospy.init_node('image_grabber_node')
    service_topic = rospy.get_param('~service_topic', '/kinect2/image_grabber')
    image_topic = rospy.get_param('~image_topic', '/kinect2/rgb/image')
    global topic_reader
    topic_reader = apc_scripts.TopicReader(image_topic, sensor_msgs.msg.Image)
    rospy.spin()

if __name__ == "__main__":
    main()
