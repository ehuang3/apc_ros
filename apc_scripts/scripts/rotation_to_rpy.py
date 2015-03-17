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
import numpy
import tf.transformations


def main():
    T = numpy.array([ [0.0373284,   0.290763,   0.956067,   0.109844 ],
                      [0.998929,   -0.0370362, -0.0277383,  0.0232314],
                      [0.0273439,   0.956078,  -0.291834,   0.152649 ],
                      [0,           0,          0,          1] ])

    R = numpy.array([ [0.0373284,   0.290763,   0.956067 ],
                      [0.998929,   -0.0370362, -0.0277383],
                      [0.0273439,   0.956078,  -0.291834 ] ])

    euler = tf.transformations.euler_from_matrix(R, 'rzyx')

    q = tf.transformations.quaternion_from_matrix(T)

    # print euler[0], euler[1], euler[2]

    # # print tf.transformations.rotation_matrix(0.123, (1,2,3))
    # print q
    # print q[1], q[2], q[3], q[0]

    numpy.set_printoptions(suppress=True)

    rx = 1.58
    ry = 1.61
    rz = -0.01
    x = -0.91
    y = -0.13
    z = 0.0

    T = tf.transformations.euler_matrix(rx, ry, rz, 'rxyz')
    T[0, 3] = -x
    T[1, 3] = -y
    T[2, 3] = z

    print "T\n", T



    

if __name__ == "__main__":
    main()
