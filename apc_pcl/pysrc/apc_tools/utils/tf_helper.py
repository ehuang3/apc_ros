import numpy as np
from tf_conversions import posemath

def xyzharray(msg):
    return np.array([msg.x, msg.y, msg.z, 1.0], np.float32)

def xyzarray(msg):
    return np.array([msg.x, msg.y, msg.z], np.float32)

def xyzwarray(msg):
    return np.array([msg.x, msg.y, msg.z, msg.w], np.float32)

def pqfrompose(msg):
    return xyzharray(msg.position), xyzwarray(msg.orientation)

def pose_from_matrix(matrix):
    return posemath.toMsg(posemath.fromMatrix(matrix))