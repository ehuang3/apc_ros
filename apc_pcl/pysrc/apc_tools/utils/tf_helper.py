import numpy as np

def xyzharray(msg):
    return np.array([msg.x, msg.y, msg.z, 1.0], np.float32)

def xyzarray(msg):
    return np.array([msg.x, msg.y, msg.z], np.float32)

def xyzwarray(msg):
    return np.array([msg.x, msg.y, msg.z, msg.w], np.float32)
