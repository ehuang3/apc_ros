import rosbag
import os 
from . import path_to_root
from sensor_msgs.msg import PointCloud2

def load_background():
    '''Returns a background cloud, a background image, and the pose the original background was taken at'''
    background_path = os.path.join(path_to_root(), 'apc_pcl', 'calibration', 'background.bag')
    returns = []
    bag = rosbag.Bag(background_path)
    backgrounds = ['background_cloud', 'background_image', 'background_pose']
    for background_topic in backgrounds:
        messages = bag.read_messages(topics=background_topic)
        for topic, message, t in messages:
            pass
        returns.append(message)
    bag.close()
    return returns
