#!/usr/bin/python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
'''ros_image_pass
This is a module that has an image publisher and an image reciever object that does shit in ROS
'''

def make_image_msg(cv_image, encoding='bgr8'):
    bridge = CvBridge()    
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding)#,desired_encoding="passthrough")
    return image_message

def get_image_msg(ros_image, encoding='bgr8'):
    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(ros_image, desired_encoding=encoding)

class Image_Publisher(object):
    def __init__(self, topic="camera", encoding="bgr8", queue_size=1):
        '''Image Publisher -> Image_Publisher('/camera')'''
        self.im_pub = rospy.Publisher(topic, Image, queue_size=queue_size)
        self.bridge = CvBridge()    
        self.encoding = encoding
    
    def publish(self, cv_image):
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, self.encoding)#,desired_encoding="passthrough")
            self.im_pub.publish(image_message)
        except CvBridgeError, e:
            print e


class Image_Subscriber(object):
    def __init__(self, topic="camera", callback=None, encoding="bgr8", queue_size=1):
        '''Image_Subscriber('/camera', callback_function)
        Will call `callback_function` on each image every time a new image is published on `topic`
        Assumes topic of type "sensor_msgs/Image"'''
        self.encoding = encoding
        self.im_sub = rospy.Subscriber(topic, Image, self.convert, queue_size=queue_size)
        self.bridge = CvBridge()
        self.image = None
        self.callback = callback

    def convert(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
            self.callback(self.image)
        except CvBridgeError, e:
            print e


class Image_Clicker(object):
    def __init__(self, topic, left_callback, right_callback=None, encoding="bgr8", window_name="my_image"):
        self.window_name = window_name
        cv2.namedWindow(self.window_name)
        self.left_callback = left_callback
        self.right_callback = right_callback
        cv2.setMouseCallback(self.window_name, self.mouse_call)
        self.im_sub = Image_Subscriber(topic, self.got_image)
        self.image = None

    def got_image(self, msg):
        self.image = msg

    def mouse_call(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.left_callback((x, y))
        if event == cv2.EVENT_RBUTTONDOWN:
            if self.right_callback is not None:
                self.right_callback((x, y))

    def run(self):
        rate = rospy.Rate(10)
        while (not rospy.is_shutdown()):

            if self.image is None:
                continue

            image = np.copy(self.image)
            state = self.states[self.state].replace('_', ' ')
            cv2.putText(image, 'Click the {} corner'.format(state), (10, 500), self.font, 4, (255, 100, 80), 2)
            self.draw_corners(image)

            if self.is_done:
                cv2.polylines(image, np.int32([self.corners]), True, (0, 255, 0), 6)

            cv2.imshow(self.window_name, image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                rate.sleep()