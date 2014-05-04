#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import CameraInfo, Image
import numpy
import cv
from cv_bridge import CvBridge
import message_filters

import code # FOR TESTING


class ImageGrabber():
    def __init__(self, topics):
        
        # 
        self.bridge = CvBridge()

        # Flags
        self.need_rgb = True
        self.need_depth = True

        # RGB Subscribers
        sub_rgb = message_filters.Subscriber(topics['rgb'], Image)
        sub_rgb_info = message_filters.Subscriber(topics['rgb_info'], CameraInfo)
        ts_rgb = message_filters.TimeSynchronizer([sub_rgb, sub_rgb_info], 10)
        ts_rgb.registerCallback(self.rgb_callback)

        # Depth Subscribers
        sub_depth = message_filters.Subscriber(topics['depth'], Image)
        sub_depth_info = message_filters.Subscriber(topics['depth_info'], CameraInfo)
        ts_depth = message_filters.TimeSynchronizer([sub_depth, sub_depth_info], 10)
        ts_depth.registerCallback(self.depth_callback)

        self.pub_filtered = rospy.Publisher('/camera/rgb/image_color_filtered', Image)


    def rgb_callback(self, image_rgb, rgb_info):
        self.image_rgb = image_rgb
        self.rgb_info = rgb_info
        self.need_rgb = False

    def depth_callback(self, image_depth, depth_info):
        self.image_depth = image_depth
        self.depth_info = depth_info
        self.need_depth = False

    def convert_to_numpy(self):
        self.need_rgb = self.need_depth = True  # reset flags
        self.array_rgb = numpy.asarray(self.bridge.imgmsg_to_cv(self.image_rgb))
        self.array_depth = numpy.asarray(self.bridge.imgmsg_to_cv(self.image_depth, desired_encoding='16UC1'))

    def publish_rgb(self):
        image_cv = cv.fromarray(self.array_rgb)
        image_msg = self.bridge.cv_to_imgmsg(image_cv, encoding='bgr8')
        self.pub_filtered.publish(image_msg)
        

if __name__ == '__main__':

    rospy.init_node('python_images')

    topics = {'rgb' : '/camera/rgb/image_color',
              'rgb_info' : '/camera/rgb/camera_info',
              'depth' : '/camera/depth_registered/image_raw',
              'depth_info' : '/camera/depth_registered/camera_info'}

    image_grabber = ImageGrabber(topics)

    while not rospy.is_shutdown():
        if not image_grabber.need_rgb and not image_grabber.need_depth:

            image_grabber.convert_to_numpy()

            # APPLY FILTER HERE
            image_grabber.array_depth[numpy.isnan(image_grabber.array_depth)] = 0.0
            mask = numpy.logical_or(image_grabber.array_depth > 2000, image_grabber.array_depth == 0)   # 2000mm = 2m
            for i in range(image_grabber.array_rgb.shape[2]): 
                image_grabber.array_rgb[:, :, i][mask] = 0

            image_grabber.publish_rgb()
