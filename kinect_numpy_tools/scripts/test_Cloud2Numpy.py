#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2  # MAGICAL classes for PointCloud2 handling
import numpy
from kinect_numpy_tools.numpy_msg import numpy_msg

import code  # HACK!!!

class CloudGrabber():
    def __init__(self):
        self.pub = rospy.Publisher('/cloud_out', numpy_msg(PointCloud2))
        self.sub = rospy.Subscriber('/cloud_in', numpy_msg(PointCloud2), self.cloud_callback, queue_size=1)

    def cloud_callback(self, cloud):
        """ Build and publish new PointCloud2. """
        print 'Cloud has hit the node!'
        code.interact(local=locals()
        self.pub.publish(cloud)
        print 'Cloud away!\n'
        

if __name__ == '__main__':
    rospy.init_node('test_Cloud2Numpy')
    cloud_grabber = CloudGrabber()
    rospy.spin()
