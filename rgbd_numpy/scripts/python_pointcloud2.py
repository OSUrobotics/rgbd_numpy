#!/usr/bin/env python

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String
import numpy as np
import struct
from cv_bridge import CvBridge
import cv
import copy
import code  # HACK for debugging


def _float2rgb(x):
    rgb = struct.unpack('I', struct.pack('f', x))[0]
    b = (rgb >> 16) & 0x0000ff;
    g = (rgb >> 8)  & 0x0000ff;
    r = (rgb)       & 0x0000ff;
    return r,g,b

class CloudConverter():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/camera/depth_registered/image_filtered', Image)
        self.sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.cloud_callback)


    def cloud_callback(self, cloud):
        points = point_cloud2.read_points(cloud)
        points_list = np.asarray(list(points))
        points_arr = np.asarray(points_list)

        # Unpack RGB color info
        _float2rgb_vectorized = np.vectorize(_float2rgb)
        r, g, b = _float2rgb_vectorized(points_arr[:, 3])

        # Concatenate and Reshape
        r = np.expand_dims(r, 1)  # insert blank 3rd dimension (for concatenation)
        g = np.expand_dims(g, 1)  
        b = np.expand_dims(b, 1)  
        points_rgb = np.concatenate((points_arr[:, 0:3], r, g, b), axis=1)
        image_rgb = points_rgb.reshape(cloud.height, cloud.width, 6)
        z = copy.deepcopy(image_rgb[:, :, 2])  # get depth values (I think)
        image_np = copy.deepcopy(image_rgb[:, :, 3:].astype('uint8'))
        #code.interact(local=locals())
        
        # TWO-METER DISTANCE FILTER
        z[np.isnan(z)] = 0.0
        mask = np.logical_or(z > 2, z == 0)
        for i in range(image_np.shape[2]): 
            image_np[:, :, i][mask] = 0
        
        # Convert to Image msg
        image_cv = cv.fromarray(image_np)
        image_msg = self.bridge.cv_to_imgmsg(image_cv, encoding='bgr8')
        self.pub.publish(image_msg)



if __name__ == '__main__':

    rospy.init_node('point_cloud2_test')

    myCloudConverter = CloudConverter()

    rospy.spin()
