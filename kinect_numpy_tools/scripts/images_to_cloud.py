#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from image_geometry import PinholeCameraModel
import numpy
import message_filters
from kinect_numpy_tools.numpy_msg import numpy_msg

import code  # HACK!!

class ImageGrabber():
    def __init__(self, topics):
        
        # Flags
        self.need_rgb = True
        self.need_depth = True

        # RGB Subscribers
        sub_rgb = message_filters.Subscriber(topics['rgb'], numpy_msg(Image))
        sub_rgb_info = message_filters.Subscriber(topics['rgb_info'], CameraInfo)
        ts_rgb = message_filters.TimeSynchronizer([sub_rgb, sub_rgb_info], 100)
        ts_rgb.registerCallback(self.rgb_callback)

        # Depth Subscribers
        sub_depth = message_filters.Subscriber(topics['depth'], numpy_msg(Image))
        sub_depth_info = message_filters.Subscriber(topics['depth_info'], CameraInfo)
        ts_depth = message_filters.TimeSynchronizer([sub_depth, sub_depth_info], 100)
        ts_depth.registerCallback(self.depth_callback)

        self.pub_cloud = rospy.Publisher('/camera/xyz_rgb/points', numpy_msg(PointCloud2))


    def rgb_callback(self, image_rgb, rgb_info):
        self.image_rgb = image_rgb
        self.rgb_info = rgb_info
        self.need_rgb = False

    def depth_callback(self, image_depth, depth_info):
        image_depth.data[numpy.isnan(image_depth.data)] = 0.0  # filter out NANs
        self.image_depth = image_depth
        self.depth_info = depth_info
        self.need_depth = False

    def convert_to_xyz(self):
        self.array_xyz = numpy.zeros(self.array_rays.shape)
        for i in range(self.array_rays.shape[2]): 
            self.array_xyz[:, :, i] = self.image_depth.data * self.array_rays[:, :, i]
        self.array_xyz /= 1000  # convert from millimeters -> meters

    def get_rays(self):
        model = PinholeCameraModel()
        model.fromCameraInfo(self.depth_info)
        self.array_rays = numpy.zeros((self.image_depth.height, self.image_depth.width, 3))
        
        for u in range(self.image_depth.height):
            for v in range(self.image_depth.width):
                ray = model.projectPixelTo3dRay((u, v))
                ray_z = [el / ray[2] for el in ray]  # normalize the ray so its Z-component equals 1.0
                self.array_rays[u, v, :] = ray_z

        

if __name__ == '__main__':

    rospy.init_node('rgbd_to_xyz_converter')

    topics = {'rgb' : '/camera/rgb/image_color',
              'rgb_info' : '/camera/rgb/camera_info',
              'depth' : '/camera/depth_registered/image_raw',
              'depth_info' : '/camera/depth_registered/camera_info'}

    image_grabber = ImageGrabber(topics)
                                        
    while image_grabber.need_depth:  # wait for first depth image
        rospy.sleep(0.01)

    rospy.loginfo('Got depth image; processing...')
    image_grabber.get_rays()  # make map of 3D rays for each pixel for converting depth values -> XYZ points
    rospy.loginfo('Made array of all pixel->ray correspondences!')
        
    while not rospy.is_shutdown():  # Do RGBD->XYZ conversions forever

        if not image_grabber.need_rgb and not image_grabber.need_depth:  # if have both RGB and Depth image

            image_grabber.convert_to_xyz()

            # Initialize the PointCloud2 msg
            cloud = PointCloud2()
            cloud.header.stamp = image_grabber.image_rgb.header.stamp
            cloud.header.frame_id = image_grabber.image_rgb.header.frame_id
            cloud.height = image_grabber.image_rgb.height
            cloud.width = image_grabber.image_rgb.width
            cloud.point_step = 32
            cloud.row_step = 20480

            # Jam the numpy XYZ/RGB data into the cloud
            points_arr = image_grabber.array_xyz.astype('float32')
            image_rgb = image_grabber.image_rgb.data
            cloud.data = numpy.rec.fromarrays((points_arr, image_rgb), names=('xyz', 'rgb'))
            
            # Generate 'fields' attribute of PointCloud2 according to Kinect conventions
            cloud.fields.append( PointField(name='x',   offset=0,  datatype=7, count=1) )
            cloud.fields.append( PointField(name='y',   offset=4,  datatype=7, count=1) )
            cloud.fields.append( PointField(name='z',   offset=8,  datatype=7, count=1) )
            cloud.fields.append( PointField(name='rgb', offset=16, datatype=7, count=1) )

            # Publish!
            image_grabber.pub_cloud.publish(data=cloud.data, fields=cloud.fields, header=cloud.header, height=cloud.height, width=cloud.width, point_step=cloud.point_step, row_step=cloud.row_step)
            image_grabber.need_rgb = True   # reset flags
            image_grabber.need_depth = True

