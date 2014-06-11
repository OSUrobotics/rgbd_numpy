#!/usr/bin/env python
from sensor_msgs.msg import PointCloud2, PointField, Image
import numpy as np
import struct
import cv
from cv_bridge import CvBridge

import copy 
import code  # HACK for debugging

fmt_full = ''

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

_NP_TYPES = {
    np.dtype('uint8')   :   (PointField.UINT8,  1),
    np.dtype('int8')    :   (PointField.INT8,   1),
    np.dtype('uint16')  :   (PointField.UINT16, 2),
    np.dtype('int16')   :   (PointField.INT16,  2),
    np.dtype('uint32')  :   (PointField.UINT32, 4),
    np.dtype('int32')   :   (PointField.INT32,  4),
    np.dtype('float32') :   (PointField.FLOAT32,4),
    np.dtype('float64') :   (PointField.FLOAT64,8)
}

def pointcloud2_to_array(msg):
    global fmt_full
    if not fmt_full:
        fmt = _get_struct_fmt(msg)
        fmt_full = '>' if msg.is_bigendian else '<' + fmt.strip('<>')*msg.width*msg.height
    # import pdb; pdb.set_trace()
    unpacker = struct.Struct(fmt_full)
    unpacked = np.asarray(unpacker.unpack_from(msg.data))
    unpacked = unpacked.reshape(msg.height, msg.width, len(msg.fields))

    # Unpack RGB color info
    _float2rgb_vectorized = np.vectorize(_float2rgb)
    r, g, b = _float2rgb_vectorized(unpacked[:, :, 3])
    z = np.expand_dims(copy.deepcopy(unpacked[:, :, 2]), 2)
    r = np.expand_dims(r, 2)  # insert blank 3rd dimension (for concatenation)
    g = np.expand_dims(g, 2)  
    b = np.expand_dims(b, 2)  
    unpacked = np.concatenate((unpacked[:, :, 0:3], r, g, b), axis=2)
    return unpacked

def _get_struct_fmt(cloud, field_names=None):
    fmt = '>' if cloud.is_bigendian else '<'
    offset = 0
    for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def _float2rgb(x):
    rgb = struct.unpack('I', struct.pack('f', x))[0]
    b = (rgb >> 16) & 0x0000ff;
    g = (rgb >> 8)  & 0x0000ff;
    r = (rgb)       & 0x0000ff;
    return r,g,b


######### ROS NODE FOR TESTING ################

image_pub = None
bridge = CvBridge()

def cloud_cb(msg):
    arr = pointcloud2_to_array(msg)
    image_np = copy.deepcopy(arr[:, :, 3:].astype('uint8'))
    image_cv = cv.fromarray(image_np)
    image_msg = bridge.cv_to_imgmsg(image_cv, encoding='rgb8')
    image_pub.publish(image_msg)

if __name__ == '__main__':
    import rospy
    from std_msgs.msg import Empty
    rospy.init_node('test_pointclouds')
    image_pub = rospy.Publisher('/camera/depth_registered/points_image', Image)
    rospy.Subscriber('/camera/depth_registered/points', PointCloud2, cloud_cb)
    rospy.spin()
