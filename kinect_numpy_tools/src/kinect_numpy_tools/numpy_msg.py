# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$
# 
# Modified by Matthew Rueben
# Oregon State University
# Beginning in June, 2014



"""
Support for using numpy with rospy messages.

For tutorials, see U{http://www.ros.org/wiki/rospy_tutorials/Tutorials/numpy}

Modified to work for Image messages by Matthew Rueben.

"""

# TODO: we will need to generate a new type structure with
# little-endian specified and then pass that type structure into the
# *_numpy calls.

import rospy
import numpy
from cv_bridge import CvBridge
import cv
import struct
from sensor_msgs import point_cloud2

import code  # HACK!!!

bridge = CvBridge()


def _float2rgb(x):  # unpack single FLOAT value from Kinect's PointCloud2 into UINT8 <r,g,b> values
    rgb = struct.unpack('I', struct.pack('f', x))[0]
    b = (rgb >> 16) & 0x0000ff;
    g = (rgb >> 8)  & 0x0000ff;
    r = (rgb)       & 0x0000ff;
    return r,g,b

def _rgb2float(r, g, b):  # opposite of above
    x = 1.00  # BLUE!
    return x


def _serialize_numpy(self, buff):
    """
    wrapper for factory-generated class that passes numpy module into serialize
    """

    # for Image msgs
    if self._type == 'sensor_msgs/Image':
        self.data = bridge.cv_to_imgmsg(cv.fromarray(self.data), encoding=self.encoding).data

    # for PointCloud2 msgs
    if self._type == 'sensor_msgs/PointCloud2':
        print 'Cloud is being serialized...'

        # Pack each RGB triple into a single float
        _rgb2float_vectorized = numpy.vectorize(_rgb2float)
        rgb = _rgb2float_vectorized(self.data.rgb[:, :, 0],
                                    self.data.rgb[:, :, 1],
                                    self.data.rgb[:, :, 2])

        rgb[:, :320] = 0.0  # turn left half WHITE

        # Reshape to a list of (x, y, z, rgb) tuples
        xyz = self.data.xyz.reshape(self.height * self.width, 3)
        rgb = rgb.reshape(self.height * self.width)
        rgb = numpy.expand_dims(rgb, 1)  # insert blank 2nd dimension (for concatenation)
        self.data = numpy.concatenate((xyz, rgb), axis=1)

        # Make the PointCloud2 msg
        self.header.stamp = rospy.Time.now()
        self = point_cloud2.create_cloud(self.header, self.fields, self.data)


    return self.serialize_numpy(buff, numpy)  # serialize (with numpy wherever possible)


def _deserialize_numpy(self, str):
    """
    wrapper for factory-generated class that passes numpy module into deserialize    
    """
    self.deserialize_numpy(str, numpy)  # deserialize (with numpy wherever possible)

    # for Image msgs
    if self._type == 'sensor_msgs/Image':
        self.data = numpy.asarray(bridge.imgmsg_to_cv(self, desired_encoding=self.encoding))  # convert pixel data to numpy array

    # for PointCloud2 msgs
    if self._type == 'sensor_msgs/PointCloud2':
        print 'Cloud is being deserialized...'
        points = point_cloud2.read_points(self)
        points_arr = numpy.asarray(list(points))
        
        # Unpack RGB color info
        _float2rgb_vectorized = numpy.vectorize(_float2rgb)
        r, g, b = _float2rgb_vectorized(points_arr[:, 3])
        r = numpy.expand_dims(r, 1).astype('uint8')  # insert blank 3rd dimension (for concatenation)
        g = numpy.expand_dims(g, 1).astype('uint8')  
        b = numpy.expand_dims(b, 1).astype('uint8')  

        # Concatenate and Reshape
        pixels_rgb = numpy.concatenate((r, g, b), axis=1)
        image_rgb = pixels_rgb.reshape(self.height, self.width, 3)
        points_arr = points_arr[:, :3].reshape(self.height, self.width, 3).astype('float32')

        # Build record array to separate datatypes -- int16 for XYZ, uint8 for RGB
        image_xyzrgb = numpy.rec.fromarrays((points_arr, image_rgb), names=('xyz', 'rgb'))
        self.data = image_xyzrgb

    return self

    
## Use this function to generate message instances using numpy array
## types for numerical arrays. 
## @msg_type Message class: call this functioning on the message type that you pass
## into a Publisher or Subscriber call. 
## @returns Message class
def numpy_msg(msg_type):
    classdict = { '__slots__': msg_type.__slots__, '_slot_types': msg_type._slot_types,
                  '_md5sum': msg_type._md5sum, '_type': msg_type._type,
                  '_has_header': msg_type._has_header, '_full_text': msg_type._full_text,
                  'serialize': _serialize_numpy, 'deserialize': _deserialize_numpy,
                  'serialize_numpy': msg_type.serialize_numpy,
                  'deserialize_numpy': msg_type.deserialize_numpy
                  }

    # create the numpy message type
    msg_type_name = "Numpy_%s"%msg_type._type.replace('/', '__')
    return type(msg_type_name,(msg_type,),classdict)
