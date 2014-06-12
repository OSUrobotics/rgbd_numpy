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

import numpy
from cv_bridge import CvBridge
import cv

bridge = CvBridge()

# TODO: we will need to generate a new type structure with
# little-endian specified and then pass that type structure into the
# *_numpy calls.

def _serialize_numpy(self, buff):
    """
    wrapper for factory-generated class that passes numpy module into serialize
    """
    # for Image msgs
    if self._type == 'sensor_msgs/Image':
        self.data = bridge.cv_to_imgmsg(cv.fromarray(self.data), encoding=self.encoding).data
        
        
    return self.serialize_numpy(buff, numpy)  # serialize (with numpy wherever possible)


def _deserialize_numpy(self, str):
    """
    wrapper for factory-generated class that passes numpy module into deserialize    
    """
    self.deserialize_numpy(str, numpy)  # deserialize (with numpy wherever possible)

    # for Image msgs
    if self._type == 'sensor_msgs/Image':
        self.data = numpy.asarray(bridge.imgmsg_to_cv(self, desired_encoding=self.encoding))  # convert pixel data to numpy array
    
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
