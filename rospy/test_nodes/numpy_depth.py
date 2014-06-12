#!/usr/bin/env python
from rospy.numpy_msg import numpy_msg
import numpy

if __name__ == '__main__':
  from sensor_msgs.msg import Image
  import rospy
  def image_callback(image):
    rospy.loginfo('Got a depth image!')
    data = numpy.zeros([480,640,3], dtype='uint8')
    image.data /= 1000  # scale to meters (??)
    for i in range(3):
      data[:, :, i] = image.data.astype('uint8')

    image.data = data
    image.data[200:300, 300:400, :] = 0  # redact a rectangle
    image.encoding = 'bgr8'
    image.step = 1920
    pub.publish(image)
    rospy.loginfo('Sent an image!\n')
     

  rospy.init_node('test_numpy_depth')
  pub = rospy.Publisher('image', numpy_msg(Image))
  rospy.Subscriber('/camera/depth_registered/image_raw', numpy_msg(Image), image_callback)
  rospy.loginfo('Listening for an Image on topic /camera/depth_registered/image_raw')

  rospy.spin()
