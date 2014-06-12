#!/usr/bin/env python
from rospy.numpy_msg import numpy_msg
import numpy

if __name__ == '__main__':
  from sensor_msgs.msg import Image
  import rospy
  def image_callback(image):
    rospy.loginfo('Got an RGB image!')
    image.data[200:300, 300:400, :] = 0  # redact a rectangle
    pub.publish(image)
    rospy.loginfo('Sent an image!\n')
     

  rospy.init_node('test_numpy_msg')
  pub = rospy.Publisher('image', numpy_msg(Image))
  rospy.Subscriber('/camera/rgb/image_color', numpy_msg(Image), image_callback)
  rospy.loginfo('Listening for an Image on topic /camera/rgb/image_color')

  rospy.spin()
