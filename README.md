PACKAGE DESCRIPTIONS:
==========

rgbd_numpy: ROS package for converting RGBD camera data between PointCloud2 msgs, Image msgs, and Numpy arrays. 

rospy: Contains numpy_msg tool, which generates (de)serializers that convert ROS msgs to numpy arrays.

        To test my numpy_msg mods for Image msgs:
          
	  -> roslaunch openni_launch openni.launch depth_registration:=true
	  -> rosrun rospy numpy_depth.py [or numpy_rgb.py]
	  -> rosrun image_view image_view image:=/image [you should see the image with a square redacted from the center]



