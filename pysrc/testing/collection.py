#!/usr/bin/python2.7
import rospy
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import airsim
import cv2
import numpy as np
import math
import ros_numpy
from tf2_ros import transform_broadcaster
import tf2_ros

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix


if __name__ == '__main__':
    rospy.init_node('data_collection')
    client = airsim.MultirotorClient()

    rospy.Subscriber('/airsim_node/drone_1/odom_local_ned', Twist, vel_cmd_cb)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        

        rate.sleep()


