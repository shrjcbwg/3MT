#!/usr/bin/python2.7
#### This is a modified version for AirSim/PythonClient/multirotor/kinect_publisher_fast.py

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
from sensor_msgs.msg import Imu
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix

CLAHE_ENABLED = False  # when enabled, RGB image is enhanced using CLAHE

CAMERA_FX = 320
CAMERA_FY = 320
CAMERA_CX = 320
CAMERA_CY = 240

CAMERA_K1 = -0.000591
CAMERA_K2 = 0.000519
CAMERA_P1 = 0.000001
CAMERA_P2 = -0.000030
CAMERA_P3 = 0.0

IMAGE_WIDTH = 640  # resolution should match values in settings.json
IMAGE_HEIGHT = 480

pointcloud_msg = PointCloud2()
class KinectPublisher:
    def __init__(self):
        self.bridge_rgb = CvBridge()
        self.msg_rgb = Image()
        self.bridge_d = CvBridge()
        self.msg_d = Image()
        self.msg_info = CameraInfo()
        self.msg_tf = TFMessage()
        self.cam_pose_msg = PoseStamped()
        self.odom_msg = Odometry()
        self.odom_msg_ned = Odometry()
        self.br0 = tf2_ros.TransformBroadcaster()
        self.br1 = tf2_ros.TransformBroadcaster()
        self.br2 = tf2_ros.TransformBroadcaster()
        self.gps_msg = NavSatFix()
        self.imu_msg = Imu()

    def getDepthImage(self,response_d):
        img_depth = np.array(response_d.image_data_float, dtype=np.float32)
        img_depth = img_depth.reshape(response_d.height, response_d.width)
        return img_depth

    def getRGBImage(self,response_rgb):
        img1d = np.fromstring(response_rgb.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response_rgb.height, response_rgb.width, 3)
        img_rgb = img_rgb[..., :3][..., ::-1]
        return img_rgb

    def enhanceRGB(self,img_rgb):
        lab = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2LAB)
        lab_planes = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(10, 10))
        lab_planes[0] = clahe.apply(lab_planes[0])
        lab = cv2.merge(lab_planes)
        img_rgb = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return img_rgb

    def GetCurrentTime(self):
        self.ros_time = rospy.Time.now()

    def CreateRGBMessage(self,img_rgb):
        self.msg_rgb.header.stamp = self.ros_time
        self.msg_rgb.header.frame_id = "/camera_rgb_optical_frame"
        self.msg_rgb.encoding = "bgr8"
        self.msg_rgb.height = IMAGE_HEIGHT
        self.msg_rgb.width = IMAGE_WIDTH
        self.msg_rgb.data = self.bridge_rgb.cv2_to_imgmsg(img_rgb, "bgr8").data
        self.msg_rgb.is_bigendian = 0
        self.msg_rgb.step = self.msg_rgb.width * 3
        return self.msg_rgb

    def CreateDMessage(self,img_depth):
        self.msg_d.header.stamp = self.ros_time
        self.msg_d.header.frame_id = "/camera_rgb_optical_frame"
        # self.msg_d.header.frame_id = "/camera_link"
        self.msg_d.encoding = "32FC1"
        self.msg_d.height = IMAGE_HEIGHT
        self.msg_d.width = IMAGE_WIDTH
        self.msg_d.data = self.bridge_d.cv2_to_imgmsg(img_depth, "32FC1").data
        self.msg_d.is_bigendian = 0
        self.msg_d.step = self.msg_d.width * 4
        return self.msg_d

    def CreateInfoMessage(self):
        self.msg_info.header.frame_id = "camera_rgb_optical_frame"
        self.msg_info.height = self.msg_rgb.height
        self.msg_info.width = self.msg_rgb.width
        self.msg_info.distortion_model = "plumb_bob"

        self.msg_info.D.append(CAMERA_K1)
        self.msg_info.D.append(CAMERA_K2)
        self.msg_info.D.append(CAMERA_P1)
        self.msg_info.D.append(CAMERA_P2)
        self.msg_info.D.append(CAMERA_P3)

        self.msg_info.K[0] = CAMERA_FX
        self.msg_info.K[1] = 0
        self.msg_info.K[2] = CAMERA_CX
        self.msg_info.K[3] = 0
        self.msg_info.K[4] = CAMERA_FY
        self.msg_info.K[5] = CAMERA_CY
        self.msg_info.K[6] = 0
        self.msg_info.K[7] = 0
        self.msg_info.K[8] = 1

        self.msg_info.R[0] = 1
        self.msg_info.R[1] = 0
        self.msg_info.R[2] = 0
        self.msg_info.R[3] = 0
        self.msg_info.R[4] = 1
        self.msg_info.R[5] = 0
        self.msg_info.R[6] = 0
        self.msg_info.R[7] = 0
        self.msg_info.R[8] = 1

        self.msg_info.P[0] = CAMERA_FX
        self.msg_info.P[1] = 0
        self.msg_info.P[2] = CAMERA_CX
        self.msg_info.P[3] = 0
        self.msg_info.P[4] = 0
        self.msg_info.P[5] = CAMERA_FY
        self.msg_info.P[6] = CAMERA_CY
        self.msg_info.P[7] = 0
        self.msg_info.P[8] = 0
        self.msg_info.P[9] = 0
        self.msg_info.P[10] = 1
        self.msg_info.P[11] = 0

        self.msg_info.binning_x = self.msg_info.binning_y = 0
        self.msg_info.roi.x_offset = self.msg_info.roi.y_offset = self.msg_info.roi.height = self.msg_info.roi.width = 0
        self.msg_info.roi.do_rectify = False
        self.msg_info.header.stamp = self.msg_rgb.header.stamp
        return self.msg_info

    def CreateTFMessage(self):
        pose = self.cam_pose_msg.pose

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[0].header.stamp = self.ros_time
        self.msg_tf.transforms[0].header.frame_id = "/world"
        self.msg_tf.transforms[0].child_frame_id = "/map"
        self.msg_tf.transforms[0].transform.translation.x = 0.000
        self.msg_tf.transforms[0].transform.translation.y = 0.000
        self.msg_tf.transforms[0].transform.translation.z = 0.000
        self.msg_tf.transforms[0].transform.rotation.x = 0.00
        self.msg_tf.transforms[0].transform.rotation.y = 0.00
        self.msg_tf.transforms[0].transform.rotation.z = 0.00
        self.msg_tf.transforms[0].transform.rotation.w = 1.00

        # self.msg_tf.transforms.append(TransformStamped())
        # self.msg_tf.transforms[1].header.stamp = self.ros_time
        # self.msg_tf.transforms[1].header.frame_id = "/map"
        # self.msg_tf.transforms[1].child_frame_id = "/base_link"
        # self.msg_tf.transforms[1].transform.translation.x = pose.position.x - 0.5
        # self.msg_tf.transforms[1].transform.translation.y = pose.position.y
        # self.msg_tf.transforms[1].transform.translation.z = pose.position.z
        # self.msg_tf.transforms[1].transform.rotation.x = pose.orientation.x
        # self.msg_tf.transforms[1].transform.rotation.y = pose.orientation.y
        # self.msg_tf.transforms[1].transform.rotation.z = pose.orientation.z
        # self.msg_tf.transforms[1].transform.rotation.w = pose.orientation.w

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[1].header.stamp = self.ros_time
        self.msg_tf.transforms[1].header.frame_id = "/base_link"
        self.msg_tf.transforms[1].child_frame_id = "/camera_link"
        self.msg_tf.transforms[1].transform.translation.x = 0.500
        self.msg_tf.transforms[1].transform.translation.y = 0.000
        self.msg_tf.transforms[1].transform.translation.z = 0.000
        self.msg_tf.transforms[1].transform.rotation.x = 0.00
        self.msg_tf.transforms[1].transform.rotation.y = 0.00
        self.msg_tf.transforms[1].transform.rotation.z = 0.00
        self.msg_tf.transforms[1].transform.rotation.w = 1.00

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[2].header.stamp = self.ros_time
        self.msg_tf.transforms[2].header.frame_id = "/camera_link"
        self.msg_tf.transforms[2].child_frame_id = "/camera_rgb_frame"
        self.msg_tf.transforms[2].transform.translation.x = 0.000
        self.msg_tf.transforms[2].transform.translation.y = 0
        self.msg_tf.transforms[2].transform.translation.z = 0.000
        self.msg_tf.transforms[2].transform.rotation.x = 0.00
        self.msg_tf.transforms[2].transform.rotation.y = 0.00
        self.msg_tf.transforms[2].transform.rotation.z = 0.00
        self.msg_tf.transforms[2].transform.rotation.w = 1.00

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[3].header.stamp = self.ros_time
        self.msg_tf.transforms[3].header.frame_id = "/camera_rgb_frame"
        self.msg_tf.transforms[3].child_frame_id = "/camera_rgb_optical_frame"
        self.msg_tf.transforms[3].transform.translation.x = 0.000
        self.msg_tf.transforms[3].transform.translation.y = 0.000
        self.msg_tf.transforms[3].transform.translation.z = 0.000
        self.msg_tf.transforms[3].transform.rotation.x = -0.500
        self.msg_tf.transforms[3].transform.rotation.y = 0.500
        self.msg_tf.transforms[3].transform.rotation.z = -0.500
        self.msg_tf.transforms[3].transform.rotation.w = 0.500

        return self.msg_tf

    def publish_tf_msg(self):

        pose = self.odom_msg.pose.pose
        
        tf_drone = TransformStamped()
        # populate tf ros message
        tf_drone.header.stamp = rospy.Time.now()
        tf_drone.header.frame_id = "/map"
        tf_drone.child_frame_id = "/base_link"
        tf_drone.transform.translation.x = pose.position.x
        tf_drone.transform.translation.y = pose.position.y
        tf_drone.transform.translation.z = pose.position.z
        tf_drone.transform.rotation.x = pose.orientation.x
        tf_drone.transform.rotation.y = pose.orientation.y
        tf_drone.transform.rotation.z = pose.orientation.z
        tf_drone.transform.rotation.w = pose.orientation.w

        self.br0.sendTransform(tf_drone)


    def get_sim_pose(self,cameraInfo):
        pos_ned = cameraInfo.pose.position
        orientation_ned = cameraInfo.pose.orientation

        # pos, orientation = convert_ned_to_enu(pos_ned, orientation_ned)
        pos = airsim.Vector3r(pos_ned.x_val, - pos_ned.y_val, - pos_ned.z_val)
        # orientation = airsim.Quaternionr( orientation_ned.w_val, - orientation_ned.z_val, - orientation_ned.x_val, orientation_ned.y_val)
        orientation = airsim.Quaternionr( orientation_ned.x_val, orientation_ned.y_val, - orientation_ned.z_val, orientation_ned.w_val)
    
        # populate PoseStamped ros message
        self.cam_pose_msg.pose.position.x = pos.x_val
        self.cam_pose_msg.pose.position.y = pos.y_val
        self.cam_pose_msg.pose.position.z = pos.z_val
        self.cam_pose_msg.pose.orientation.w = orientation.w_val
        self.cam_pose_msg.pose.orientation.x = orientation.x_val
        self.cam_pose_msg.pose.orientation.y = orientation.y_val
        self.cam_pose_msg.pose.orientation.z = orientation.z_val
        self.cam_pose_msg.header.seq = 1
        self.cam_pose_msg.header.stamp = self.ros_time
        # self.cam_pose_msg.header.frame_id = "/map"
        self.cam_pose_msg.header.frame_id = "/world"
        return self.cam_pose_msg
    
    def get_odom(self,DroneState):

        self.odom_msg.pose.pose.position.x = DroneState.position.x_val
        self.odom_msg.pose.pose.position.y = - DroneState.position.y_val
        self.odom_msg.pose.pose.position.z = - DroneState.position.z_val
        self.odom_msg.pose.pose.orientation.x = DroneState.orientation.x_val
        self.odom_msg.pose.pose.orientation.y = DroneState.orientation.y_val
        self.odom_msg.pose.pose.orientation.z = - DroneState.orientation.z_val
        self.odom_msg.pose.pose.orientation.w = DroneState.orientation.w_val

        self.odom_msg.twist.twist.linear.x = DroneState.angular_velocity.x_val
        self.odom_msg.twist.twist.linear.y = - DroneState.angular_velocity.y_val
        self.odom_msg.twist.twist.linear.z = - DroneState.angular_velocity.z_val
        self.odom_msg.twist.twist.angular.x = - DroneState.linear_velocity.x_val
        self.odom_msg.twist.twist.angular.y = DroneState.linear_velocity.y_val
        self.odom_msg.twist.twist.angular.z = - DroneState.linear_velocity.z_val
        self.odom_msg.header.seq = 1
        self.odom_msg.header.stamp = self.ros_time
        self.odom_msg.header.frame_id = "/world"

        self.odom_msg_ned.pose.pose.position.x = DroneState.position.x_val
        self.odom_msg_ned.pose.pose.position.y = DroneState.position.y_val
        self.odom_msg_ned.pose.pose.position.z = DroneState.position.z_val
        self.odom_msg_ned.pose.pose.orientation.x = DroneState.orientation.x_val
        self.odom_msg_ned.pose.pose.orientation.y = DroneState.orientation.y_val
        self.odom_msg_ned.pose.pose.orientation.z = DroneState.orientation.z_val
        self.odom_msg_ned.pose.pose.orientation.w = DroneState.orientation.w_val

        self.odom_msg_ned.twist.twist.linear.x = DroneState.angular_velocity.x_val
        self.odom_msg_ned.twist.twist.linear.y = DroneState.angular_velocity.y_val
        self.odom_msg_ned.twist.twist.linear.z = DroneState.angular_velocity.z_val
        self.odom_msg_ned.twist.twist.angular.x = DroneState.linear_velocity.x_val
        self.odom_msg_ned.twist.twist.angular.y = DroneState.linear_velocity.y_val
        self.odom_msg_ned.twist.twist.angular.z = DroneState.linear_velocity.z_val
        self.odom_msg_ned.header.seq = 1
        self.odom_msg_ned.header.stamp = self.ros_time

        self.odom_msg.header.frame_id = "/world"

        # self.odom_msg.child_frame_id = "/base_link"
        return self.odom_msg, self.odom_msg_ned
        
    def get_geo_home_pt(self, gps):
        self.gps_msg.header.stamp = rospy.Time.now()
        self.gps_msg.header.frame_id = "/world"

        self.gps_msg.latitude = gps.latitude
        self.gps_msg.longitude = gps.longitude
        self.gps_msg.altitude = gps.altitude


        return self.gps_msg

    def depthConversion(self, PointDepth, f):
        H = PointDepth.shape[0]
        W = PointDepth.shape[1]
        i_c = float(H) / 2 - 1
        j_c = float(W) / 2 - 1
        columns, rows = np.meshgrid(np.linspace(0, W-1, num=W), np.linspace(0, H-1, num=H))
        DistanceFromCenter = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
        PlaneDepth = PointDepth / (1 + (DistanceFromCenter / f)**2)**(0.5)
        return PlaneDepth

    def creat_IMU_msg(self, IMUdata):
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = "/base_link"

        self.imu_msg.orientation.x = IMUdata.orientation.x_val
        self.imu_msg.orientation.y = IMUdata.orientation.y_val
        self.imu_msg.orientation.z = IMUdata.orientation.z_val
        self.imu_msg.orientation.w = IMUdata.orientation.w_val

        self.imu_msg.angular_velocity.x = IMUdata.angular_velocity.x_val
        self.imu_msg.angular_velocity.y = IMUdata.angular_velocity.y_val
        self.imu_msg.angular_velocity.z = IMUdata.angular_velocity.z_val

        self.imu_msg.linear_acceleration.x = IMUdata.linear_acceleration.x_val
        self.imu_msg.linear_acceleration.y = IMUdata.linear_acceleration.y_val
        self.imu_msg.linear_acceleration.z = IMUdata.linear_acceleration.z_val


        return self.imu_msg

if __name__ == "__main__":
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    rospy.init_node('airsim_publisher', anonymous=True)
    client.simSetCameraPose(0, airsim.to_quaternion(0, 0, 0))
    client.simSetCameraOrientation(0, airsim.to_quaternion(-math.pi/2, 0, 0))


    publisher_d = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1)
    publisher_rgb = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=1)
    publisher_info_depth = rospy.Publisher('/camera/depth_registered/camera_info', CameraInfo, queue_size=1)
    publisher_imu = rospy.Publisher('/airsim_kin/drone_1/imu', Imu,queue_size=1)
    publisher_pose = rospy.Publisher('/camera/pose', PoseStamped, queue_size=1)
      
    rate = rospy.Rate(30)  # 30hz
    pub = KinectPublisher()

    while not rospy.is_shutdown():
        responses = client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPerspective, True, False),
                                         airsim.ImageRequest("front_center_custom", airsim.ImageType.Scene, False, False)])
        img_depth = pub.getDepthImage(responses[0])
        img_rgb = pub.getRGBImage(responses[1])
        img2d_converted = pub.depthConversion(img_depth, CAMERA_FX)

        state = client.getMultirotorState("drone_1")
        camerainfo = client.simGetCameraInfo("front_center_custom", "drone_1")
        DroneState = client.simGetGroundTruthKinematics("drone_1")
        DronePose = client.simGetVehiclePose("drone_1")
        Home_GPS = client.getHomeGeoPoint("drone_1")
        IMUdata = client.getImuData(imu_name='', vehicle_name='drone_1')

        if CLAHE_ENABLED:
            img_rgb = pub.enhanceRGB(img_rgb)

        pub.GetCurrentTime()
        pub.publish_tf_msg()
        msg_rgb = pub.CreateRGBMessage(img_rgb)
        msg_d = pub.CreateDMessage(img_depth)
        msg_info = pub.CreateInfoMessage()
        msg_imu = pub.creat_IMU_msg(IMUdata)

        cam_pose_msg = pub.get_sim_pose(camerainfo)
        gps_msg = pub.get_geo_home_pt(Home_GPS)
        odom_msg,odom_msg_ned = pub.get_odom(DroneState)

        publisher_rgb.publish(msg_rgb)
        publisher_d.publish(msg_d)
        publisher_info_depth.publish(msg_info)
        publisher_imu.publish(msg_imu)
        publisher_pose.publish(cam_pose_msg)

        del pub.msg_info.D[:]
        del pub.msg_tf.transforms[:]

        rate.sleep()