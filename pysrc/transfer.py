#!/usr/bin/python2.7
# airsim
import setup_path
import airsim
# standard python
import math
import sys
import numpy as np
from numpy.lib.stride_tricks import as_strided
# ROS
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
# ROS Image message
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

name_to_dtypes = {
	"rgb8":    (np.uint8,  3),
	"rgba8":   (np.uint8,  4),
	"rgb16":   (np.uint16, 3),
	"rgba16":  (np.uint16, 4),
	"bgr8":    (np.uint8,  3),
	"bgra8":   (np.uint8,  4),
	"bgr16":   (np.uint16, 3),
	"bgra16":  (np.uint16, 4),
	"mono8":   (np.uint8,  1),
	"mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
	"bayer_rggb8":	(np.uint8,  1),
	"bayer_bggr8":	(np.uint8,  1),
	"bayer_gbrg8":	(np.uint8,  1),
	"bayer_grbg8":	(np.uint8,  1),
	"bayer_rggb16":	(np.uint16, 1),
	"bayer_bggr16":	(np.uint16, 1),
	"bayer_gbrg16":	(np.uint16, 1),
	"bayer_grbg16":	(np.uint16, 1),

    # OpenCV CvMat types
	"8UC1":    (np.uint8,   1),
	"8UC2":    (np.uint8,   2),
	"8UC3":    (np.uint8,   3),
	"8UC4":    (np.uint8,   4),
	"8SC1":    (np.int8,    1),
	"8SC2":    (np.int8,    2),
	"8SC3":    (np.int8,    3),
	"8SC4":    (np.int8,    4),
	"16UC1":   (np.int16,   1),
	"16UC2":   (np.int16,   2),
	"16UC3":   (np.int16,   3),
	"16UC4":   (np.int16,   4),
	"16SC1":   (np.uint16,  1),
	"16SC2":   (np.uint16,  2),
	"16SC3":   (np.uint16,  3),
	"16SC4":   (np.uint16,  4),
	"32SC1":   (np.int32,   1),
	"32SC2":   (np.int32,   2),
	"32SC3":   (np.int32,   3),
	"32SC4":   (np.int32,   4),
	"32FC1":   (np.float32, 1),
	"32FC2":   (np.float32, 2),
	"32FC3":   (np.float32, 3),
	"32FC4":   (np.float32, 4),
	"64FC1":   (np.float64, 1),
	"64FC2":   (np.float64, 2),
	"64FC3":   (np.float64, 3),
	"64FC4":   (np.float64, 4)
}
pointcloud2_lidar_msg = PointCloud2()
odometry_enu_msg = Odometry()
vins_cam_pose_msg = PoseStamped()

def numpy_to_image(arr, encoding):
	if not encoding in name_to_dtypes:
		raise TypeError('Unrecognized encoding {}'.format(encoding))

	im = Image(encoding=encoding)

	# extract width, height, and channels
	dtype_class, exp_channels = name_to_dtypes[encoding]
	dtype = np.dtype(dtype_class)
	if len(arr.shape) == 2:
		im.height, im.width, channels = arr.shape + (1,)
	elif len(arr.shape) == 3:
		im.height, im.width, channels = arr.shape
	else:
		raise TypeError("Array must be two or three dimensional")

	# check type and channels
	if exp_channels != channels:
		raise TypeError("Array has {} channels, {} requires {}".format(
			channels, encoding, exp_channels
		))
	if dtype_class != arr.dtype.type:
		raise TypeError("Array is {}, {} requires {}".format(
			arr.dtype.type, encoding, dtype_class
		))

	# make the array contiguous in memory, as mostly required by the format
	contig = np.ascontiguousarray(arr)
	im.data = contig.tobytes()
	im.step = contig.strides[0]
	im.is_bigendian = (
		arr.dtype.byteorder == '>' or
		arr.dtype.byteorder == '=' and sys.byteorder == 'big'
	)

	return im

def convert_ned_to_enu(pos_ned, orientation_ned):
	pos_enu = airsim.Vector3r(  pos_ned.x_val,
				  - pos_ned.y_val,
				  - pos_ned.z_val)
	orientation_enu = airsim.Quaternionr( orientation_ned.w_val,
					    - orientation_ned.z_val,
					    - orientation_ned.x_val,
					      orientation_ned.y_val)
	return pos_enu, orientation_enu

def get_sim_pose_2(odometry_enu_msg):

    # populate PoseStamped ros message
    sim_pose_msg = PoseStamped()
    # sim_pose_msg.header.seq = 1
    sim_pose_msg.header = odometry_enu_msg.header
    sim_pose_msg.header.frame_id = "front_center_body"
    # sim_pose_msg.header.frame_id = "world"
    sim_pose_msg.pose = odometry_enu_msg.pose.pose
    return sim_pose_msg

def convert_euler(r,p,y):
    pose = Pose()
    DE2RA = math.pi /180
    roll = r
    pitch = p
    yaw = y
    q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose

def publish_tf_msg(odometry_enu_msg):

    pose = odometry_enu_msg.pose.pose

    br = tf2_ros.TransformBroadcaster()
    tf_drone = TransformStamped()
    # tf_camera_optical = TransformStamped()
    # populate tf ros message
    tf_drone.header.stamp = rospy.Time.now()
    tf_drone.header.frame_id = "world"
    tf_drone.child_frame_id = "drone_link"
    tf_drone.transform.translation.x = pose.position.x
    tf_drone.transform.translation.y = pose.position.y
    tf_drone.transform.translation.z = pose.position.z
    tf_drone.transform.rotation.x = pose.orientation.x
    tf_drone.transform.rotation.y = pose.orientation.y
    tf_drone.transform.rotation.z = pose.orientation.z
    tf_drone.transform.rotation.w = pose.orientation.w

    # tf_camera_optical.header.stamp = rospy.Time.now()
    # tf_camera_optical.header.frame_id = "drone_link"
    # tf_camera_optical.child_frame_id = "front_center_optical"
    # tf_camera_optical.transform.translation.x = 0.500
    # tf_camera_optical.transform.translation.y = 0.000
    # tf_camera_optical.transform.translation.z = 0.7714
    # tf_camera_optical.transform.rotation.x = 0.500
    # tf_camera_optical.transform.rotation.y = 0.500
    # tf_camera_optical.transform.rotation.z = 0.500
    # tf_camera_optical.transform.rotation.w = 0.500

    br.sendTransform(tf_drone)
    # br.sendTransform(tf_camera_optical)

    # print(pose.orientation)
    # return tf_drone,tf_camera_optical

def get_sim_pose(client):
    # get state of the multirotor
    camerainfo = client.simGetCameraInfo("front_center_custom", "drone_1")
    pos_ned = camerainfo.pose.position
    orientation_ned = camerainfo.pose.orientation
    
    # pos_ned = drone_state.kinematics_estimated.position
    # orientation_ned = drone_state.kinematics_estimated.orientation

    pos, orientation = convert_ned_to_enu(pos_ned, orientation_ned)

    # populate PoseStamped ros message
    sim_pose_msg = PoseStamped()
    sim_pose_msg.pose.position.x = pos.x_val
    sim_pose_msg.pose.position.y = pos.y_val
    sim_pose_msg.pose.position.z = pos.z_val
    sim_pose_msg.pose.orientation.w = orientation.w_val
    sim_pose_msg.pose.orientation.x = orientation.x_val
    sim_pose_msg.pose.orientation.y = orientation.y_val
    sim_pose_msg.pose.orientation.z = orientation.z_val
    sim_pose_msg.header.seq = 1
    sim_pose_msg.header = odometry_enu_msg.header
    sim_pose_msg.header.frame_id = "world"


    return sim_pose_msg

def Odometry_callback(odometry_ned_msg):
	global odometry_enu_msg
	odometry_enu_msg = odometry_ned_msg
	odometry_enu_msg = odometry_ned_msg
	odometry_enu_msg.pose.pose.position.y = - odometry_enu_msg.pose.pose.position.y
	# odometry_enu_msg.pose.pose.position.x, odometry_enu_msg.pose.pose.position.y = odometry_enu_msg.pose.pose.position.y, odometry_enu_msg.pose.pose.position.x
	odometry_enu_msg.pose.pose.position.z = - odometry_enu_msg.pose.pose.position.z
	odometry_enu_msg.pose.pose.orientation.y = - odometry_enu_msg.pose.pose.orientation.y
	# odometry_enu_msg.pose.pose.orientation.x, odometry_enu_msg.pose.pose.orientation.y = odometry_enu_msg.pose.pose.orientation.y, odometry_enu_msg.pose.pose.orientation.x
	odometry_enu_msg.pose.pose.orientation.z = - odometry_enu_msg.pose.pose.orientation.z
	odometry_enu_msg.twist.twist.linear.y = - odometry_enu_msg.twist.twist.linear.y
	# odometry_enu_msg.twist.twist.linear.x, odometry_enu_msg.twist.twist.linear.y = odometry_enu_msg.twist.twist.linear.y, odometry_enu_msg.twist.twist.linear.x
	odometry_enu_msg.twist.twist.linear.z = - odometry_enu_msg.twist.twist.linear.z
	odometry_enu_msg.twist.twist.angular.y = - odometry_enu_msg.twist.twist.angular.y
	# odometry_enu_msg.twist.twist.angular.x, odometry_enu_msg.twist.twist.angular.y = odometry_enu_msg.twist.twist.angular.y, odometry_enu_msg.twist.twist.angular.x
	odometry_enu_msg.twist.twist.angular.z = -odometry_enu_msg.twist.twist.angular.z
	odometry_enu_msg.header.frame_id = "world"
	# odometry_enu_msg.child_frame_id = "/quadrotor"

def Lidar_callback(Lidar_msg):
    global pointcloud2_lidar_msg
    pointcloud2_lidar_msg = Lidar_msg
    pointcloud2_lidar_msg.header.frame_id = "drone_link"
 
def VINS_Odometry_callback(vins_odo_msg):
	global  vins_cam_pose_msg
	vins_cam_pose_msg.header = vins_odo_msg.header
	vins_cam_pose_msg.pose = vins_odo_msg.pose.pose

def get_camera_params():
    # read parameters
    width = rospy.get_param('~width', 640)
    height = rospy.get_param('~height', 360)
    Fx = rospy.get_param('~Fx', 320)
    Fy = rospy.get_param('~Fy', 320)
    Cx = rospy.get_param('~Cx', 320)
    Cy = rospy.get_param('~Cy', 180)

    # create sensor message
    camera_info_msg = CameraInfo()
    camera_info_msg.distortion_model = 'plumb_bob'
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.K = [Fx, 0, Cx,
                         0, Fy, Cy,
                         0, 0, 1]
    camera_info_msg.D = [0, 0, 0, 0]

    camera_info_msg.P = [Fx, 0, Cx, 0,
                         0, Fy, Cy, 0,
                         0, 0, 1, 0]
    camera_info_msg.header.frame_id = "front_center_optical"
    return camera_info_msg

def get_image_messages(client):
    # get camera images from the multirotor
    responses = client.simGetImages([
       airsim.ImageRequest("front_center_custom", airsim.ImageType.Scene, False, False),
       airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPlanner, True)
    ])


    # convert depth float array to NumPy 2D array using
    depth_img = airsim.list_to_2d_float_array(responses[1].image_data_float, responses[1].width, responses[1].height)

    # Populate image message
    depth_msg = numpy_to_image(depth_img, "32FC1")
    depth_msg.header.frame_id = "camera"
    depth_msg.header.stamp = rospy.Time.now()

    return  depth_msg

def airpub():
    ## Start ROS ---------------------------------------------------------------
    rospy.init_node('airsim_img_publisher', anonymous=False)
    loop_rate = rospy.get_param('~loop_rate', 10)
    rate = rospy.Rate(loop_rate)

    ## Subscribers --------------------------------------------------------------
    rospy.Subscriber("/airsim_node/drone_1/odom_local_ned", Odometry, Odometry_callback)
    # rospy.Subscriber("/airsim_node/drone_1/lidar/LidarCustom", PointCloud2, Lidar_callback)



    rospy.Subscriber("/vins_estimator/camera_pose", Odometry, VINS_Odometry_callback)

    ## Publishers --------------------------------------------------------------
    # pose publisher
    pose_pub = rospy.Publisher("drone/pose", PoseStamped, queue_size=1)
    VINS_pose_pub = rospy.Publisher("vins_estimator/pose", PoseStamped, queue_size=1)
    depth_pub = rospy.Publisher("drone/depth", Image, queue_size=1)
    odom_pub = rospy.Publisher("drone/odom", Odometry, queue_size=1)
    cloud_pub = rospy.Publisher("drone/cloud", PointCloud2, queue_size=1)
	



    ## Main --------------------------------------------------------------------
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)


    while not rospy.is_shutdown():

        # camera_info_msg = get_camera_params()
        sim_pose_msg = get_sim_pose_2(odometry_enu_msg)
        # sim_pose_msg = get_sim_pose(client)
        depth_msg = get_image_messages(client)

        # header message
        # odometry_enu_msg.header.stamp = rospy.Time.now()
        sim_pose_msg.header.stamp = odometry_enu_msg.header.stamp
        depth_msg.header.stamp = sim_pose_msg.header.stamp
        depth_msg.header.frame_id = "front_center_optical"
        

        #TF message
        # tf_drone, tf_camera_optical = create_tf_msg()

        # publish message
        cloud_pub.publish(pointcloud2_lidar_msg)
        pose_pub.publish(sim_pose_msg)
        VINS_pose_pub.publish(vins_cam_pose_msg)
        # publish_tf_msg(odometry_enu_msg)		
        depth_pub.publish(depth_msg)        
        odom_pub.publish(odometry_enu_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass
