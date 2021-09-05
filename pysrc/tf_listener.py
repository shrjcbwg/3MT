#!/usr/bin/python2.7

import rospy
import math
import tf
import geometry_msgs.msg
# import pcl

def quat2rot(quat_):
	q = quat_
	n = np.dot(q, q)
	if n < np.finfo(q.dtype).eps:
		return np.identity(3)
	q = q * np.sqrt(2.0 / n)
	q = np.outer(q, q)
	rot_matrix = np.array(
	[[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0]],
	 [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0]],
	 [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2]]],
	dtype=q.dtype)

	return rot_matrix

if __name__ == '__main__':
    rospy.init_node('camera_pose_tf')

    # parent_frame = rospy.get_param('~parent_frame', 'map')
    # camera_frame = rospy.get_param('~child_frame','camera_link')
    # pose_topic = rospy.get_param('~pose_topic','camera/pose')

    # pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)

    listener = tf.TransformListener()

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            # (trans,quat) = listener.lookupTransform('/world_ned', '/front_center_custom_optical', rospy.Time(0))
			(trans,quat) = listener.lookupTransform('/world', '/camera_rgb_optical_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans)
		
		# rate.sleep()
