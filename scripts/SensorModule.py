#!/usr/bin/env python3
import message_filters
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from cmp_robot.msg import Data

def get_transform_cloud(source_frame):
    """Get the transform from the robot frame to the laser frame."""
    # get the transform from the robot frame to the laser frame
    world_points = list(point_cloud2.read_points(source_frame, field_names=("x", "y"), skip_nans=False))
    return world_points

def quarternion_to_yaw(qx, qy, qz, qw):
    """Convert quarternion to yaw angle."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)

def callback_handler(laser_scan, cloud_scan, odom):
    # extract the points from the cloud
    world_points = get_transform_cloud(cloud_scan)
    # create a message to publish
    msg = Data()
    # fill the message
    msg.stamp = cloud_scan.header.stamp
    msg.sensor_ranges = laser_scan.ranges
    msg.points_x = [point[0] for point in world_points]
    msg.points_y = [point[1] for point in world_points]
    msg.robot_x = odom.pose.pose.position.x
    msg.robot_y = odom.pose.pose.position.y
    msg.robot_theta = quarternion_to_yaw(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    data_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('SensorModule', anonymous=True)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    rospy.loginfo("SensorModule node started")
    # Create a subscriber to the multi laser scan topic
    laser_scan_sub = message_filters.Subscriber('/scan_multi', LaserScan)
    laser_cloud_sub = message_filters.Subscriber('/merged_cloud', PointCloud2)
    # Create a subscriber to the odometry topic
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    # Create a publisher to the data topic
    data_pub = rospy.Publisher('/cmp_robot_data', Data, queue_size=10)
    # Create a time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([laser_scan_sub, laser_cloud_sub, odom_sub],
                                                    10, 0.1, allow_headerless=True)
    # Register a callback with the time synchronizer
    ts.registerCallback(callback_handler)
    rospy.spin()
