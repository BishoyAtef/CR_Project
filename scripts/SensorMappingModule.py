#!/usr/bin/env python3

import numpy as np
import message_filters
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from skimage.draw import line

def l2p(l):
    """Convert log-odds to probability."""
    return 1 - (1/(1+np.exp(l)))

def p2l(p):
    """Convert probability to log-odds."""
    return np.log(p/(1-p))


def is_inside (i, j):
    """Check if grid coordinates are inside the map."""
    return i < world.shape[0] and j < world.shape[1] and i>=0 and j>=0

def bresenham (i0, j0, i1, j1, d):   # i0, j0 (starting point)
    """Bresenham's line algorithm."""
    dx = np.absolute(j1-j0)
    dy = -1 * np.absolute(i1-i0)
    sx = -1
    if j0<j1:
        sx = 1
    sy = -1
    if i0<i1:
        sy = 1
    jp, ip = j0, i0
    err = dx+dy                     # error value e_xy
    while True:                     # loop
        if (jp == j1 and ip == i1) or (np.sqrt((jp-j0)**2+(ip-i0)**2) >= d) or not is_inside(ip, jp):
            return ip, jp, False
        elif world[int(ip),int(jp)]==100:
            return ip, jp, True
        if is_inside(ip, jp):
            # miss:
            world[int(ip),int(jp)] += sensor_model_l_free - sensor_model_l_prior
        e2 = 2*err
        if e2 >= dy:                # e_xy+e_x > 0 
            err += dy
            jp += sx
        if e2 <= dx:                # e_xy+e_y < 0
            err += dx
            ip += sy


def get_transform_cloud(source_frame):
    """Get the transform from the robot frame to the laser frame."""
    # get the transform from the robot frame to the laser frame
    cloud = LaserProjection().projectLaser(source_frame)
    world_points = list(point_cloud2.read_points(cloud, field_names=("x", "y"), skip_nans=False))
    return world_points


def callback_handler(multi_scan,  odom):
    """callback function handler for the time synchronizer"""
    # rospy.loginfo("Callback handler called")
    world_points = get_transform_cloud(multi_scan)
    # loop over the world_points
    for i, point in enumerate(world_points):
        rr, cc = line(int(round((odom.pose.pose.position.x-map_center['x'])/map_resolution)), int(round((odom.pose.pose.position.y-map_center['y'])/map_resolution)), int(round((point[0]-map_center['x'])/map_resolution)), int(round((point[1]-map_center['y'])/map_resolution)))
        for i in range(len(rr)-1):
            if is_inside(int(rr[i]),int(cc[i])):
                world[rr[i], cc[i]] += sensor_model_l_free - sensor_model_l_prior
        # draw the last pixel of the line with different color
        if is_inside(int(rr[-1]),int(cc[-1])):
            world[int(rr[-1]),int(cc[-1])] += sensor_model_l_occ - sensor_model_l_prior
    gridmap = world.copy()
    gridmap = gridmap.T
    gridmap_p = l2p(gridmap)
    gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
    map_msg.data = gridmap_int8.flatten()
    map_msg.header.stamp = multi_scan.header.stamp
    grid_pub.publish(map_msg)
    # rospy.loginfo("Published the map")
    

if __name__ == '__main__':
    map_resolution = 0.2
    map_width = 500
    map_height = 500
    map_center = {'x':(-map_width/2)*map_resolution, 'y':(-map_height/2)*map_resolution}
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'robot_map'
    map_msg.info.resolution = map_resolution
    map_msg.info.width = map_width
    map_msg.info.height = map_height
    map_msg.info.origin.position.x = map_center['x']
    map_msg.info.origin.position.y = map_center['y']
    sensor_model_l_occ = p2l(0.8)
    sensor_model_l_prior = p2l(0.5)
    sensor_model_l_free = p2l(0.45)
    world = sensor_model_l_prior * np.ones((map_height, map_width), dtype=np.float32)
    rospy.init_node('mapping_node', anonymous=True)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    rospy.loginfo("mapping_node node started")
    # Create a subscriber to the front laser scan topic
    multi_laser_sub = message_filters.Subscriber('/scan_multi', LaserScan)
    # Create a publisher to the occupancy grid topic
    grid_pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=50)
    # Create a subscriber to the odometry topic
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    # Create a time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([multi_laser_sub, odom_sub],
                                                    100, 0.1, allow_headerless=True)
    # Register a callback with the time synchronizer
    ts.registerCallback(callback_handler)
    rospy.spin()
