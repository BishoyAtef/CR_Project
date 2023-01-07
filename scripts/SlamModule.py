#!/usr/bin/env python3
import message_filters
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from skimage.draw import line
from cmp_robot.msg import Data


def get_robot_pose(odom):
    global x, y, theta, time
    if time == 0:
        time = odom.header.stamp
        return 0, 0
    dt = (odom.header.stamp - time).to_sec()
    time = odom.header.stamp
    
    # kallman gain
    k = 0.79
    # velocities from odometry frame to sensor frame
    vx = odom.twist.twist.linear.x * np.cos(odom.twist.twist.angular.z) - odom.twist.twist.linear.y * np.sin(odom.twist.twist.angular.z)
    if vx < 0.0001:
        vx = 0
    vy = odom.twist.twist.linear.x * np.sin(odom.twist.twist.angular.z) + odom.twist.twist.linear.y * np.cos(odom.twist.twist.angular.z)
    if vy < 0.0001:
        vy = 0
    w = odom.twist.twist.angular.z
    if w < 0.0001:
        w = 0
    xp = x + vx * dt
    yp = y + vy * dt
    thetap = theta + w * dt

    # correct robot pose
    x_sensor = odom.pose.pose.position.x
    y_sensor = odom.pose.pose.position.y
    sensor_theta = odom.pose.pose.orientation.z

    x = xp + k * (x_sensor - xp)
    y = yp + k * (y_sensor - yp)
    theta = thetap + k * (sensor_theta - thetap)
    return x, y

def l2p(log_odds):
    """Convert log-odds to probability."""
    return 1 - (1/(1+np.exp(log_odds)))

def p2l(prob):
    """Convert probability to log-odds."""
    return np.log(prob/(1-prob))


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

def callback_handler(data, odom):
    """callback function handler for the time synchronizer"""
    # loop over the world_points
    x, y = get_robot_pose(odom)
    for i, point_x in enumerate(data.points_x):
        rr, cc = line(int(round((x-map_center['x'])/map_resolution)), int(round((y-map_center['y'])/map_resolution)), int(round((point_x-map_center['x'])/map_resolution)), int(round((data.points_y[i]-map_center['y'])/map_resolution)))
        # loop over the line and draw it
        for j in range(len(rr)-1):
            # chec if teh pixel is inside the map
            if is_inside(int(rr[j]),int(cc[j])):
                world[rr[j], cc[j]] += sensor_model_l_free - sensor_model_l_prior
        # draw the last pixel of the line with different color
        if is_inside(int(rr[-1]),int(cc[-1])):
            world[int(rr[-1]),int(cc[-1])] += sensor_model_l_occ - sensor_model_l_prior
    gridmap = world.copy()
    gridmap = gridmap.T
    gridmap_p = l2p(gridmap)
    gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
    map_msg.data = gridmap_int8.flatten()
    map_msg.header.stamp = data.stamp
    grid_pub.publish(map_msg)
    rospy.loginfo_once("Published the map")
   
if __name__ == '__main__':
    x, y, theta, time = 0, 0, 0, 0
    map_center = {'x':-50, 'y':-50}
    map_resolution = 0.2
    map_width = 500
    map_height = 500
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
    rospy.init_node('SlamModule', anonymous=True)
    rospy.loginfo('SlamModule node started')
    rate = rospy.Rate(12.5) # 10hz
    # Create a subscriber to the front laser scan topic
    data_sub = message_filters.Subscriber('/cmp_robot_data', Data)
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    grid_pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=50)
    # create a time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([data_sub, odom_sub],
                                                    10, 0.1, allow_headerless=True)
    # Register a callback with the time synchronizer
    ts.registerCallback(callback_handler)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()
