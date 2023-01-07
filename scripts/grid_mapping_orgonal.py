#!/usr/bin/env python3

import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf

# p(x) = 1 - \frac{1}{1 + e^l(x)}
def l2p(l):
    """Convert log-odds to probability."""
    return 1 - (1/(1+np.exp(l)))

# l(x) = log(\frac{p(x)}{1 - p(x)})
def p2l(p):
    """Convert probability to log-odds."""
    return np.log(p/(1-p))


class GridMapping:
    """Grid mapping class."""
    def __init__(self, map_center_x, map_center_y, map_size_x, map_size_y, map_resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist, sensor_model_p_occ, sensor_model_p_free, sensor_model_p_prior):
        """Initialize grid mapping class."""
        self.map_center_x = map_center_x          #meter
        self.map_center_y = map_center_y          #meter
        self.map_size_x = map_size_x              #meter
        self.map_size_y = map_size_y              #meter
        self.map_resolution = map_resolution      #meter/cell
        self.laser_min_angle = laser_min_angle    #radian
        self.laser_max_angle = laser_max_angle    #radian
        self.laser_resolution = laser_resolution  #radian
        self.laser_max_dist = laser_max_dist      #meter
        self.sensor_model_l_occ = p2l(sensor_model_p_occ)
        self.sensor_model_l_free = p2l(sensor_model_p_free)
        self.sensor_model_l_prior = p2l(sensor_model_p_prior)

        map_rows = int(map_size_y / map_resolution)
        map_cols = int(map_size_x / map_resolution)
        self.gridmap = self.sensor_model_l_prior * np.ones((map_rows, map_cols), dtype=np.float32)

    def to_xy (self, i, j):
        """Convert grid coordinates to world coordinates."""
        x = j * self.map_resolution + self.map_center_x
        y = i * self.map_resolution + self.map_center_y
        return x, y

    def to_ij (self, x, y):
        """Convert world coordinates to grid coordinates."""
        # rospy.loginfo(self.map_center_x)
        # rospy.loginfo(self.map_center_y)
        i = (y-self.map_center_y)  / self.map_resolution
        j = (x-self.map_center_x)  / self.map_resolution
        # rospy.loginfo(i)
        # rospy.loginfo(j)
        return i, j

    def is_inside (self, i, j):
        """Check if grid coordinates are inside the map."""
        return i < self.gridmap.shape[0] and j < self.gridmap.shape[1] and i>=0 and j>=0

    def raycast_update(self, x0, y0, theta, d):
        """Update the grid map using a raycast."""
        # rospy.loginfo("Raycasting update")
        # see: https://www.ros.org/reps/rep-0117.html
        # Detections that are too close to the sensor to quantify shall be represented by -Inf. 
        # Erroneous detections shall be represented by quiet (non-signaling) NaNs. 
        # Finally, out of range detections will be represented by +Inf.
        if np.isinf(d) and np.sign(d) == +1:
            d = self.laser_max_dist
        elif np.isinf(d) or np.isnan(d):
            return

        x1 = x0 + d*np.cos(theta)
        y1 = y0 + d*np.sin(theta)
        i0, j0 = self.to_ij(x0, y0)
        i1, j1 = self.to_ij(x1, y1)
        d_cells = d / self.map_resolution
        ip, jp, is_hit = self.bresenham(i0, j0, i1, j1, d_cells)
        if not np.isnan(d) and d != self.laser_max_dist and self.is_inside(int(ip),int(jp)):
            # Hit!
            self.gridmap[int(ip),int(jp)] += self.sensor_model_l_occ - self.sensor_model_l_prior
        return
    
    #bresenham method is used to plot the lines
    def bresenham (self, i0, j0, i1, j1, d, debug=False):   # i0, j0 (starting point)
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
            if (jp == j1 and ip == i1) or (np.sqrt((jp-j0)**2+(ip-i0)**2) >= d) or not self.is_inside(ip, jp):
                return ip, jp, False
            elif self.gridmap[int(ip),int(jp)]==100:
                return ip, jp, True

            if self.is_inside(ip, jp):
                # miss:
                self.gridmap[int(ip),int(jp)] += self.sensor_model_l_free - self.sensor_model_l_prior

            e2 = 2*err
            if e2 >= dy:                # e_xy+e_x > 0 
                err += dy
                jp += sx
            if e2 <= dx:                # e_xy+e_y < 0
                err += dx
                ip += sy

    def update(self, x, y, theta, scan):
        """Update the grid map using a laser scan."""
        # test by printing robot trajectory
        #i,j = self.to_ij(x,y)
        #self.gridmap[int(i), int(j)] = 100
        # rospy.loginfo("update grid map")
        for i, z in enumerate(scan):
            self.raycast_update(x, y, (theta + self.laser_min_angle + i*self.laser_resolution), z)
        return self.gridmap



class GridMappingROS:
    """ROS interface for grid mapping."""
    def __init__(self):
        rospy.init_node('RosGridMapping', anonymous=True)
        self.is_gridmapping_initialized = False
        self.map_last_publish = rospy.Time()
        self.prev_robot_x = -99999
        self.prev_robot_y = -99999

        self.sensor_model_p_occ   = rospy.get_param('sensor_model_p_occ', 0.75)
        print(self.sensor_model_p_occ)
        self.sensor_model_p_free  = rospy.get_param('sensor_model_p_free', 0.45)
        self.sensor_model_p_prior = rospy.get_param('sensor_model_p_prior', 0.5)
        self.robot_frame          = rospy.get_param('robot_frameeeeeee', 'robot_base_footprint')
        print(self.robot_frame)
        self.map_frame            = rospy.get_param('map_frame', 'map')
        print(self.map_frame)
        self.map_center_x         = rospy.get_param('map_center_xxxxx', -50)
        self.map_center_y         = rospy.get_param('map_center_yxxxx', -50)
        self.map_size_x           = rospy.get_param('mmmmmap_size_x', 100)
        print(self.map_size_x)
        self.map_size_y           = rospy.get_param('mmmmmmap_size_y', 100)
        self.map_resolution       = rospy.get_param('map_resolution', 0.02)
        self.map_publish_freq     = rospy.get_param('/robot/move_base/local_costmap/publish_frequency', 60.0)
        print(self.map_publish_freq)
        self.update_movement      = rospy.get_param('update', 0.1)

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = -50
        self.map_msg.info.origin.position.y = -50

        self.laser_sub = rospy.Subscriber("/robot/front_laser/scan", LaserScan, self.laserscan_callback, queue_size=2)
        self.map_pub = rospy.Publisher('/ourMap', OccupancyGrid, queue_size=2)
        self.tf_sub = tf.TransformListener()

    def init_gridmapping(self, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        """Initialize the grid mapping."""
        self.gridmapping = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y, self.map_resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist, self.sensor_model_p_occ, self.sensor_model_p_free, self.sensor_model_p_prior)
        self.is_gridmapping_initialized = True

    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    def quarternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quarternion to yaw angle."""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

    def publish_occupancygrid(self, gridmap, stamp):
        """Publish the occupancy grid map."""
        # Convert gridmap to ROS supported data type : int8[]
        # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
        gridmap_p = l2p(gridmap)
        #unknown_mask = (gridmap_p == self.sensor_model_p_prior)  # for setting unknown cells to -1
        gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
        #gridmap_int8[unknown_mask] = -1  # for setting unknown cells to -1

        # Publish map
        self.map_msg.data = gridmap_int8
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        rospy.loginfo_once("Published map!")

    def laserscan_callback(self, data):
        """Callback function for laserscan data."""
        if not self.is_gridmapping_initialized:
            print("min & max", data.angle_min, data.angle_max)
            angle = 135*np.pi/180
            self.init_gridmapping((-angle)+3*0.7853981634, (angle)+3*0.7853981634, data.angle_increment, data.range_max)

        print(self.robot_frame, self.map_frame)
        self.tf_sub.waitForTransform(self.map_frame, self.robot_frame, data.header.stamp, rospy.Duration(1.0))
        try:
            # get the robot position associated with the current laserscan
            (x, y, _),(qx, qy, qz, qw) = self.tf_sub.lookupTransform(self.map_frame, self.robot_frame, data.header.stamp)
            theta = self.quarternion_to_yaw(qx, qy, qz, qw)

            # check the movement if update is needed
            # if ( (x-self.prev_robot_x)**2 + (y-self.prev_robot_y)**2 >= self.update_movement**2 ):
            print(theta)
            # readingranges starts from index 89 and ends at index 449 inclusive
            readingranges = data.ranges[0:]
            gridmap = self.gridmapping.update(x, y, theta, readingranges).T.flatten() # update map
            # rospy.loginfo(x)
            # rospy.loginfo(y)
            self.prev_robot_x = x
            self.prev_robot_y = y

                # publish map (with the specified frequency)
                # if (self.map_last_publish.to_sec() + 1.0/self.map_publish_freq < rospy.Time.now().to_sec() ):
            self.map_last_publish = rospy.Time.now()
            self.publish_occupancygrid(gridmap, data.header.stamp)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)


gm = GridMappingROS()
while not rospy.is_shutdown():
    rospy.spin()