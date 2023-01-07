#!/usr/bin/env python3

from glob import glob
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import random
from math import pi, sin, cos, degrees
from skimage.draw import line
import cv2 as cv
from nav_msgs.msg import OccupancyGrid
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection
import message_filters
import traceback

BGR_RED_COLOR = (0, 0, 255)
BGR_BLUE_COLOR = (255, 0, 0)
BGR_GREEN_COLOR = (0, 255, 0)

# GRAYSCALE_BLACK_COLOR = 0
# GRAYSCALE_WHITE_COLOR = 255

# CM_IN_METER = 100


class ScanObj:
    def __init__(self, m, n, i, rn, rm):
        self.angle_max = m
        self.angle_min = n
        self.angle_increment = i
        self.range_min = rn
        self.range_max = rm


class RayCaster:
    def __init__(self, map, angle_range=360, angle_accuracy=0.5,
                 length_range=3, pixel_size=0.1, occ_th=0.9, offset_x=125, offset_y=125):
        """
        Description: constructor for the ray caster.

        Input:
          - map: 2d numpy array of the map where the ray will be casted.
          - angle_range: the angle range to be scanned in degrees 
          from (1 to 360). Half of this range is for the right
          scan and the second half is for the left part.
          - angle_accuracy: An integer representing the step to
          increment the angle with in degrees.
          - length_range: the maximum length for the ray to be scanned
          in meters.
          - pixel_size: the pixel size in real world in centimeters.
          - occ_th: the threshold for the cell to be considered occupied.
        """
        self.angle_range = angle_range
        self.angle_accuracy = angle_accuracy
        self.length_range = length_range
        self.pixel_size = pixel_size
        self.map = map
        self.occ_th = occ_th
        self.offset_x = offset_x
        self.offset_y = offset_y

        # reuse
        self.angles = None
        self.cos_vec = None
        self.sin_vec = None
        self.dst_vec = None
        self.X_org = None
        self.Y_org = None
        self.measurements = None

    def _is_collided(self, p):
        """
        Description: check if (a) certain point(s) is/are collided with
        an obstacle in the map.

        Input:
          - p: the onject to be checked (x, y).

        Output: 
          - True if collided, False otherwise
        """
        X, Y = p

        return self.map[Y, X] > self.occ_th

    def _calculate_dist(self, p1, p2):
        """
        Description: calculate the euclidean distance.

        Input:
          - p1
          - p2

        output:
          - distance
        """
        dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return dist

    def cast(self, pose, scan, show_rays=False):
        """
        Description: Cast a ray.

        Input:
          - pose: robot pose (x, y, theta).
          - scan: LaserScan msg
          - show_rays: if True, show the image with the rays shown.
          shown.

        Output:
          - measurements: numpy array for the distances in pixels.
          The ray that's not collided has a distance of -1
        """

        global world_points

        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = np.degrees(scan.angle_increment)
        stepsCount = int((angle_max - angle_min) / angle_increment)
        rangeLength = len(scan.ranges)

        size = 680

        # rospy.loginfo("cast")
        x, y, theta = pose
        x = int(x / self.pixel_size) + self.offset_x
        y = int(y / self.pixel_size) + self.offset_y
        ranges = np.array(scan.ranges)[:680] / self.pixel_size

        if self.X_org is None:
            angle_min = scan.angle_min
            angle_max = scan.angle_max
            angle_increment = np.degrees(scan.angle_increment)

            start_angle = np.degrees(angle_min)
            end_angle = np.degrees(angle_max)
            start_len = int(scan.range_min / self.pixel_size)
            end_len = int(scan.range_max / self.pixel_size)

            # size = int((end_angle-start_angle)//angle_increment)
            self.measurements = np.ones(680) * -1

            self.angles = np.arange(
                start_angle, end_angle, angle_increment)[:680]
            self.cos_vec = np.cos(np.radians(self.angles)).reshape(-1, 1)
            self.sin_vec = np.sin(np.radians(self.angles)).reshape(-1, 1)
            self.dst_vec = np.arange(
                start_len, end_len, 1).reshape(1, -1)[:680]
            self.X_org = np.matmul(self.cos_vec, self.dst_vec).astype(np.int32)
            self.Y_org = np.matmul(self.sin_vec, self.dst_vec).astype(np.int32)

            # ranges = ranges[:size]

        X = x + self.X_org * np.cos(theta) - self.Y_org * np.sin(theta)
        Y = y + self.X_org * np.sin(theta) + self.Y_org * np.cos(theta)
        X = X.astype(np.int32)
        Y = Y.astype(np.int32)

        # print(len(X))

        # worldPoints = np.rollaxis(np.array(world_points).reshape(-1, 3), 1)
        # X = np.round(worldPoints[0].astype(np.int32)[:680]/ 0.1).astype(np.int32) + self.offset_x
        # Y = np.round(worldPoints[1].astype(np.int32)[:680]/ 0.1).astype(np.int32) + self.offset_y

        # print(len(X))

        X_Y = (X, Y)

        X[X < 0] = 0
        X[X >= self.map.shape[1]] = self.map.shape[1]-1
        Y[Y < 0] = 0
        Y[Y >= self.map.shape[0]] = self.map.shape[0]-1

        collision = self._is_collided(X_Y)
        rospy.loginfo(len(collision))
        measurements = self.measurements.copy()
        rospy.loginfo(len(measurements))
        dst_idx = np.argmax(collision, axis=-1)
        mask_collided = np.any(collision, axis=-1)
        rospy.loginfo(mask_collided)
        dst = self.dst_vec.flatten()[dst_idx]
        rospy.loginfo(len(dst))
        measurements[mask_collided] = dst[mask_collided]

        if show_rays:
            img = self.map.copy()
            img[img < self.occ_th] = 0
            img[img > self.occ_th] = 255

            img = cv.cvtColor(img.astype(np.uint8), cv.COLOR_GRAY2RGB)
            # img[Y, X] = BGR_RED_COLOR
            n = np.arange(X.shape[0])
            X = X[n, dst_idx]
            Y = Y[n, dst_idx]
            # img[Y, X] = BGR_BLUE_COLOR
            X = X - 1
            Y = Y - 1
            # img[Y, X] = BGR_BLUE_COLOR

            # show robot
            cv.circle(img, (x, y), 15, BGR_GREEN_COLOR, 2)
            pt1 = (int(x), int(y))
            pt2 = (int(x + 15 * np.cos(theta)), int(y + 15 * np.sin(theta)))
            cv.line(img, pt1, pt2, BGR_GREEN_COLOR, 2)

            rot = int(np.round(np.degrees(-theta)))
            scan_epsx = ranges * np.roll(self.cos_vec.reshape(-1), rot)
            scan_epsy = ranges * np.roll(self.sin_vec.reshape(-1), rot)

            scan_epsx = x + scan_epsx
            scan_epsy = y + scan_epsy
            scan_epsx = scan_epsx.astype(np.int32)
            scan_epsy = scan_epsy.astype(np.int32)

            scan_epsx[scan_epsx < 0] = 0
            scan_epsx[scan_epsx >= self.map.shape[1]] = self.map.shape[1]-1
            scan_epsy[scan_epsy < 0] = 0
            scan_epsy[scan_epsy >= self.map.shape[0]] = self.map.shape[0]-1

            # for i, j in zip(scan_epsx, scan_epsy):
            #   cv.circle(img, (i, j), 2, BGR_GREEN_COLOR, 5)

            img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
            img = cv.flip(img, 0)
            cv.imshow("Ray Casting", img)

        return measurements, mask_collided, (scan_epsx, scan_epsy)


class Particle(object):
    """Particle for tracking a robot with a particle filter.

    The particle consists of:
    - a robot pose
    - a weight
    - a map consisting of landmarks
    """

    def __init__(self, num_particles, noise, pixel_size=0.1,
                 offset_x=125, offset_y=125, map_h=2500, map_w=2500):
        """Creates the particle and initializes location/orientation"""
        self.noise = noise

        # initialize robot pose at origin
        self.pose = np.vstack([0, 0, 0])

        # initialize weights uniformly
        self.weight = 1.0 / float(num_particles)

        # Trajectory of the particle
        self.trajectory = []

        # initialize the grid map
        self.prior = 0.5
        self.free_lo = .4
        self.occ_lo = .9
        self.offset_x = offset_x
        self.offset_y = offset_y

        self.map = np.ones((map_h, map_w)) * self.prior
        self.trajectory_map = np.zeros((map_h, map_w), dtype=np.uint8)

        # sensor model
        self.ray_caster = RayCaster(self.map, pixel_size=pixel_size,
                                    offset_x=offset_x, offset_y=offset_y)
        self.pixel_size = pixel_size
        self.laser_eps = []

    def motion_update(self, odom):
        """Predict the new pose of the robot"""

        # rospy.loginfo('motion update')

        # append the old position
        self.trajectory.append(self.pose)

        # Sample odometry motion model - slide 27
        # @ http://ais.informatik.uni-freiburg.de/teaching/ss15/robotics/slides/06-motion-models.pdf

        # noise sigma for delta_rot1
        delta_rot1_noisy = random.gauss(odom.r1, self.noise[0])

        # noise sigma for translation
        translation_noisy = random.gauss(odom.tr, self.noise[1])

        # noise sigma for delta_rot2
        delta_rot2_noisy = random.gauss(odom.r2, self.noise[2])

        # Estimate of the new position of the Particle
        x_new = self.pose[0] + translation_noisy * \
            cos(self.pose[2] + delta_rot1_noisy)
        y_new = self.pose[1] + translation_noisy * \
            sin(self.pose[2] + delta_rot1_noisy)
        theta_new = normalize_angle(
            self.pose[2] + delta_rot1_noisy + delta_rot2_noisy)

        self.pose = np.array([x_new, y_new, theta_new])

        y_new = int(y_new/self.pixel_size + self.offset_x)
        x_new = int(x_new/self.pixel_size + self.offset_y)

    def sensor_update(self, scan):
        """
        Description: Weight the particles according to the current map
         of the particle and the scan observations z.

        Input:
            - scan : LaserMsg
        """

        # rospy.loginfo('sensor update')
        # beam based sensor model - slide 5
        # @ http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/07-sensor-models.pdf

        est, mask_collided, laser_eps = self.ray_caster.cast(self.pose, scan,
                                                             show_rays=True)
        ranges = np.array(scan.ranges)[:680] / self.pixel_size

        error = np.sum(np.abs(est[mask_collided] - ranges[mask_collided]))

        self.weight = 1 / (1 + error)

        self.laser_eps = laser_eps

    def _inv_sensor_model(self, pf):
        """
        Description: Inverse Sensor Model for
        Sonars Range Sensors

        Inverse Sensor Model for Sonars Range Sensors - slide 26
        @ http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam11-gridmaps.pdf

        Input:
            - pf: perceptual field

        Output:
            - map: map with logodds.
        """

        # rospy.loginfo('inv sensor model')
        result_map = np.zeros(np.shape(self.map))

        pf_x, pf_y = pf
        eps_x, eps_y = self.laser_eps

        # subsitute the rasters with free logodds on the result_map
        result_map[pf_y, pf_x] = self.free_lo
        # subsitute the laser_eps with occupied logodds
        result_map[eps_y, eps_x] = self.occ_lo

        return result_map

    def _get_perceptual_field(self, pose):
        """
        Description: get the perceptual field of a scan

        Input:
            - pose: the robot pose (x, y, theta)
            - laser_eps: laser endpoints

        Output:
            - X: the x coordiantes of the field cells
            - Y: the y coordinates of the field cells
        """
        # rospy.loginfo('get perceptual field')
        X = []
        Y = []

        x, y, theta = pose
        x = int(x / self.pixel_size) + self.offset_x
        y = int(y / self.pixel_size) + self.offset_y

        # get the rasters from the robot_pose to the laser_eps
        eps_x, eps_y = self.laser_eps
        for x2, y2 in zip(eps_x, eps_y):
            raster_x, raster_y = line(x, y, x2, y2)
            X.extend(raster_x)
            Y.extend(raster_y)

        return X, Y

    def map_update(self):
        """
        Description: update the map based on the occupancy grid 
        mapping algorithm.

        Occupancy Mapping Algorithm - slide 24 
        @ http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam11-gridmaps.pdf 

        Input: 
            - laser_eps: laser end points

        Output:
            - map: the updated map
        """
        pf = self._get_perceptual_field(self.pose)
        invmod = self._inv_sensor_model(pf)
        pf_x, pf_y = pf
        self.map[pf_y, pf_x] = self.map[pf_y, pf_x] + \
            invmod[pf_y, pf_x] - self.prior
        # rospy.loginfo(self.map)


class OdometryData(object):
    """Represents an odometry command.

    - r1: initial rotation in radians counterclockwise
    - tr: translation in meters
    - r2: final rotation in radians counterclockwise
    """

    def __init__(self, r1, tr, r2):
        self.r1 = self.normalize_angle(r1)
        self.tr = tr
        self.r2 = self.normalize_angle(r2)

    def normalize_angle(self, angle):
        """Normalize the angle between -pi and pi"""

        while angle > np.pi:
            angle = angle - 2 * np.pi

        while angle < -np.pi:
            angle = angle + 2 * np.pi

        return angle

    def __str__(self):
        # return "Odometry(r1 = {0} rad,\ntr = {1} m\n, r2 = {2} rad\n)".format(self.r1, self.tr, self.r2)
        return "Odometry(r1 = {0} deg,\ntr = {1} m\n, r2 = {2} deg\n)".format(np.degrees(self.r1), self.tr, np.degrees(self.r2))


class FastSlam:
    def __init__(self, particles):
        self.particles = particles

    def fast_slam(self, odom, sensor):
        '''Executes one iteration of the prediction-correction-resampling loop of FastSLAM.

        Returns the plot objects to be drawn for the current frame.
        '''

        # rospy.loginfo('fast slam')

        for particle in self.particles:
            particle.motion_update(odom)
            particle.sensor_update(sensor)
            particle.map_update()

        # Resample the particle set
        # Use the "number of effective particles" approach to resample only when
        # necessary. This approach reduces the risk of particle depletion.
        # For details, see Section IV.B of
        # http://www2.informatik.uni-freiburg.de/~burgard/postscripts/grisetti05icra.pdf
        s = sum([particle.weight for particle in self.particles])
        neff = 1. / sum([(particle.weight/s) **
                        2 for particle in self.particles])
        if neff < len(self.particles) / 2.:
            # rospy.loginfo("resample")
            self.particles = self.resample(self.particles)

    def resample(self, particles):
        """Resample the set of particles.

        A particle has a probability proportional to its weight to get
        selected. A good option for such a resampling method is the so-called low
        variance sampling, Probabilistic Robotics page 109"""

        # rospy.loginfo('resample')
        num_particles = len(particles)
        new_particles = []
        weights = [particle.weight for particle in particles]

        # normalize the weight
        sum_weights = sum(weights)
        weights = [weight / sum_weights for weight in weights]

        # the cumulative sum
        cumulative_weights = np.cumsum(weights)
        normalized_weights_sum = cumulative_weights[len(
            cumulative_weights) - 1]

        # check: the normalized weights sum should be 1 now (up to float representation errors)
        assert abs(normalized_weights_sum - 1.0) < 1e-5

        # initialize the step and the current position on the roulette wheel
        step = normalized_weights_sum / num_particles
        position = random.uniform(0, normalized_weights_sum)
        idx = 1

        # walk along the wheel to select the particles
        for i in range(1, num_particles + 1):
            position += step
            if position > normalized_weights_sum:
                position -= normalized_weights_sum
                idx = 1
            while position > cumulative_weights[idx - 1]:
                idx = idx + 1

            new_particles.append(copy.deepcopy(particles[idx - 1]))
            new_particles[i - 1].weight = 1 / num_particles

        return new_particles

    def get_map(self):
        """Returns the map of the best particle."""

        # rospy.loginfo('get map')
        best_particle = max(self.particles, key=lambda p: p.weight)
        return best_particle.map


def normalize_angle(angle):
    # rospy.loginfo('normalize angle')
    """Normalize the angle between -pi and pi"""

    while angle > pi:
        angle = angle - 2 * pi

    while angle < -pi:
        angle = angle + 2 * pi

    return angle


def ms_to_sec(x): return x * (10**-3)
def ms_to_ns(x): return x * (10**6)
def s_to_ns(x): return x * (10**9)


INTERVAL_IN_MS = 1000
INTERVAL_IN_S = ms_to_sec(INTERVAL_IN_MS)
INTERVAL_IN_NS = ms_to_ns(INTERVAL_IN_MS)

FREQ = 1/INTERVAL_IN_S


t1 = 0  # t
pt1 = np.zeros(3)  # odom at t-1
pt2 = np.zeros(3)  # odom at t
r1 = 0  # initial rotation in radians counterclockwise
tr = 0  # translation
r2 = 0  # initial rotation in radians counterclockwise

gscan = False
godom = False

read_first_odom = False

stamp = LaserScan().header.stamp


def lidar_callback(scan):
    # rospy.loginfo("lidar callback")
    global gscan
    global stamp
    global godom

    world_points = get_transform_cloud(scan, godom)

    # calculate the range of the lidar
    ranges = []
    for i in range(len(world_points)):
        ranges.append((world_points[i][0]**2 + world_points[i][1]**2)**0.5)

    scan.ranges = ranges

    gscan = scan
    stamp = scan.header.stamp


def odom_callback(odom):
    # rospy.loginfo("odom callback")
    global t1
    global pt1
    global pt2
    global r1
    global tr
    global r2
    global godom
    global gscan
    global read_first_odom
    global fs

    t2 = int(str(odom.header.stamp))

    if t2 - t1 >= INTERVAL_IN_NS:
        # advance time
        t1 = t2

        # advance readings
        pt1[:] = pt2[:]

        # get new readings
        pt2[0] = odom.pose.pose.position.x
        pt2[1] = odom.pose.pose.position.y
        orientation_q = odom.pose.pose.orientation
        # new theta
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, theta) = euler_from_quaternion(orientation_list)
        pt2[2] = theta
        # only executed when there is a previous reading
        # not in the first callback
        if read_first_odom:
            # Odometry model - slide 10
            # @ http://ais.informatik.uni-freiburg.de/teaching/ss15/robotics/slides/06-motion-models.pdf

            # calculate s1, t, s2
            # translation
            tr = np.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)
            ydiff = np.around(pt2[1]-pt1[1], 2)
            xdiff = np.around(pt2[0]-pt1[0], 2)
            # initial rotation
            r1 = np.arctan2(ydiff, xdiff) - pt1[2]
            # final rotation
            r2 = pt2[2] - pt1[2] - r1
            godom = OdometryData(r1, tr, r2)
            # rospy.loginfo(godom)
            # rospy.loginfo('\n')

            # rospy.loginfo(godom)
        if godom and gscan:
            fs.fast_slam(godom, gscan)
            godom = False  # None
            gscan = False  # None
            cv.waitKey(1)

        read_first_odom = True


def get_transform_cloud(source_frame, target_frame):
    """Get the transform from the robot frame to the laser frame."""
    # get the transform from the robot frame to the laser frame
    cloud = LaserProjection().projectLaser(source_frame)
    transform = tf_buffer.lookup_transform(
        'robot_odom', source_frame.header.frame_id.replace("/", ""), rospy.Time())
    pcd2_laser = do_transform_cloud(cloud, transform)  # returns PointCloud2
    world_points = list(point_cloud2.read_points(
        pcd2_laser, field_names=("x", "y", "z"), skip_nans=False))
    # rospy.loginfo(world_points)
    return world_points


tf_buffer = None
tf_listener = None


lastLaserScan = None
world_points = None


def callback_handler(scan,  odom):
    # if high odometry variance, ignore
    # if odom.twist.covariance[0] > 0.1:
    #     return
    try:
        # rospy.loginfo("lidar callback")
        global gscan
        global stamp
        global godom
        global world_points

        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = np.degrees(scan.angle_increment)

        # if len(scan.ranges) < 701 or int((angle_max - angle_min) / angle_increment) < 701:
        #     return

        world_points = get_transform_cloud(scan, odom)

        # calculate the range of the lidar
        ranges = []
        for i in range(len(world_points)):
            ranges.append((world_points[i][0]**2 + world_points[i][1]**2)**0.5)

        print(ranges[0:4])

        scan.ranges = ranges

        gscan = scan
        stamp = scan.header.stamp

        global t1
        global pt1
        global pt2
        global r1
        global tr
        global r2
        global read_first_odom
        global fs
        global lastLaserScan

        t2 = int(str(odom.header.stamp))

        if t2 - t1 >= INTERVAL_IN_NS:
            # advance time
            t1 = t2

            # advance readings
            pt1[:] = pt2[:]

            # get new readings
            pt2[0] = odom.pose.pose.position.x
            pt2[1] = odom.pose.pose.position.y
            orientation_q = odom.pose.pose.orientation
            # new theta
            orientation_list = [orientation_q.x,
                                orientation_q.y, orientation_q.z, orientation_q.w]
            (_, _, theta) = euler_from_quaternion(orientation_list)
            pt2[2] = theta
            # only executed when there is a previous reading
            # not in the first callback
            if read_first_odom:
                # Odometry model - slide 10
                # @ http://ais.informatik.uni-freiburg.de/teaching/ss15/robotics/slides/06-motion-models.pdf

                # calculate s1, t, s2
                # translation
                tr = np.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)
                ydiff = np.around(pt2[1]-pt1[1], 2)
                xdiff = np.around(pt2[0]-pt1[0], 2)
                # initial rotation
                r1 = np.arctan2(ydiff, xdiff) - pt1[2]
                # final rotation
                r2 = pt2[2] - pt1[2] - r1
                godom = OdometryData(r1, tr, r2)
                # rospy.loginfo(godom)
                # rospy.loginfo('\n')

                # rospy.loginfo(godom)
            if godom and gscan:
                fs.fast_slam(godom, gscan)
                godom = False  # None
                gscan = False  # None
                if lastLaserScan == None or lastLaserScan.header.stamp != scan.header.stamp:
                    fsmap = fs.get_map() * 100
                    mapObject = OccupancyGrid()
                    mapObject.header.frame_id = "robot_map"
                    mapObject.info.width = 2500
                    mapObject.info.height = 2500
                    mapObject.info.resolution = 0.1
                    mapObject.info.origin.position.x = (-2500/2) * 0.1
                    mapObject.info.origin.position.y = (-2500/2) * 0.1
                    mapObject.info.origin.position.z = 0
                    # mapObject.info.origin.orientation.x = 0
                    # mapObject.info.origin.orientation.y = 0
                    # mapObject.info.origin.orientation.z = -quarternion_to_yaw(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
                    # mapObject.info.origin.orientation.w = 1
                    mapObject.data = (fsmap).astype(
                        dtype=np.int8).flatten().tolist()
                    mapObject.header.stamp = stamp
                    pub.publish(mapObject)
                    lastLaserScan = scan
                # cv.waitKey(1)

            read_first_odom = True
    except Exception as e:
        # print(e.with_traceback())
        print(traceback.format_exc())
        print(e)


pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)

fs = None


def main():
    # rospy.loginfo('Starting slam')
    '''
    Description: the entry point for the program.
    '''
    global gscan
    global godom
    global fs
    global stamp
    global tf_buffer
    global tf_listener

    rospy.init_node('slam', anonymous=True)
    rospy.loginfo('slam node started')

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    # initial timestamp
    t1 = int(str(rospy.Time.now()))  # t

    front_laser_sub = message_filters.Subscriber("/scan_multi", LaserScan)
    odom_sub = message_filters.Subscriber("/robot/robotnik_base_control/odom",
                                          Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([front_laser_sub, odom_sub],
                                                     10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_handler)

    # # sensors and map subscribtion
    # rospy.Subscriber("/scan_multi", LaserScan, lidar_callback)
    # rospy.Subscriber("/robot/robotnik_base_control/odom",
    #                  Odometry, odom_callback)

    # pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)

    # how many particles
    num_particles = 5
    noise = [0.001, 0.001, 0.000]
    particles = [Particle(num_particles, noise, pixel_size=0.1,
                          offset_x=1250, offset_y=1250, map_h=2500, map_w=2500) for _ in range(num_particles)]

    # set the axis dimensions
    fs = FastSlam(particles)

    rate = rospy.Rate(FREQ)  # rate of the loop
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
