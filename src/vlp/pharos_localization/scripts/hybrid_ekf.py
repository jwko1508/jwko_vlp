#!/usr/bin/env python
import numpy as np
from math import *

import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavVELNED
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from pharos_msgs.msg import StateStamped2016 as VehicleState
from pharos_msgs.msg import MotionCommandStamped3
from pharos_msgs.msg import LaneOffset
from tf import transformations
from std_msgs.msg import Int32

import copy
import math
import time

###############################################################################
# def TicTocGenerator():
#     # Generator that returns time differences
#     ti = 0           # initial time
#     tf = time.time() # final time
#     while True:
#         ti = tf
#         tf = time.time()
#         yield tf-ti # returns the time difference

# TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# # This will be the main function through which we define both tic() and toc()
# def toc(tempBool=True):
#     # Prints the time difference yielded by generator instance TicToc
#     tempTimeInterval = next(TicToc)
#     if tempBool:
#         print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

# def tic():
#     # Records a time in TicToc, marks the beginning of a time interval
#     toc(False)
#################################################################################


roslib.load_manifest('pharos_localization')
def normalizeHeading(heading):
    # print "tuned x:", heading
    if heading > math.pi:
        heading -= 2*math.pi
    elif heading < -math.pi:
        heading += 2*math.pi
    # print "tuned o:", heading
    return heading

class RosMclEkf:
    """docstring for RosMclEkf"""
    hybrid_ekf_odom_pub = rospy.Publisher('/odom/ekf', Odometry)
    mcl_odom_pub = rospy.Publisher('/onlymcl', Odometry)
    lane_offset_pub = rospy.Publisher('/lane_offset', LaneOffset)

    relative_mcl = False
    publish_odom_tf = True

    frame_id = ''
    child_frame_id = ''


    def __init__(self):
        self.ekf_reset()

    def ekf_reset(self):
        print('enter reset')
        # EKF variables
        self.stop_index = True
        self.heading_init = False
        self.Q = np.mat(np.diag([0.1, 0.1, 0.001, 0.01]))
        self.Q_init = copy.deepcopy(self.Q)
        self.P = np.mat(np.diag([5, 5, 10, 0]))
        # self.P = np.mat(np.diag([0, 0, 0]))
        self.R = np.mat(np.diag([5.0, 5.0, 500, 100.0]))
        self.R_init = copy.deepcopy(self.R)

        # self.R = np.mat(np.diag([2, 2, 3]))

        self.F = np.mat(np.diag([1.0, 1.0, 1.0, 1.0]))
        self.H = np.mat(np.diag([1, 1, 1, 0]))
        self.X = np.zeros((4, 1))
        # self.old_X = np.mat('0.0; 0.0; 0.0; 0.0')

    # initialize variables
        self.x_meas, self.y_meas, self.theta_meas = [0.0] * 3
        # self.stop_x, self.stop_y, self.x_bias, self.y_bias = [0.0] * 4
        self.last_time = rospy.get_rostime()
        self.filter_initialized = False
        self.old_mcl = np.mat([0, 0, 0, 0]).T
        self.new_mcl = np.mat([0, 0, 0, 0]).T
        # self.particle_odom = [0.0, 0.0, 0.0]
        self.old_vel = 0.0
        self.old_alpha = 0.0
        self.back = False
        # self.restart = False
        self.L = 0
        # self.delta = 0
        self.inclination = 0
        # self.current_wp_index = 1000

        self.origin_mcl_x = 0.0
        self.origin_mcl_y = 0.0

        # self.lane_offset = 0.0
        # self.heading_difference = 0.0
        # self.lateral_offset = 0.0
        # self.curvature = 0.0

        self.covariance = np.zeros(16)

        self.steer_bias = 15.0
        self.azimuth = 0.0
        self.yaw_rate = 0.0

        self.Y = []
        self.time4bagplay = 0
        self.use_const_ratio = True
        self.stear_const = 15.14
        self.alpha_gain = 0.95
        self.vel_gain = 1.02
        self.cov_a = 0.0

        self.mcl_t0 = []

        print('init')

    def bicycle_model(self, x, y, theta, z, vel, alpha, dt):
        L = 2.7
        d = vel*self.vel_gain * dt
        beta = d / L * tan(alpha*self.alpha_gain)
        delta = self.inclination
        # gamma = d / L / cos(alpha)
        x = x + d * cos(theta)*cos(delta)
        y = y + d * sin(theta)*cos(delta)
        z = z + d * sin(delta)
        # theta = theta + self.yaw_rate * dt
        theta = theta + beta
        x_dot = - d * sin(theta) * cos(delta)
        y_dot = d * cos(theta) * cos(delta)

        return x, y, theta, z, x_dot, y_dot

    def vehicle_state_stack(self, new_vel, new_alpha, dt, crio_time):
        # print len(self.Y)
        self.Y.append([self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0], self.old_vel, self.old_alpha, dt, crio_time])
        # print crio_time

    def mcl_predict(self, velo_time):

        x, y, theta, z = [0.0]*4
        after_velo = 0
        Y=[]
        for i in range(1, len(self.Y), 1):
            if self.Y[i][7] > velo_time-rospy.Duration.from_sec(0.1):
                after_velo = i-1
                break
        for i in range(after_velo, len(self.Y)-1, 1):
            Y.append(self.Y[i])

        for i in range(0, len(Y)-1, 1):
            x, y, theta, z, null, null = self.bicycle_model(x,y,theta,z,Y[i][4],Y[i][5],Y[i][6])

        predict_time = Y[len(Y)-1][7] - Y[0][7]

        self.Y = Y
        if len(self.Y)>500:
            print "!!!!!stack overflow!!!!!"

        # print('step:',len(Y))
        print 'x:',x
        print 'pt:',predict_time/1000000000
        # print('theta:',theta)

        return [x,y,theta], predict_time


    def ekf_init(self, init_x, init_y, init_theta, init_z):
        self.X = np.mat([init_x, init_y, init_theta, init_z]).T

    def ekf_predict(self, new_vel, new_alpha, dt):
        x, y, theta, z, x_dot, y_dot = self.bicycle_model(self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0],
                                                       self.old_vel, self.old_alpha, dt)

    # model linearization
        self.F[0, 2] = x_dot
        self.F[1, 2] = y_dot

        self.P = self.F * self.P * self.F.T + self.Q
        K = self.P * self.H.T * (self.H * self.P * self.H.T + self.R).I

        if new_vel < 0.5:
            if abs(self.old_mcl[2,0] - self.new_mcl[2,0]) > 0.5:
                self.Q[2, 2] = 0
        if new_vel == 0 :
            self.Q[0,0] = 0; self.Q[1,1] = 0
        else:
            self.stop_index = True

        # if self.restart == True:
        #     self.R = np.mat(np.diag([1000.0, 1000.0, 1000.0, 1000.0]))
        #     if self.L >= 10:
        #         self.R = self.R_init
        #         self.restart = False

        self.P = (np.eye(4) - K * self.H) * self.P

        dL = new_vel * dt
        self.L += dL

        self.X = np.mat([x, y, theta, z]).T

        return K

    # MCL update
    def ekf_update(self, x_meas, y_meas, theta_meas, z_meas, K):
        if theta_meas > 0 and self.X[2,0] < 0 and theta_meas > 1.5:
            theta_meas-=math.pi*2
        elif theta_meas < 0 and self.X[2,0] > 0 and theta_meas <-1.5:
            theta_meas+=math.pi*2

        if self.back == True:
            if theta_meas < 0:
                theta_meas += math.pi
            elif theta_meas > 0:
                theta_meas -= math.pi

        Z = np.mat([x_meas, y_meas, theta_meas, z_meas]).T
        S = Z - self.X
        S[2, 0] = normalizeHeading(S[2,0])
        self.X = self.X + K * S
        self.X[2,0] = normalizeHeading(self.X[2,0])


    def gps_odom_callback(self, msg):
        if not self.filter_initialized:
            1


    def mcl_odom_callback(self, msg):

        self.covariance = msg.pose.covariance

        euler = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        if not self.mcl_t0 : self.mcl_t0 = msg.header.stamp
        if abs((msg.header.stamp - self.mcl_t0).to_sec()) > 10 :
            self.filter_initialized = False
            print "reinitialized"

        self.mcl_t0 = msg.header.stamp

        if not self.filter_initialized:
            self.ekf_reset()

            # We always calculate in relative coordinates, but then displace result to global frame
            self.origin_mcl_x = msg.pose.pose.position.x
            self.origin_mcl_y = msg.pose.pose.position.y
            self.origin_altitude = msg.pose.pose.position.z
            # first calculation of relative pos measurement
            new_mcl = np.mat([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              euler[2],
                              msg.pose.pose.position.z]).T

            self.ekf_init(new_mcl[0,0], new_mcl[1,0], new_mcl[2,0], new_mcl[3,0])

            if self.relative_mcl:
                print 'origin'
                rospy.set_param('~origin/x', self.origin_mcl_x)
                rospy.set_param('~origin/y', self.origin_mcl_y)
                rospy.loginfo("Using relative MCL coordinates. Origin stored in ~/origin")

            self.filter_initialized = True
            self.last_time = rospy.get_rostime()
        else:
            new_mcl = np.mat([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              euler[2],
                              msg.pose.pose.position.z]).T

            predict_vect, predict_time = self.mcl_predict(msg.header.stamp)

            predicted_stamp = msg.header.stamp + predict_time

            x = predict_vect[0]
            y = predict_vect[1]
            new_mcl[0] += sqrt(x*x+y*y)*cos(atan2(y, x) + new_mcl[2])
            new_mcl[1] += sqrt(x*x+y*y)*sin(atan2(y, x) + new_mcl[2])
            new_mcl[2] += predict_vect[2]


            covx=self.covariance[0]/0.05
            covy=self.covariance[7]/0.05
            covt=self.covariance[35]/0.0005
            self.Q[0,0]=self.Q_init[0,0]*(1-self.cov_a)+self.Q_init[0,0]*self.cov_a/covx
            self.Q[1,1]=self.Q_init[1,1]*(1-self.cov_a)+self.Q_init[1,1]*self.cov_a/covy
            self.Q[2,2]=self.Q_init[2,2]*(1-self.cov_a)#+self.Q_init[2,2]*self.cov_a/covt
            self.R[0,0]=self.R_init[0,0]*(1-self.cov_a)+self.R_init[0,0]*self.cov_a*covx
            self.R[1,1]=self.R_init[1,1]*(1-self.cov_a)+self.R_init[1,1]*self.cov_a*covy
            self.R[2,2]=self.R_init[2,2]*(1-self.cov_a)#+self.R_init[2,2]*self.cov_a*covt
            print("Q", "%.4f"%self.Q[0,0],"%.4f"%self.Q[2,2])
            print("R", "%.4f"%self.R[0,0],"%.2f"%self.R[2,2])

            if self.time_state:
                dt = (self.time4bagplay-self.last_time).to_sec()
            else:
                dt = (rospy.Time.now()-self.last_time).to_sec()

            K = self.ekf_predict(self.old_vel, self.old_alpha, dt)
            # print 'mcl_dt : ', dt

                #############################################################################################################################
            if abs(self.old_mcl[2,0] - self.new_mcl[2,0])< 0.5:# and self.covariance[0] < 15:
                if not self.old_vel == 0:
                    self.ekf_update(new_mcl[0,0], new_mcl[1,0], new_mcl[2,0], new_mcl[3,0], K)
                #############################################################################################################################
            # print self.time_state

            if self.time_state:
                self.last_time = self.time4bagplay
            else:
                self.last_time = rospy.Time.now()

        self.publish_odom()
        # self.old_X = self.X
        self.old_mcl = self.new_mcl
        self.new_mcl = new_mcl

    def vehicle_state_callback(self, msg):
        self.time4bagplay = msg.header.stamp

        steering_bias = 0.0
        steer_angle = msg.state.wheel_angle * self.stear_const
        self.gear = msg.state.gear
        if not self.filter_initialized:
            return

        if msg.state.gear == 2 and abs(self.new_mcl[2, 0] - self.old_mcl[2, 0]) > 2.0:
            self.back = True
        elif msg.state.gear == 4 and abs(self.new_mcl[2, 0] - self.old_mcl[2, 0]) > 2.0:
            self.back = False
        p1 = -1.184*pow(10,-5)
        p2 = 0.0003041
        p3 = -0.002027
        p4 = -0.01141
        p5 = 0.2303
        p6 = -1.303
        p7 = 3.497
        p8 = -4.567
        p9 = 17.4

        steer_angle = abs(steer_angle)
        if steer_angle >= math.pi / 3:
            self.ratio = p1 * pow(steer_angle, 8) + p2 * pow(steer_angle, 7) + p3 * pow(steer_angle, 6) + p4 * pow(
                steer_angle, 5) + p5 * pow(steer_angle, 4) + p6 * pow(steer_angle, 3) + p7 * pow(steer_angle, 2) + p8 * steer_angle + p9
        else:
            self.ratio = (self.stear_const - self.steer_bias) * 3 / math.pi * steer_angle + self.steer_bias

        if self.use_const_ratio:
            new_alpha = msg.state.wheel_angle
        else:
            new_alpha = msg.state.wheel_angle*self.stear_const / self.ratio  # + steering_bias

        new_vel = msg.state.velocity
        self.vel = new_vel

        dt = (msg.header.stamp - self.last_time).to_sec()
        # print "veh_dt : ", dt

        # R meas
        if self.heading_init == True:
            if abs(new_vel) <= 0.5:
                self.R = np.mat(np.diag([-513.8 * new_vel * new_vel + 1000, -513.8 * new_vel * new_vel + 1000,
                                         -508.6 * new_vel * new_vel + 1000, 100]))
                self.Q = np.mat(np.diag([0.0, 0.0, 0.0, 0.0]))
                # print 'low speed'
            else:
                self.R = copy.deepcopy(self.R_init)
                self.Q = copy.deepcopy(self.Q_init)
            # for covariance
        covariance_x = self.covariance[0];
        covariance_y = self.covariance[7];
        covariance_theta = self.covariance[14]
        if covariance_x > 8 or covariance_y > 8:
            self.Q[0,0] = 0.2; self.Q[1,1] = 0.2; self.Q[2,2] = 0.01
            self.R[0,0] = pow(covariance_x-8,2)+15.0
            self.R[1,1] = pow(covariance_y-8,2)+15.0
            self.R[2,2] = pow(covariance_theta-5,2)+10.0
        else:
            self.Q = copy.deepcopy(self.Q_init)           

        # Heading initializing
        # if self.current_wp_index < 570:
        #     self.heading_init = True
        if self.heading_init == False:
            if new_vel < 1.388:
                self.Q[2, 2] = 100
            else:
                self.heading_init = True
        else:
            self.Q = copy.deepcopy(self.Q_init)


        K = self.ekf_predict(new_vel, new_alpha, dt)
        self.vehicle_state_stack(new_vel, new_alpha, dt, msg.header.stamp)


        self.publish_odom()

        # self.publish_lane_offset()
        # self.old_X = self.X
        self.old_vel = new_vel
        self.old_alpha = new_alpha
        self.last_time = msg.header.stamp

    # def motion_command_callback(self, msg):
    #     self.inclination = msg.inclination
    #     self.heading_difference = msg.heading_difference
    #     self.lateral_offset = msg.lateral_offset
    #     self.curvature = msg.curvature

    # def current_wp_index_callback(self, msg):
    #     self.current_wp_index = msg.data

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.last_time
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        ekf_heading = self.X[2,0]
        if self.relative_mcl:
            msg.pose.pose.position = Point(self.X[0,0], self.X[1,0], self.X[3,0])
        else:
            msg.pose.pose.position = Point(self.X[0,0] + self.origin_mcl_x, self.X[1,0] + self.origin_mcl_y, 0.0)

        msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, -self.inclination, ekf_heading).GetQuaternion()))
        p_cov = np.array([0.0] * 36).reshape(6, 6)
    # position covariance
        p_cov[0:2, 0:2] = self.P[0:2, 0:2]
    # x and Yaw
        p_cov[5, 0] = p_cov[0, 5] = self.P[2, 0]
    # y and Yaw
        p_cov[5, 1] = p_cov[1, 5] = self.P[2, 1]
    # Yaw and Yaw
        p_cov[5, 5] = self.P[2, 2]

        msg.pose.covariance = tuple(p_cov.ravel().tolist())
        # pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        # ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.hybrid_ekf_odom_pub.publish(msg)


    # def publish_lane_offset(self):
    #     msg = LaneOffset()
    #     msg.lane_offset = self.lane_offset
    #     msg.steering_ratio = self.ratio
    #     self.lane_offset_pub.publish(msg)

    # def publish(self, publish_rate):
    #     loop_rate = rospy.Rate(publish_rate)

    #     while not rospy.is_shutdown():
    #         if not self.filter_initialized:
    #             loop_rate.sleep()
    #             continue

    #         self.publish_odom()
    #         try:
    #             loop_rate.sleep()
    #         except rospy.ROSException, e:
    #             if e.message == 'ROS time moved backwards':
    #                 rospy.logwarn("Saw a negative time change, resetting MCL EKF.")
    #                 self.filter_initialized = False


if __name__ == '__main__':
    rospy.init_node('hybrid_ekf_node')

    hybrid_ekf = RosMclEkf()


    hybrid_ekf.time_state = rospy.get_param('~isbagfile', False)
    hybrid_ekf.frame_id = rospy.get_param('~frame_id', '/odom')
    hybrid_ekf.child_frame_id = rospy.get_param('~child_frame_id', '/hybrid_ekf')
    gps_odom = rospy.get_param('~gps_odom_topic', '/odom/ublox')
    mcl_odom = rospy.get_param('~mcl_odom_topic', '/odom/mcl')
    vehicle_state = rospy.get_param('~vehicle_state_topic', '/vehicle/state2016')

    # Set MCL position relative to first measurement, i.e. first measurement will be at (0,0)
    hybrid_ekf.relative_mcl = rospy.get_param('~relative_mcl', True)
    # hybrid_ekf.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)

    rospy.Subscriber(gps_odom, Odometry , hybrid_ekf.gps_odom_callback, queue_size=1)
    rospy.Subscriber(mcl_odom, Odometry , hybrid_ekf.mcl_odom_callback, queue_size=1)
    rospy.Subscriber(vehicle_state, VehicleState, hybrid_ekf.vehicle_state_callback, queue_size=1)
    # rospy.Subscriber('/vehicle/motion_command', MotionCommandStamped3, hybrid_ekf.motion_command_callback, queue_size=1)

    rospy.spin()