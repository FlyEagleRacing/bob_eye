import os
import rclpy
from rclpy.node import Node
import numpy as np
from vectornav_msgs.msg import RtkObserver, ImuGroup
from a2rl_bs_msgs.msg import VectornavIns
from eav24_bsu_msgs.msg import Kistler_Vel_Angle
from std_msgs.msg import Float32, Int32
import math
from numpy.linalg import inv

class ExtendedKalmanFilter:
    def __init__(self):
        # State vector [x, y, theta, v]
        self.x = np.matrix([[0.], [0.], [0.], [0.]]) 
        # State covariance matrix
        self.P = np.diag([1.0, 1.0, 0.1, 1.0])
        # Process noise covariance matrix
        self.Q = np.diag([0.1, 0.1, 0.01, 0.1])
        # Measurement matrix
        self.H = np.matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # Measurement noise covariance
        self.R = np.matrix([
            [0.5, 0, 0, 0],   # Variance in x measurement
            [0, 0.5, 0, 0],   # Variance in y measurement
            [0, 0, 0.1, 0],   # Variance in theta measurement
            [0, 0, 0, 0.1]    # Variance in v measurement
        ])
        self.last_timestamp = None
        
    def predict(self, v_control, yaw, current_time):
        if self.last_timestamp is None:
            self.last_timestamp = current_time
            return
        dt = (current_time - self.last_timestamp).nanoseconds / 1e9
        self.last_timestamp = current_time
        x, y, theta, v = self.x[0,0], self.x[1,0], self.x[2,0], self.x[3,0]
        
        # State transition model
        f = np.matrix([
            [x + v_control * np.cos(theta) * dt],
            [y + v_control * np.sin(theta) * dt],
            [yaw],
            [v_control]
        ])
        
        # Jacobian of the state transition model
        F_j = np.matrix([
            [1, 0, -v_control * np.sin(theta) * dt, np.cos(theta) * dt],
            [0, 1, v_control * np.cos(theta) * dt, np.sin(theta) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Predict state
        self.x = f
        # Predict state covariance
        self.P = F_j * self.P * F_j.T + self.Q
    
    def update(self, z):
        # Measurement residual
        y = z - self.H * self.x
        
        # Residual covariance
        S = self.H * self.P * self.H.T + self.R
        
        # Kalman gain
        K = self.P * self.H.T * inv(S)
        
        # Update state estimate
        self.x = self.x + K * y
        
        # Update covariance estimate
        size = self.P.shape[0]
        I = np.eye(size)
        self.P = (I - K * self.H) * self.P

    def get_state(self):
        return self.x.tolist()
    
    def get_covariance_diagonal(self):
        return np.diag(self.P).tolist()

class BobEyeFilter(Node):
    def __init__(self):
        super().__init__('bob_eye_filter')
        # if orientation_stddev < VN_ORI_STDDEV_TH, we use vn_ins orientation, otherwise integral by angular rate
        self.VN_ORI_STDDEV_TH = 0.01
        # max delta_time to integral, if exceed this value, will fail
        self.MAX_DT_TH = 0.3
        '''
            force use which source to calc orientation
            0: auto select by stddev of vn/ins
            1: force use integral of angular rate
            2: force use vn/ins orientation
        '''
        self.force_use_orientation_mode = 0
        # if filter is ok to work
        self.filter_inited = False
        self.orientation_yaw = 0.0
        self.last_vn_msg = VectornavIns()
        self.vn_msg_recv = False
        self.imu_msg_recv = False
        self.last_recv_vn_time = 0
        self.last_recv_imu_time = 0
        
        self.last_rtk_position = None
        self.last_rtk_time = None
        
        self.ekf = ExtendedKalmanFilter()
        
        # subscription
        self.sub_rtk_observer = self.create_subscription(RtkObserver, "bob_eye/rtk_observer", self.rtk_observer_callback, 1)
        self.sub_vn_ins = self.create_subscription(VectornavIns, "a2rl/vn/ins", self.vn_ins_callback, 1)
        self.sub_imu = self.create_subscription(ImuGroup, "vectornav/raw/imu", self.imu_callback, 1)
        self.sub_kistler = self.create_subscription(Kistler_Vel_Angle, "a2rl/eav24_bsu/kistler_vel_angle", self.kistler_callback, 10)
        self.get_logger().info(f"self.sub_kistler.get_publisher_count(): {self.sub_kistler.get_publisher_count()}")
        self.sub_debug_mode = self.create_subscription(Int32, "bob_eye/debug/use_mode", self.debug_use_mode_callback, 1)
        # publisher
        self.pub_debug_integral_angle = self.create_publisher(Float32, "bob_eye/debug/integral_angle", 1)
        self.pub_rtk_speed = self.create_publisher(Float32, "bob_eye/debug/rtk_speed", 1)
        self.pub_estimate_localiaztion = self.create_publisher(VectornavIns, "bob_eye/estimate_local", 1)
        
        self.get_logger().info("BobEyeFilter initialized.")
    
    def debug_use_mode_callback(self, msg):
        self.force_use_orientation_mode = msg.data
    
    def rtk_observer_callback(self, msg):
        ego_x_observed = msg.position_enu_rtk.x
        ego_y_observed = msg.position_enu_rtk.y
        yaw_observed = self.orientation_yaw
        if self.last_rtk_time is None:
            self.last_rtk_position = (ego_x_observed, ego_y_observed)
            self.last_rtk_time = self.get_clock().now()
            return
        
        if (ego_x_observed, ego_y_observed) == self.last_rtk_position:
            # self.get_logger().info("Ignore repeat rtk data")
            return
        
        last_x, last_y = self.last_rtk_position
        dx, dy = ego_x_observed - last_x, ego_y_observed - last_y
        delta_dis = math.sqrt(dx * dx + dy * dy)
        # self.get_logger().warn(f"delta_dis: {delta_dis} m") 
        delta_time_sec = (self.get_clock().now() - self.last_rtk_time).nanoseconds / 1e9
        speed_observed = delta_dis / delta_time_sec
        
        debug_msg = Float32()
        debug_msg.data = speed_observed
        self.pub_rtk_speed.publish(debug_msg)
        self.last_rtk_position = (ego_x_observed, ego_y_observed)
        self.last_rtk_time = self.get_clock().now()
            
        # observed (x, y, theta)
        z = np.matrix([[ego_x_observed], [ego_y_observed], [yaw_observed], [speed_observed]])
        self.ekf.update(z)
        return
    
    def vn_ins_callback(self, msg):
        self.last_recv_vn_time = self.get_clock().now()
        self.vn_msg_recv = True
        self.last_vn_msg = msg

    def imu_callback(self, msg):
        if self.vn_msg_recv == False:
            return
        if self.imu_msg_recv == False:
            self.last_recv_imu_time = self.get_clock().now()
            self.imu_msg_recv = True
            return
        
        yaw_angular_rate = msg.angularrate.z
        vn_ori_stddev = self.last_vn_msg.orientation_stddev.z
        vn_ori = self.last_vn_msg.orientation_ypr.z
        use_integral = False
        if self.force_use_orientation_mode == 2:
            # if force use vn orientation
            use_integral = False
        elif self.force_use_orientation_mode == 1:
            # if force use integral
            use_integral = True
        elif self.force_use_orientation_mode == 0:
            if abs(vn_ori_stddev) < self.VN_ORI_STDDEV_TH and vn_ori_stddev != 0:
                # if orientation is reliable
                use_integral = False
            else:
                use_integral = True
        if use_integral:
            self.get_logger().info("Using integral...", throttle_duration_sec=0.5)
            now = self.get_clock().now()
            dt_s = (now - self.last_recv_imu_time).nanoseconds / 1e9
            tmp_angle = self.orientation_yaw - dt_s * yaw_angular_rate
            self.orientation_yaw = self.wrap_angle(tmp_angle)
        else:
            self.get_logger().info("Using vn_ins...", throttle_duration_sec=0.5)
            self.orientation_yaw = vn_ori
        msg = Float32()
        msg.data = self.orientation_yaw
        self.pub_debug_integral_angle.publish(msg)
        self.last_recv_imu_time = self.get_clock().now()
    
    def kistler_callback(self, msg):
        speed_mps_observed = msg.vel_x / 3.6
        yaw_observed = self.orientation_yaw
        self.ekf.predict(speed_mps_observed, yaw_observed, self.get_clock().now())
        
        # get ekf state
        ekf_state = self.ekf.get_state()
        ekf_cov = self.ekf.get_covariance_diagonal()
        msg = VectornavIns()
        msg.position_enu_ins.x = ekf_state[0][0]
        msg.position_enu_ins.y = ekf_state[1][0]
        msg.velocity_body_ins.x = speed_mps_observed
        msg.vel_stddev_ins = ekf_cov[3]
        msg.orientation_ypr.z = yaw_observed
        msg.pos_stddev_ins = math.sqrt(ekf_cov[0] * ekf_cov[0] + ekf_cov[1] * ekf_cov[1])
        self.pub_estimate_localiaztion.publish(msg)
        return
    
    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    filter_node = BobEyeFilter()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
