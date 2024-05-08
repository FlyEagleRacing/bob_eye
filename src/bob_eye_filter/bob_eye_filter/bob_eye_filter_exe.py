import os
import rclpy
from rclpy.node import Node
import numpy as np
from vectornav_msgs.msg import RtkObserver, ImuGroup
from a2rl_bs_msgs.msg import VectornavIns, Kistler_Vel_Angle
from std_msgs.msg import Float32, Int32
import math

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
        
        # subscription
        self.sub_rtk_observer = self.create_subscription(RtkObserver, "bob_eye/rtk_observer", self.rtk_observer_callback, 1)
        self.sub_vn_ins = self.create_subscription(VectornavIns, "a2rl/vn/ins", self.vn_ins_callback, 1)
        self.sub_imu = self.create_subscription(ImuGroup, "vectornav/raw/imu", self.imu_callback, 1)
        self.sub_kistler = self.create_subscription(Kistler_Vel_Angle, "a2rl/eav24_bsu/kistler_vel_angle", self.kistler_callback, 1)
        self.sub_debug_mode = self.create_subscription(Int32, "bob_eye/debug/use_mode", self.debug_use_mode_callback, 1)
        # publisher
        self.pub_debug_integral_angle = self.create_publisher(Float32, "bob_eye/debug/integral_angle", 1)
        
        self.get_logger().info("BobEyeFilter initialized.")
    
    def debug_use_mode_callback(self, msg):
        self.force_use_orientation_mode = msg.data
    
    def rtk_observer_callback(self, msg):
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
            if abs(vn_ori_stddev) < self.VN_ORI_STDDEV_TH:
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
