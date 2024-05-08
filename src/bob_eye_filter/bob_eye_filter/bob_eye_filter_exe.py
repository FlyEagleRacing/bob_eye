import os
import rclpy
from rclpy.node import Node
import numpy as np
from vectornav_msgs.msg import RtkObserver, ImuGroup
from a2rl_bs_msgs.msg import VectornavIns, Kistler_Vel_Angle

class BobEyeFilter(Node):
    def __init__(self):
        super().__init__('bob_eye_filter')
        
        '''
            force use which source to calc orientation
            0: auto select by stddev of vn/ins
            1: force use integral of angular rate
            2: force use vn/ins orientation
        '''
        self.force_use_orientation_mode = 0
        # subscription
        self.sub_rtk_observer = self.create_subscription(RtkObserver, "bob_eye/rtk_observer", self.rtk_observer_callback, 1)
        self.sub_vn_ins = self.create_subscription(VectornavIns, "a2rl/vn/ins", self.vn_ins_callback, 1)
        self.sub_imu = self.create_subscription(ImuGroup, "vectornav/raw/imu", self.imu_callback, 1)
        self.sub_kistler = self.create_subscription(Kistler_Vel_Angle, "a2rl/eav24_bsu/kistler_vel_angle", self.kistler_callback, 1)
        
        self.get_logger().info("BobEyeFilter initialized.")
    
    def rtk_observer_callback(self, msg):
        return
    
    def vn_ins_callback(self, msg):
        return

    def imu_callback(self, msg):
        return
    
    def kistler_callback(self, msg):
        return
    

def main(args=None):
    rclpy.init(args=args)
    filter_node = BobEyeFilter()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
