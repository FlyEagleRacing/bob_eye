#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/rtk_observer.hpp"
#include "a2rl_bs_msgs/msg/vectornav_ins.hpp"
#include "rtk_observer/conversions.h"

class RTKObserver : public rclcpp::Node {
public:
  RTKObserver() : Node("rtk_observer")
  {
    lla_2_enu.fixReferenceCoord(utils::LLA(24.4710399, 54.605548, 0.0));
    gps_raw_sub = this->create_subscription<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps", 10, std::bind(&RTKObserver::rtk_gps_callback, this, std::placeholders::_1));
    vn_ins_sub = this->create_subscription<a2rl_bs_msgs::msg::VectornavIns>("a2rl/vn/ins", 10, std::bind(&RTKObserver::vn_ins_callback, this, std::placeholders::_1));
    observer_pub = this->create_publisher<vectornav_msgs::msg::RtkObserver>("bob_eye/rtk_observer", 10);
    RCLCPP_INFO(this->get_logger(), "RtkObserver initialized.");
  }
private:
  rclcpp::Publisher<vectornav_msgs::msg::RtkObserver>::SharedPtr observer_pub;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr gps_raw_sub;
  rclcpp::Subscription<a2rl_bs_msgs::msg::VectornavIns>::SharedPtr vn_ins_sub;
  a2rl_bs_msgs::msg::VectornavIns last_vn_msg;

  utils::LocalFrame lla_2_enu;
  void rtk_gps_callback(const vectornav_msgs::msg::GpsGroup::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "rtk_gps_callback");
    auto pub_msg = vectornav_msgs::msg::RtkObserver();

    const auto enu = lla_2_enu(utils::LLA(msg->poslla.x, msg->poslla.y, msg->poslla.z));
    pub_msg.position_enu_rtk.x = enu.x;
    pub_msg.position_enu_rtk.y = enu.y;
    pub_msg.position_enu_rtk.z = enu.z;

    pub_msg.orientation_ypr_ins.x = last_vn_msg.orientation_ypr.x;
    pub_msg.orientation_ypr_ins.y = last_vn_msg.orientation_ypr.y;
    pub_msg.orientation_ypr_ins.z = last_vn_msg.orientation_ypr.z;

    pub_msg.orientation_stddev_ypr_ins.x = last_vn_msg.orientation_stddev.x;
    pub_msg.orientation_stddev_ypr_ins.y = last_vn_msg.orientation_stddev.y;
    pub_msg.orientation_stddev_ypr_ins.z = last_vn_msg.orientation_stddev.z;
    observer_pub->publish(pub_msg);
  }

  void vn_ins_callback(const a2rl_bs_msgs::msg::VectornavIns::SharedPtr msg)
  {
    last_vn_msg = *msg;
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<RTKObserver>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
