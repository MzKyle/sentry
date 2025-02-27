//头文件
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auto_aim_interfaces/msg/referee.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
using std::placeholders::_1;
using namespace std::chrono_literals;
#ifndef RM_BT_H
#define RM_BT_H

//值的定义
geometry_msgs::msg::PoseStamped position_value;
std::string outpost_hp_value="";
std::string base_hp_value="";
std::string rfid_value="";
std::string projectile_allowance_17mm_value="";
std::string sentry_hp_value="";
auto_aim_interfaces::msg::Referee referee_info_value;

//创建ros对行为树的修饰器
class ROS2DecoratorNode  : public BT::SyncActionNode {
public:
  ROS2DecoratorNode(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {

    getInput("position", position_value);
    setOutput("referee_info", referee_info_value);

    return BT::NodeStatus::SUCCESS;
  }


  static BT::PortsList providedPorts()
  {

    return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("position"),
    BT::OutputPort<auto_aim_interfaces::msg::Referee>("referee_info"),

    };
  }
};


class RM_bt : public rclcpp::Node {
  public:
    RM_bt():Node("RM_bt")
    {
    RCLCPP_INFO(this->get_logger(), "RM_bt,go");
    //创建订阅方

    subscription_referee_info =this->create_subscription<auto_aim_interfaces::msg::Referee>("referee_info", 10,std::bind(&RM_bt::topic_callback_referee_info, this, std::placeholders::_1));


    //创建发布方

    publisher_position = this->create_publisher<geometry_msgs::msg::PoseStamped>("position", 10);
    timer_position = this->create_wall_timer(500ms, std::bind(&RM_bt::timer_callback_position, this));


    }
    //回调函数

    void topic_callback_referee_info(const auto_aim_interfaces::msg::Referee & msg)
    {
      referee_info_value=msg;
    }


    //定时器

    void timer_callback_position()
    {
      auto message = geometry_msgs::msg::PoseStamped();
      message = position_value;
      publisher_position->publish(message);
    }


  private:
    //初始化订阅方

    rclcpp::Subscription<auto_aim_interfaces::msg::Referee>::SharedPtr subscription_referee_info;

   
    //初始化发布方

    rclcpp::TimerBase::SharedPtr timer_position;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_position;

};

#endif