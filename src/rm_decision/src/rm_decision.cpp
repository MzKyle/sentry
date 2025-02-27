#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rm_decision/rm_decision.h"
#include "rm_decision/mynode.h"
// file that contains the custom nodes definitions

int main(int argc, char ** argv)
{
  // 创建行为树加工厂
  BT::BehaviorTreeFactory factory;
  // 初始化 ROS2
  rclcpp::init(argc,argv);
  // 创建ROS节点
  // 修改；需要一个继承自rclcpp::Node的类
  //auto node2 = std::make_shared<RM_bt>();
  auto node2 = std::make_shared<RM_bt>();
  // 注册自定义的行为树节点
  factory.registerFromPlugin("./build/rm_decision/libmynode_plugin.so");
  // 注册ros节点
  factory.registerNodeType<ROS2DecoratorNode>("ROS2DecoratorNode");
  // 从文件创建一个行为树
  auto tree = factory.createTreeFromFile("./src/rm_decision/bt_tree.xml");
  // 启动监视
  BT::Groot2Publisher publisher(tree);
  // 行为树，启动！
  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
    //修改
    rclcpp::spin_some(node2);
    //
  }
  // 释放资源
  rclcpp::shutdown();
  return 0;
}