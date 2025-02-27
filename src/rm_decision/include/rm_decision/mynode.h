//头文件
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auto_aim_interfaces/msg/referee.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <iostream>
namespace mynode
{
    using BT::NodeStatus;
    //行为树节点类

    //to_goal叶节点
    class to_goal : public BT::SyncActionNode
    {
        public:
            to_goal(const std::string& name, const BT::NodeConfig& config) :
            BT::SyncActionNode(name, config),
            node_(rclcpp::Node::make_shared("to_goal_node")),
            publisher_(node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10))
            {
            }
            // 给该节点申明端口
            static BT::PortsList providedPorts()
            {
                return { 
                    //端口设置
                        BT::InputPort<geometry_msgs::msg::PoseStamped>("position"),
                };
            }
            BT::NodeStatus tick() override
            {
                auto res = getInput<geometry_msgs::msg::PoseStamped>("position");
                if (!res)
                {
                    std::cerr << "Failed to get input [position]: " << res.error() << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
                auto goal = res.value();
                publisher_->publish(goal);
                return BT::NodeStatus::SUCCESS;
            }

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    };



    //referee叶节点
    class referee : public BT::SyncActionNode
    {
    public:
        referee(const std::string& name, const BT::NodeConfig& config) :
            BT::SyncActionNode(name, config),
            node_(rclcpp::Node::make_shared("referee_node")),
            msg_received_(false)
        {
            sub_ = node_->create_subscription<auto_aim_interfaces::msg::Referee>(
                "referee_info", 10,
                [this](const auto_aim_interfaces::msg::Referee::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_msg_ = *msg;
                    msg_received_ = true;
                });
        }
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<std::string>("outpost_hp"),
                BT::OutputPort<std::string>("base_hp"),
                BT::OutputPort<std::string>("rfid"),
                BT::OutputPort<std::string>("projectile_allowance_17mm"),
                BT::OutputPort<std::string>("sentry_hp")
            };
        }
        BT::NodeStatus tick() override
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!msg_received_)
                {
                    std::cerr << "No referee info received yet." << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }
            auto current_msg = last_msg_;

            setOutput("outpost_hp", std::to_string(current_msg.outpost_hp));
            setOutput("base_hp", std::to_string(current_msg.base_hp));
            setOutput("rfid", std::to_string(current_msg.rfid));
            setOutput("projectile_allowance_17mm", std::to_string(current_msg.projectile_allowance_17mm));
            setOutput("sentry_hp", std::to_string(current_msg.sentry_hp));

            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Referee>::SharedPtr sub_;
        auto_aim_interfaces::msg::Referee last_msg_;
        bool msg_received_;
        std::mutex mutex_;
    };

    //哨兵血量检查节点
    class CheckSentryHP : public BT::ConditionNode
    {
        public:
            CheckSentryHP(const std::string& name, const BT::NodeConfig& config) :
            BT::ConditionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<std::string>("sentry_hp"),
                    BT::InputPort<std::string>("threshold")
                };
            }

            BT::NodeStatus tick() override
            {
                auto hp = getInput<std::string>("sentry_hp");
                auto threshold = getInput<std::string>("threshold");

                if (!hp || !threshold)
                {
                    std::cerr << "Failed to get input [sentry_hp] or [threshold]: " << hp.error() << " " << threshold.error() << std::endl;
                    return BT::NodeStatus::FAILURE;
                }

                if (std::stoi(hp.value()) < std::stoi(threshold.value()))
                {
                    return BT::NodeStatus::SUCCESS;
                }
                return BT::NodeStatus::FAILURE;
            }
    };

    //哨兵弹丸检查节点
    class CheckAllowance17mm : public BT::ConditionNode
    {
        public:
            CheckAllowance17mm(const std::string& name, const BT::NodeConfig& config) :
            BT::ConditionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<std::string>("allowance_17mm"),
                    BT::InputPort<std::string>("threshold")
                };
            }

            BT::NodeStatus tick() override
            {
                auto allowance = getInput<std::string>("allowance_17mm");
                auto threshold = getInput<std::string>("threshold");

                if (!allowance || !threshold)
                {
                    std::cerr << "Failed to get input [allowance_17mm] or [threshold]: " << allowance.error() << " " << threshold.error() << std::endl;
                    return BT::NodeStatus::FAILURE;
                }

                if (std::stoi(allowance.value()) < std::stoi(threshold.value()))
                {
                    return BT::NodeStatus::SUCCESS;
                }
                return BT::NodeStatus::FAILURE;
            }
    };

    inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<to_goal>("to_goal");
        factory.registerNodeType<referee>("referee");
        factory.registerNodeType<CheckSentryHP>("CheckSentryHP");
        factory.registerNodeType<CheckAllowance17mm>("CheckAllowance17mm");
    }
}