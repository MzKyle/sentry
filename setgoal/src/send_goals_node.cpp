#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#
#include <iostream>
using namespace std;

uint8_t notice_=1;
bool temp = true; //是否允许小车接收下一个目标信息
uint8_t temp2 = 9;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void sengoal(uint8_t notice);

void notice_callback(const std_msgs::UInt8::ConstPtr &msg){
    uint8_t goal_number = 2;
    ROS_INFO("data: %d",msg->data);
    notice_ = msg->data;
    if(temp2 != notice_){
        temp = true;
        temp2 = notice_;
    };

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal[4];
    // goal0 为巡逻区右侧
    // goal1 为巡逻区左侧
    // goal2 为前哨站
    // goal3 为补给点
    goal[0].target_pose.pose.position.x = -1;
    goal[0].target_pose.pose.position.y =  3;
    goal[0].target_pose.pose.orientation.z =  -0.306595935327;  
    goal[0].target_pose.pose.orientation.w = 0.951839761956; 

    goal[1].target_pose.pose.position.x = -1;
    goal[1].target_pose.pose.position.y =  4;
    goal[1].target_pose.pose.orientation.z =  -0.306595935327;  
    goal[1].target_pose.pose.orientation.w = 0.951839761956;

    goal[2].target_pose.pose.position.x = 6.5;
    goal[2].target_pose.pose.position.y =  -3.5;
    goal[2].target_pose.pose.orientation.z =  -0.306595935327;  
    goal[2].target_pose.pose.orientation.w = 0.951839761956;

    goal[3].target_pose.pose.position.x = -4.3;
    goal[3].target_pose.pose.position.y =  -4.5;
    goal[3].target_pose.pose.orientation.z =  -0.306595935327;  
    goal[3].target_pose.pose.orientation.w = 0.951839761956;
    
        ROS_INFO("notice=%d",notice_);
        if(notice_ == 4 && temp){
            goal[2].target_pose.header.frame_id = "map";
            goal[2].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[2]);
            ROS_INFO("sent goal!");
            notice_ = 0;
            temp = false;
        }

        if(notice_ == 8 && temp){
            temp = false;
            goal[1].target_pose.header.frame_id = "map";
            goal[1].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[1]);
            ROS_INFO("sent goal!");
            notice_ = 0;
            temp = false;
        }

        if(notice_ == 16 && temp){
            goal[3].target_pose.header.frame_id = "map";
            goal[3].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[3]);
            ROS_INFO("sent goal!");
            notice_ = 0;
            temp = false;
        }
        
    
    // ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The Goal achieved success !!!" );
    
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "send_goals_node");
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("notice", 10 , notice_callback);
    
    ros::spin();

    
    // 第一个待发送的 目标点 在 map 坐标系下的坐标位置
     

    // 第二个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[1].target_pose.pose.position.x = 3.24358606339;
    // goal[1].target_pose.pose.position.y = 0.977679371834;
    // goal[1].target_pose.pose.orientation.z = 0.647871240469;  
    // goal[1].target_pose.pose.orientation.w = 0.761749864308;  

    // // 第三个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[2].target_pose.pose.position.x = 2.41693687439;
    // goal[2].target_pose.pose.position.y = 1.64631867409;
    // goal[2].target_pose.pose.orientation.z = 0.988149484601;  
    // goal[2].target_pose.pose.orientation.w = 0.153494612555;  

    // // 第四个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[3].target_pose.pose.position.x =-0.970185279846;
    // goal[3].target_pose.pose.position.y = 0.453477025032;
    // goal[3].target_pose.pose.orientation.z = 0.946238058267;
    // goal[3].target_pose.pose.orientation.w = -0.323471076121;

    // goal[0].target_pose.header.frame_id = "map";
    // goal[0].target_pose.header.stamp = ros::Time::now();
    
    
    // ac.sendGoal(goal[0]);
    
    // ac.waitForResult();
    //     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //         ROS_INFO("The NO. %d Goal achieved success !!!", 4-goal_number );
    //         goal_number -- ;
    //     }else{ROS_WARN("The NO. %d Goal Planning Failed for some reason",4-goal_number); }
    
    // ROS_INFO(" Init success!!! ");
    // while(goal_number)    // total is 4 goals
    // {
    //     switch( (4 - goal_number) ){
    //         case 0:
    //                  goal[4 -goal_number].target_pose.header.frame_id = "map";
    //                  goal[4 -goal_number].target_pose.header.stamp = ros::Time::now();
    //                  ac.sendGoal(goal[4 -goal_number]);
    //                  ROS_INFO("Send NO. %d Goal !!!", 4-goal_number );
    //             break;
    //         case 1:
    //                  goal[4 -goal_number].target_pose.header.frame_id = "map";
    //                  goal[4 -goal_number].target_pose.header.stamp = ros::Time::now();
    //                  ac.sendGoal(goal[4 -goal_number]);
    //                  ROS_INFO("Send NO. %d Goal !!!", 4-goal_number );
    //             break;
    //         case 2:
    //                  goal[4 -goal_number].target_pose.header.frame_id = "map";
    //                  goal[4 -goal_number].target_pose.header.stamp = ros::Time::now();
    //                  ac.sendGoal(goal[4 -goal_number]);
    //                  ROS_INFO("Send NO. %d Goal !!!", 4-goal_number );
    //             break;
    //         case 3:
    //                  goal[4 -goal_number].target_pose.header.frame_id = "map";
    //                  goal[4 -goal_number].target_pose.header.stamp = ros::Time::now();
    //                  ac.sendGoal(goal[4 -goal_number]);
    //                  ROS_INFO("Send NO. %d Goal !!!", 4-goal_number );
    //             break;
    //         default:
    //             break;
    //     }
        
  return 0;}