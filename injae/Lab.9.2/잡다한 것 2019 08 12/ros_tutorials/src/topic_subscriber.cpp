#include "ros/ros.h"  //ros 기본 헤더 파일
#include "ros_tutorials/MsgTutorial.h"//메시지 파일 헤더

void msgCallback(const ros_tutorials::MsgTutorial::ConstPtr& msg) 
{
        ROS_INFO("receive msg = %d",msg->stamp.sec);
        ROS_INFO("receive msg = %d",msg->stamp.nsec);
        ROS_INFO("receive msg = %d",msg->data);
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"topic_subscriber");
    ros::NodeHandle nh;//노드 핸들러 선언 (글로벌 네임 스페이스 )
    ros::Subscriber ros_tutorials_sub=nh.subscribe("ros_tutorial_msg",100,msgCallback);
    ros::spin();// while 문 포함 
    return 0;
}

