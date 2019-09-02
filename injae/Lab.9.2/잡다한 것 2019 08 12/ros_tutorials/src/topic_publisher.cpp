#include "ros/ros.h"  //ros 기본 헤더 파일
#include "ros_tutorials/MsgTutorial.h"//메시지 파일 헤더

int main (int argc, char **argv)
{
    ros::init(argc,argv,"topic_publisher");
    ros::NodeHandle nh;//노드 핸들러 선언 (글로벌 네임 스페이스 )
    ros::Publisher ros_tutorials_pub=nh.advertise<ros_tutorials::MsgTutorial>("ros_tutorial_msg",100);
    ros::Rate loop_rate(10);
    ros_tutorials::MsgTutorial msg;
    int count =0;
    while(ros::ok())
    {
        msg.stamp=ros::Time::now();
        msg.data =count;

        ROS_INFO("send msg = %d",msg.stamp.sec);
        ROS_INFO("send msg = %d",msg.stamp.nsec);
        ROS_INFO("send msg = %d",msg.data);

        ros_tutorials_pub.publish(msg);
        loop_rate.sleep();
        ++count;

    }
    return 0;
}

