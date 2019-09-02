#include "ros/ros.h"
#include "ros_communication/Position.h"
#include "ros_communication/SetPosition.h"
#include "ros_communication/State.h"
#include "ros_communication/SetState.h"

#define offset 0

ros_communication::Position Pos;

void msgCallback(const ros_communication::State::ConstPtr& msg)
{
    if(msg->state == 0){
    
    }
    if(msg->state == 1){
        Pos.x=Pos.x*(-1);
        Pos.y=Pos.y*(-1);
        Pos.z=Pos.z*(-1);

    }
    if(msg->state == 2){
        Pos.x=Pos.x+offset;
        Pos.y=Pos.y+offset;
        Pos.z=Pos.z+offset;
    }
}


bool position(ros_communication::SetPosition::Request &req,ros_communication::SetPosition::Response &res)
{
    
    

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node1");
    ros::NodeHandle nh;

    nh.setParam("param", offset);

    ros::Publisher position_pub = nh.advertise<ros_communication::Position>("position_msg", 100);
    ros::Subscriber state_sub = nh.subscribe("state_msg", 100, msgCallback1);
    ros::ServiceServer set_position_ser = nh.advertiseService("set_position_srv", position);

    ros::Rate r(10); 
    while (ros::ok())
    {
        position_pub.publish(Pos)
        nh.getParam("offset", g_operator);
        ros::spinOnce();
        r.sleep();
    } 
    return 0;
}

