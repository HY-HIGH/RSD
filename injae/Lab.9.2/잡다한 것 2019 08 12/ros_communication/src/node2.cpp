#include "ros/ros.h"
#include "ros_communication/Position.h"
#include "ros_communication/SetPosition.h"
#include "ros_communication/State.h"
#include "ros_communication/SetState.h"

void msgCallback2(const ros_communication::Position::ConstPtr& msg)
{

}


bool state(ros_communication::SetState::Request &req,ros_communication::SetState::Response &res)
{

return true;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "node2");
ros::NodeHandle nh;

ros::Publisher state_pub = nh.advertise<ros_communication::State>("state_msg", 100);
ros::Subscriber position_sub = nh.subscribe("position_msg", 100, msgCallback2);
ros::ServiceServer set_state_ser = nh.advertiseService("set_state_srv", state);



return 0;
}