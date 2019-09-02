#include "ros/ros.h"
#include "ros_communication/Position.h"
#include "ros_communication/SetPosition.h"
#include "ros_communication/State.h"
#include "ros_communication/SetState.h"

void msgCallback2(const ros_communication::Position::ConstPtr& msg)
{

}
void msgCallback3(const ros_communication::Position::ConstPtr& msg)
{

}





int main(int argc, char **argv)
{
ros::init(argc, argv, "node2");
ros::NodeHandle nh;


ros::Subscriber position_sub = nh.subscribe("position_msg", 100, msgCallback2);
ros::Subscriber state_sub = nh.subscribe("state_msg", 100, msgCallback3);



return 0;
}