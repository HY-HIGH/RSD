#include "ros/ros.h"
#include "ros_communication/Position.h"
#include "ros_communication/SetPosition.h"
#include "ros_communication/State.h"
#include "ros_communication/SetState.h"


ros_communication::Position Pos;
//노드 2는 rosservice call /set_state 를 통해 요청된 state 값을 state 토픽을 통해 퍼블리시를 합니다. 
//요청된 state 값이 0,1,2중의 값이 아니면 ROS_WARN을 통해 잘못된값을 요청했다고 출력하고 요청받기 전의 값을 유지합니다. 
//서비스의 result를 false로 리턴합니다. 구독한 position값의 x또는 y또는 z중 하나라도 5를 넘으면 자동으로 state 값을 1로 변경하고, 
//x 또는 y또는 z중 하나라도 -20이 넘으면 자동으로 state값을 2로 변경합니다.


void msgCallback(const ros_communication::Position::ConstPtr& msg)
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
ros::Subscriber position_sub = nh.subscribe("position_msg", 100, msgCallback);
ros::ServiceServer set_state_ser = nh.advertiseService("set_state_srv", state);

while()
ros::spin();

return 0;
}