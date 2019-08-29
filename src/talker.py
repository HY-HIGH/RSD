#!/usr/bin/env python


import rospy

from people_detection.msg import MsgState

def talker():
    pub = rospy.Publisher('custom_chatter',MsgState)
    rospy.init_node('custom_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    msg=MsgState()
    msg.x_mid = 0.5
    msg.y_mid = 0.618
    msg.box_size = 0.150
    msg.box_count = 1
    
    while not rospy.is_shutdown():
      

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
