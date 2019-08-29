#!/usr/bin/env python
import rospy
from people_detection.msg import MsgState

def callback(data):
    rospy.loginfo('I heard x_mid     %lf', data.x_mid)
    rospy.loginfo('I heard y_mid     %lf', data.y_mid)
    rospy.loginfo('I heard box_size  %lf', data.box_size)
    rospy.loginfo('I heard box_count %d', data.box_count)


def listener():

    
    rospy.init_node('custom_listener', anonymous=True)

    rospy.Subscriber('custom_chatter', MsgState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
