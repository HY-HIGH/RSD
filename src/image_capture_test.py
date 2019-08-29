#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
#/iitp_drone/camera_1/image_raw
#sensor_msgs/Image

set_image=Image()

def ImageCB(Imagedata):
    
    rospy.loginfo('I got Image')
    set_image.data              =Imagedata.data
    set_image.encoding          =Imagedata.encoding
    set_image.header.frame_id   =Imagedata.header.frame_id   
    set_image.header.seq        =Imagedata.header.seq        
    set_image.header.stamp.nsecs=Imagedata.header.stamp.nsecs
    set_image.header.stamp.secs =Imagedata.header.stamp.secs 
    set_image.height            =Imagedata.height            
    set_image.is_bigendian      =Imagedata.is_bigendian      
    set_image.step              =Imagedata.step              
    set_image.width             =Imagedata.width             

def capture():
    
    rate = rospy.Rate(10) 
    
    
    pub.publish(set_image)
    
    rate.sleep()


if __name__ == '__main__':
    rospy.init_node('image', anonymous=True)
   
    pub = rospy.Publisher('image_capture',Image,queue_size=10)
    sub = rospy.Subscriber('/iitp_drone/camera_1/image_raw',Image, ImageCB)
  
    while not rospy.is_shutdown():
        success_image_capture=True
        print("hello")

        if success_image_capture == True:
            capture()
    
