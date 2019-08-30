#!/usr/bin/env python
#-*- coding:utf-8 -*-

import os
import sys
import cv2
import time #타임 스탬프
import math
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image #이미지 처리

#ROS related imports
import rospy


# Object detection module imports
import std_msgs.msg
from tensorflow_object_detector.msg import MsgState

#Drone Control module imports
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from swarm_ctrl_pkg.srv import srvMultiSetpointLocal, srvMultiSetpointLocalRequest


realtimestate=MsgState()#상태를 저장하기 위한 객체
pose=PoseStamped()#드론의 현재위치를 받아서 저장하는 객체
fail_detect=0  #디텍트 실패시 카운트

def poseenvCB(posedata):
    """To updateDrone's local position 
    """
    global pose
    pose=posedata
def stateenvCB(msgdata):
    """To updateDrone's state 
    """
    global realtimestate
    realtimestate=msgdata


class State():
    X_MID   = 0.0
    Y_MID   = 0.0
    BOX_SIZE= 0.0   

class Environment:
    def __init__(self, image_shape=(720, 1280)):
        self.current_episode  = 0
        self.current_timestep = 0
        self.current_frame    = None
        self.current_output   = None
        self.state_sub = rospy.Subscriber('data_state',MsgState,callback=stateenvCB)
        self.pose_sub=rospy.Subscriber("/camila1/mavros/local_position/pose",PoseStamped,callback=poseenvCB)
        rospy.init_node('DQNAgent', anonymous=False)
        rospy.wait_for_service('/multi_setpoint_local')
        self.goto_env_client = rospy.ServiceProxy('/multi_setpoint_local', srvMultiSetpointLocal)

    def state_to_array(self, state):
        out = np.zeros((3,), dtype='float64')
        out[0] = float(state.X_MID)
        out[1] = float(state.Y_MID)
        out[2] = float(state.BOX_SIZE)
        return out

    def reset(self):
        self.current_episode  += 1
        self.current_timestep =0
        print('now init_x,y,z')##테스트
        #초기화 init
        # init_x = 4.0
        # init_y = -1.0
        # init_z = 2.0
        init_x = 3.0
        init_y = -9.0
        init_z = 8.0

        self.goto_env_client("POINT",init_x,init_y,init_z)
        while True :
            if (abs(pose.pose.position.x-init_x)<0.5 and abs(pose.pose.position.y-init_y)<0.5 and abs(pose.pose.position.z-init_z)<0.5):
                break
            else:
                pass
        _state = State()
        _state.X_MID = realtimestate.x_mid 
        _state.Y_MID = realtimestate.y_mid
        _state.BOX_SIZE = realtimestate.box_size
        print ("\n-----Parameters-----")
        print ("X_MID   :", _state.X_MID)
        print ("Y_MID   :", _state.Y_MID)
        print ("BOX_SIZE:", _state.BOX_SIZE)
        self.current_state  = _state 

        return self.state_to_array(_state)

    def step(self, action):
        # TRACKER
        done   = False
        reward = 0.0 #누적될 필요는 없다
        target_x_mid=0.5
        target_y_mid=0.618
        target_box_size=0.07 
        reward_negative_weight=-10
        #reward_possitive_weight=10
        error=0.05
        success=0.03
        success_image_capture=False
        reward_possitive = 0.0
        reward_negative = 0.0

        global fail_detect

        print ("\n-----Parameters-----")
        print ("Delta X_MID      :", self.current_state.X_MID)
        print ("Delta Y_MID      :", self.current_state.Y_MID)
        print ("Delta BOX_SIZE   :", self.current_state.BOX_SIZE)
        

        dist_x = abs(target_x_mid - self.current_state.X_MID)
        dist_y = abs(target_y_mid - self.current_state.Y_MID)
        dist_box_size = abs(target_box_size - self.current_state.BOX_SIZE)

        
        
        #보상
        # if (dist_x <=error) and (dist_y <=error) and (dist_box_size <=error) :
        #     reward_possitive=reward_possitive_weight*((1-dist_x)+(1-dist_y)+(1-dist_box_size))
        
            #Reward: state value가 작을 수록 reward는 커진다.
        if ((self.current_state.X_MID)!=-1 and (self.current_state.Y_MID))!=-1 :
            reward_negative = reward_negative_weight*(dist_x+dist_y+dist_box_size)


        reward    = (reward_possitive + reward_negative)


        print ("Distance_X:%lf" % dist_x)
        print ("Distance_Y:%lf" % dist_y)
        print ("Distance_Box_Size:%lf" % dist_box_size)
        print ("Reward:%lf" % reward)

        
        # NEXT frame
        _state = State()
        
        
        #done 상태의 정의
        # 일정 time동안 detect 실패(=xmid,ymid value가 전부 -1 )시 학습을 재시작 하며 reward=-50으로 준다.
        if ((self.current_state.X_MID==-1) or (self.current_state.Y_MID==-1) or (self.current_state.BOX_SIZE>=0.8)):
            fail_detect=fail_detect-1
            print('faild_detect_count %d'%fail_detect)
            if(fail_detect<-5):
                reward-=50
                done   = True
            elif(self.current_state.X_MID>0) or (self.current_state.Y_MID>0):
                if(self.current_state.BOX_SIZE>=0.8):
                    reward-=50
                    done   = True
                else:
                    fail_detect=0
        #드론의 z value가 0.3 이하가 되면 종료
        if (pose.pose.position.z<0.3):
            reward-=50
            done   = True

        elif (dist_x <=success) and (dist_y <=success) and (dist_box_size <=(4*success)) : #box_size는 다른것의 기준 4배 범위준다
            reward+=10 #+ 점수 더줄 필요 있음
            done   = True
            success_image_capture=True
        

        else:
            _state.X_MID = realtimestate.x_mid
            _state.Y_MID = realtimestate.y_mid
            _state.BOX_SIZE = realtimestate.box_size
            self.current_state = _state
       

        print ("Reward     :", reward)
        print ("Action RL  :", action ,"stage")
        print ("Done       :", done)

        self.current_timestep += 1
        reward+=(self.current_timestep*(-1))

        return self.state_to_array(_state), reward, done, success_image_capture

if __name__ == "__main__":
    rospy.init_node('Environment', anonymous=False)
    while True:
       pass
       # print("Done")

#에피소드의 끝은 사진찍기