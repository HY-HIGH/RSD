#!/usr/bin/env python
#-*- coding:utf-8 -*-

#Reinforcement Learning related imports
import sys
import pylab
import random
import numpy as np
from collections import deque
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import Sequential2
try:
    from env import Environment 
except ImportError:
    print("import Environment one more check")
    sys.exit(1)

#ROS related imports
try:
    import rospy
except ImportError:
    print("Unable to import rospy!")
    print("Please to check it.")
    sys.exit(1)

# Object detection module imports
import std_msgs.msg
from tensorflow_object_detector.msg import MsgState

#Drone Control module imports
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Quaternion
from swarm_ctrl_pkg.srv import srvMultiSetpointLocal, srvMultiSetpointLocalRequest
from sensor_msgs.msg import Image #이미지 캡쳐
#Max Episode 300
EPISODES = 300

#드론의 현재위치를 받아서 저장하는 object 
#gazebo에서 위치를 publishing 때마다 계속 업데이트
quad_pose=PoseStamped()
#image_capture
set_image=Image()

def poseagentCB(posedata):
    """To updateDrone's local position 
    """
    global quad_pose
    quad_pose=posedata

def ImageCB(Imagedata):
    
    #rospy.loginfo('I got Image')
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

def image_capture():
    
    rate = rospy.Rate(10) 
    
    
    pub.publish(set_image)
    
    rate.sleep()
class DQNAgent:
    """Learning Agent
    """
    def __init__(self, state_size, action_size):
        
        self.load_model = False
        
        # Define state size and action size(상태와 행동의 크기 정의)
        self.state_size = state_size
        self.action_size = action_size

        # DQN Hyperparameter(하이퍼파라미터)
        self.discount_factor = 0.99
        self.learning_rate = 0.001
        self.epsilon = 1.0
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.01
        self.batch_size = 64
        #Minimum learning period(최소 학습 주기)
        self.train_start = 1000 
        #Replay Memory maxlen=2000(리플레이 메모리 최대 2000)
        self.memory = deque(maxlen=2000)

        # model:Saving temp weight, target_model=Updating model's weight 
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()

        self.pose_sub=rospy.Subscriber("/camila1/mavros/local_position/pose",PoseStamped,callback=poseagentCB)

        # if self.load_model:
        #     self.model.load_weights("./save_model/cartpole_dqn_trained.h5")

    # 상태가 입력, 큐함수가 출력인 인공신경망 생성
    def build_model(self):
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu',
                        kernel_initializer='he_uniform'))
        model.add(Dense(24, activation='relu',
                        kernel_initializer='he_uniform'))
        model.add(Dense(self.action_size, activation='linear',
                        kernel_initializer='he_uniform'))
        model.summary()
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate)) #loss 는 mean square equation optimizer는 gradient decent의 방식
        return model
        #수정 필요 아웃풋이 7개일 필요는 없는가?

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)#해당 범위 내의 인덱스를 랜덤하게 1개 반환한다.
        else:
            q_value = self.model.predict(state)
            return np.argmax(q_value[0])#결과가 [[1,2,3,4,5,7]]의 형태를 가지기 때문이다.

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습
    def train_model(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, self.batch_size)

        states = np.zeros((self.batch_size, self.state_size))
        next_states = np.zeros((self.batch_size, self.state_size))
        actions, rewards, dones = [], [], []

        for i in range(self.batch_size):
            states[i] = mini_batch[i][0]
            actions.append(mini_batch[i][1])
            rewards.append(mini_batch[i][2])
            next_states[i] = mini_batch[i][3]
            dones.append(mini_batch[i][4])

        # 현재 상태에 대한 모델의 큐함수
        # 다음 상태에 대한 타깃 모델의 큐함수
        target = self.model.predict(states)
        target_val = self.target_model.predict(next_states)

        # 벨만 최적 방정식을 이용한 업데이트 타깃 (배치 모델 업데이트)
        for i in range(self.batch_size):
            if dones[i]:
                target[i][actions[i]] = rewards[i]
            else:
                target[i][actions[i]] = rewards[i] + self.discount_factor * (
                    np.amax(target_val[i]))

        self.model.fit(states, target, batch_size=self.batch_size,
                       epochs=1, verbose=0)


def interpret_action(action):
    scaling_factor   = 0.5 #위치 이동값
    if action == 0:
        quad_action = (0, 0, 0)
    elif action == 1:
        quad_action = (scaling_factor, 0, 0)
    elif action == 2:
        quad_action = (0, scaling_factor, 0)
    elif action == 3:
        quad_action = (0, 0 , 0.2)# z값 고정 0.2
    elif action == 4:
        quad_action = (-scaling_factor, 0, 0)
    elif action == 5:
        quad_action = (0, -scaling_factor, 0)
    elif action == 6:
        quad_action = (0, 0, -0.2)

    return quad_action


if __name__ == "__main__":
   
    env = Environment(image_shape=(1280, 720))
    state_size = 3
    action_size = 7
    threshehold_pose = 0.1

    # DQN 에이전트 생성
    rospy.init_node('DQNAgent', anonymous=False)
    
   
    pub = rospy.Publisher('image_capture',Image,queue_size=10)
    sub = rospy.Subscriber('/iitp_drone/camera_1/image_raw',Image, ImageCB)
  
    agent = DQNAgent(state_size, action_size)
    rospy.wait_for_service('/multi_setpoint_local')
    goto_agent_client = rospy.ServiceProxy('/multi_setpoint_local', srvMultiSetpointLocal)
    scores, episodes = [], []

    for e in range(EPISODES):
        done = False
        score = 0
        print("now_reset")##필요없음
        state = env.reset()
        state = np.reshape(state, [1, state_size])#배열 reshape하는 코드 이후 맞는지 check  수정
        
        while not done:
            action = agent.get_action(state)#현재 state에 대한 action결과를 quad_action에 저장하고
            quad_action=interpret_action(action)
            
            #드론이 실제로 움직일 좌표
            quad_action_offset_x=quad_pose.pose.position.x+quad_action[0]
            quad_action_offset_y=quad_pose.pose.position.y+quad_action[1] 
            quad_action_offset_z=quad_pose.pose.position.z+quad_action[2]
            
            goto_agent_client("POINT",quad_action_offset_x,quad_action_offset_y,quad_action_offset_z)

            #드론 위치 이동시 실제 드론의 위치와 서비스 요청한 액션과의 비교 후 다음상태 받기
            #quad_pose다시 서브스크라이브후 quad_action_offset와 비교 
            while True:
                if ((abs(quad_pose.pose.position.x-quad_action_offset_x)<threshehold_pose) \
                    and (abs(quad_pose.pose.position.y-quad_action_offset_y)<threshehold_pose) \
                        and (abs(quad_pose.pose.position.z-quad_action_offset_z)<threshehold_pose)):
                    break
                else:
                    pass

            # 선택한 행동으로 환경에서 한 타임스텝 진행
            next_state, reward, done, success_image_capture = env.step(action)
            next_state = np.reshape(next_state, [1, state_size])
            if success_image_capture == True:
            
                image_capture()

                #/iitp_drone/camera_1/image_raw 을 토픽으로 하는 새로운 토픽 발행
               
                # capture() publish 이미지 캡쳐 노드 
                
            # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
            agent.append_sample(state, action, reward, next_state, done)
            # 매 타임스텝마다 학습
            if len(agent.memory) >= agent.train_start:
                agent.train_model()

            score += reward #score는 int scores는 리스트 형태
            state = next_state

            if done:
                #각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                agent.update_target_model()

                # 에피소드마다 학습 결과 출력
                scores.append(score)
                episodes.append(e)
                pylab.plot(episodes, scores, 'b')
                pylab.savefig("/home/injae/catkin_ws/src/RSD/src/save_graph/selfie_drone_dqn.png")
                print("episode:", e, "  score:", score, "  memory length:",
                      len(agent.memory), "  epsilon:", agent.epsilon)

                #수정필요# 이전 10개 에피소드의 점수 평균이 490보다 크면 학습 중단 
<<<<<<< HEAD
                if np.mean(scores[-min(10, len(scores)):]) > 490:
                    agent.model.save_weights("/home/injae/catkin_ws/src/RSD/src/save_model/selfie_drone_dqn.h5")
                    sys.exit()
=======
                if np.mean(scores[-min(10, len(scores)):]) > 490:#+점수가 10점 한번이니까 리워드를 더주고 스코어로는 부족 그냥 리워드가 30 이상일떄 학습을 종료하는  건 어떰? 
                    agent.model.save_weights("/home/injae/catkin_ws/src/people_detection/src/save_model/selfie_drone_dqn.h5")
                    sys.exit()




# 수정필요!! 지금 문제점 물체가 안잡혀서 리셋이 될때 사람이 디텍션 되면 리셋 위치가 아닌데도 디텍션을 해서 스텝을 진행
>>>>>>> 8108dd414dbcae555609f8890e3d0549b6665766
