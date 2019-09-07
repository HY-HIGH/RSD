HY_HIGH

REINFORCEMENT SELFIE DRONE


9월 6일 까지 

-1차 판넬 자료 제출 

-보고자료

-젯슨

추석 (9/12~9/15)

2차 심사 전까지 

-드론제작

-판넬 수정

------------------------------------------------------
#1.introduction 개요
강화학습 내용
제작 배경 및 필요성
ㅇ
드론 

강화학습 의 필요성
제어방식

-PID의 
?
-강화 학습

셀피 드론 필요성
여행 불가능한 상황속 필요성


------------------------------------------------------
#2.project diagram 도식화
알고리즘
그림
수식,도식화

------------------------------------------------------
#3.작품 상세 설명
핵심코드
peple-detect detectiong
보상,액션
dqn 학습 방정식-벨만 방정식
pid

(env, agent, DQN 설명)
objectdetection ->사람만을 검출하고 박스의 정보 추출 수정
ros, gazebo -> 가상 강화 학습 환경 및 시뮬레이션
PID제어 ->비교군 설정 matlab 그래프  
jetson->학습된 모델을 사용하여 네트워크 (학습 또는) 

드론
-제작과정(fusion360)
-실제 구동 사진(simulation)

------------------------------------------------------
#4.concluding remarks 작품 장점 및 발전 가능성

강화 학습 결과 사진

baseline DQN->a3c 


# 수정필요!! 지금 문제점 물체가 안잡혀서 리셋이 될때 사람이 디텍션 되면 리셋 위치가 아닌데도 디텍션을 해서 스텝을 진행

#그냥 하늘로 떠버림

#리워드 -1 -1 31.xxx

#스코어가 490 점 넘으면 저장
#  succsss 10wja 100점 20~30 점 
# launch 파일 작성







#-----------------------------------------------------------------------------
#bashrc
#cd + pwd
cd() { builtin cd "$@" && pwd; }

#cd + ls 
cd() { builtin cd "$@" && pwd && ls; }

#short cut
alias at='source ~/tensorflow/bin/activate'
alias sn='shutdown now'



#gedit .bashrc
alias sb='source ~/.bashrc'
alias eb='gedit ~/.bashrc'

#office
alias lo='libreoffice --writer'
alias ch='/usr/bin/google-chrome-stable'

#ROS Control
alias sb='source ~/.bashrc'
alias eb='gedit ~/.bashrc'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
#
alias goto='rosservice call /multi_setpoint_local -- POINT'
alias offboard='rostopic pub /multi/set_mode std_msgs/String "offboard"'
alias land='rostopic pub /multi/set_mode std_msgs/String "auto.land"'
alias arm='rostopic pub /multi/arming std_msgs/Bool 1'
alias disarm='rostopic pub /multi/arming std_msgs/Bool 0'

alias rlworld='roslaunch swarm_ctrl_pkg iitp_test.launch'
alias rlpid='roslaunch selfie_drone selfie.launch' 
alias rldetect='roslaunch tensorflow_object_detector usb_cam_detector.launch'

source  /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

#export ROS_MASTER_URI=http://192.168.0.107:11311
#export ROS_HOSTNAME=192.168.0.107

export ROS_LAUNCH_SSH_UNKNOWN=1
export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/usr/include/python2.7/"
