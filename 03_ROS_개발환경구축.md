## ROS 한줄 설치 
* Ubuntu 버전에 따라서 지원하는 ROS 버전이 다르다 .
> * Ubuntu 16.04 -> ROS Kinetic, Ubuntu 18.04 -> ROS Melodic, Ubuntu20.04 -> ROS Noetic
* 환경 : Ubuntu 18.04 
* ROS 한줄 설치 
```
# ubuntu terminal
wget https://raw.githubusercontent.com/orocapangyo/meetup/master/190830/install_ros_melodic.sh && chmod 755 ./install_ros_melodic.sh && bash ./install_ros_melodic.sh
```
* home/사용자계정이름에 가보면 catkin_ws 폴더와 install_ros_melodic.sh 가 추가되어 있다. 
* .bashrc를 열어보면 단축키 설정 및 환경변수 설정 및 네트워크 설정이 자동으로 되어있다.
* 설치과정에서 필요한 환경설정을 자동으로 해주긴 하지만 시스템에는 따로 반영시켜야 한다.
```
# ubuntu terminal
source .bashrc
```

## ROS 동작 테스트 
* turtlesim 패키지 
```
# ubuntu terminal
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
rosrun rqt_graph rqt_graph
```
