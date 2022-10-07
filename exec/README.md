# 포팅 메뉴얼

Morai 시뮬레이터가 있다고 가정하고 작성하겠다!

## ROS 설치

먼저 ROS와 관련된 패키지들을 설치해야 한다!

```
# -sc 사이에 띄어쓰기가 없는 줄 알고 해맷다...
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 여기서는 고정 IP 설정이 잘못되어있어 설치가 진행이 되지 않아 힘들었다... 네트워크 설정은 가장 아래에 정리하였다.
$ sudo apt install curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# apt 업데이트 후 ros 설치!
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo apt-get install python-rosdep

# ros 기본설정 진행
$ sudo rosdep init
$ rosdep update
$ echo “source /opt/ros/melodic/setup.bash” >> ~/.bashrc

# ros basic setting
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt-get install ros-melodic-velodyne
```

위의 명령어를 사용하면 잘 설치되었는지 확인이 가능하다.

```
$ roscore
```

필자의 경우 에러가 나서 구글링을 진행하였다. [ROS community](https://answers.ros.org/question/60366/problem-with-roscore/)에서 `sudo rosdep fix-permissions`을 실행하고 진행하라고 해서 따라했더니 잘 된다!

## ROS 패키지 환경설정

아래와 같이 패키지에 맞춰 환경설정을 진행한다.

```
# 원하는 폴더에서 깃랩 레포지토리를 클론해온다.
$ git clone https://lab.ssafy.com/s07-mobility-autodriving-sub2/S07P22D208.git

# catkin 환경변수 선언
# 필자의 레포지토리는 Desctop 안에 gitlab이라는 폴더 내에 있었으므로 아래와 같지만 각자 설치한 폴더경로에 맞게 작성해주어야 한다.
$ source ~/Desktop/gitlab/drive/catkin_ws/devel/setup.bash

# catkin_ws 폴더로 이동하여 ROS 패키지 빌드, 위와 마찬가지로 본인이 만든 폴더로 이동해준다.
$ cd ~/Desktop/gitlab/drive/catkin_ws
$ catkin_make

# catkin 패키지 재구축
$ rospack profile
```

## rosbridge 설치

window에서 실행되는 시뮬레이터와 virtualbox의 우분투에서 실행되는 ros를 연결해주는 rosbridge를 설치하고 설치가 잘 되었는지 확인한다.

```
# rosbidge 설치
$ sudo apt-get install ros-melodic-rosbridge-suite

# 이 환경변수는 rosbridge를 실행할 때마다 켜주어야 해서 파일에서 자동으로 만드는 것이 편하다...
$ source ~/Desktop/gitlab/drive/catkin_ws/devel/setup.bash

# rosbridge를 실행하여 잘 작동하는지 확인하자.
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

## 시뮬레이션과 ROS 통신

먼저 두 컴퓨터간의 통신이 되는지 확인해야하는데 윈도우 내의 가상환경이 아니라면 보통은 윈도우에 방화벽이 있어 통신이 안되는 경우가 있다. 방화벽을 신경쓰며 ping 테스트를 진행하여 두 컴퓨터가 통신이 가능하다는 것을 확인했다는 가정하에 진행하겠다.

ROS를 실행시킬 우분투 환경에서 `ifconfig` 명령어를 이용하여 ip 주소를 확인하고

<img src="https://user-images.githubusercontent.com/19484971/194457066-2384997b-2c03-4028-a9ae-ab0c0dad6f48.png" width=500>

우분투의 명령창에서 위에서 설치한 `rosbridge`를 실행한 후에

시뮬레이션의 네트워크, controll, publisher/subscriber, 센서 등에 ip 주소를 기입하고 연결한다.

`rosbridge`에 client가 연결되었다는 메시지가 뜨면 이번에는 다른 명령창에서 `roslaunch alpha_car alpha_car.launch`를 입력하여 실행시킨다.

앱을 통하여 목적지를 받으면 자동주행모드 `q`를 눌러 목적지까지 주행하는 모습을 보면 된다.
