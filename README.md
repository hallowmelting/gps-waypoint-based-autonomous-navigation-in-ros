# GPS-waypoint-based-Autonomous-Navigation-in-ROS
로봇은 장애물을 피하며 목적지로 안전하게 이동하기 위해 미리 정의된 GPS 지점을 따라갑니다.

이 저장소 패키지는 다음에서 테스트되었습니다:
### Custom Rover 스펙
- 엔비디아 젯슨 TX2 Ubuntu 18.04
- Razor 9DOF IMU
- ZED F9P (RTK2) GPS
- RPLidar A1 lidar
### The base station
- Laptop with Ubuntu 18.04

## 동기부여
이 작업은 Interplanetar 팀(BUET Mars Rover Robotics Team)이 2019년 대학 로버 챌린지(University Rover Challenge, URC)에 참가하기 위해 수행되었습니다. 이 작업을 오픈 소스로 공개한 Daniel Snider에게 특별한 감사를 전합니다:
- ROS Rover <[code](https://github.com/danielsnider/ros-rover)>
- Simple Drive <[code](https://github.com/danielsnider/simple_drive)> <[ROS Wiki](http://wiki.ros.org/simple_drive)>
- Follow Waypoints <[code](https://github.com/danielsnider/follow_waypoints)> <[ROS Wiki](http://wiki.ros.org/follow_waypoints)>
- GPS Goal <[code](https://github.com/danielsnider/gps_goal)> <[ROS Wiki](http://wiki.ros.org/gps_goal)>
- ROS Offline Google Maps for MapViz <[code](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite)>
- Advanced Terminal Organization <[code1](https://github.com/teamr3/URC/blob/master/.tmuxinator.yml)> <[code2](https://github.com/teamr3/URC/blob/master/devstuff/dan/.tmuxinator.yml)>

## Run the package

터미널에서 catkin_ws의 소스(src) 디렉터리로 이동한 다음 다음 명령을 실행하세요:
```
cd catkin_ws/src
git clone https://github.com/ArghyaChatterjee/gps-waypoint-based-autonomous-navigation-in-ros.git gps_waypoint_nav
cd ..
catkin_make --only-pkg-with-deps gps_waypoint_nav
```
그 터미널에서 navigation 파일을 실행하세요:
```
source devel/setup.bash
roslaunch gps_waypoint_nav gps_waypoint_nav.launch
```
다른 터미널에서 joystick controller 파일을 실행하세요:
```
source devel/setup.bash
roslaunch gps_waypoint_nav joy_launch_control.launch
```
조이스틱으로 로버를 실행하세요. 실행 중에 "LB" 버튼을 눌러 웨이포인트 수집을 시작하세요. 웨이포인트는 'points_outdoor.txt' 파일에 저장됩니다. 실행이 완료되면 "RB" 버튼을 눌러 웨이포인트를 따라가기 시작하세요. 

<p align="center">
    <img src="assets/gps_image.png", width="800">
</p>

## 패키지 설명
이 패키지는 다음 패키지들의 조합을 사용합니다:

   - ekf_localization: 오도메트리 데이터를 IMU와 GPS 데이터와 융합합니다.
   - navsat_transform: GPS 데이터를 오도메트리로 변환하고, 위도 및 경도 포인트를 로봇의 오도메트리 좌표 시스템으로 변환합니다.
   - GMapping: 지도를 생성하고 장애물을 감지합니다.
   - move_base: 장애물을 피하면서 목표지점으로 이동합니다. 
   - 목표지점은 녹화된 또는 입력된 웨이포인트를 사용하여 설정됩니다.

## 노드 설명
이 저장소의 네비게이션 패키지에는 다음과 같은 사용자 정의 노드가 포함되어 있습니다:
	
   - gps_waypoint: 웨이포인트 파일을 읽어 웨이포인트를 지도 프레임 내의 점으로 변환한 다음 목표지점을 move_base로 전송합니다.
   - gps_waypoint_continuous1: 하나의 컨트롤러를 사용하여 웨이포인트 간 지속적인 내비게이션을 수행합니다.
   - gps_waypoint_continuous2: 다른 별도의 컨트롤러를 사용하여 웨이포인트 간 지속적인 내비게이션을 수행합니다.
   - collect_gps_waypoint: 사용자가 로봇을 주행하며 자신만의 웨이포인트를 수집할 수 있도록 합니다.
   - calibrate_heading: 시작 시 로봇의 헤딩을 설정하고, 열악한 자기계 데이터 문제를 해결합니다.
   - plot_gps_waypoints: 그래프 작성 목적으로 GPS의 원시 데이터를 저장합니다.
   - gps_waypoint_mapping: Mandala Robotics의 3D 매핑 소프트웨어와 웨이포인트 내비게이션을 결합하여 3D 매핑을 수행합니다.

## 위도 경도를 로컬 오도메트리 메시지로 변환
만약 GPS 센서의 /navsat/fix 토픽을 로컬 좌표 프레임인 /navsat/odom 토픽으로 변환하려면, 다음 두 패키지를 직접 사용하세요:
- Geonav Transform <[코드](https://github.com/bsb808/geonav_transform)> <[ROS 위키](http://wiki.ros.org/geonav_transform)>
- Geographic Info <[코드](https://github.com/ros-geographic-info/geographic_info)> <[ROS 위키](http://wiki.ros.org/geographic_info)> <[웹사이트](http://library.isr.ist.utl.pt/docs/roswiki/navsat_odometry.html)> <[ROS 위키2](http://wiki.ros.org/navsat_odometry)>
- Lattitude, Longitude & Altitude to Pose Stamped <[코드](https://github.com/arpg/ROS-UTM-LLA)>
- Eagle Eye for GPS & IMU Fused Odometry <[코드](https://github.com/MapIV/eagleye)> <[유튜브](https://www.youtube.com/watch?v=u8Nan38BkDw)>
- IMU GPS Localization: Using EKF to fuse IMU and GPS data <[코드](https://github.com/ydsf16/imu_gps_localization)>


## 패키지의 상세한 이해
- [ROS Extra Class #2: How to use GPS to do autonomous robot navigation?](https://www.youtube.com/watch?v=cmOplaq8cHc)

## ROS에서 활용되는 GPS 관련 드라이버들
- GPSD <[code](https://github.com/ros-drivers/gps_umd)> <[ROS Tutorial](https://wiki.ros.org/gpsd_client/Tutorials/)>

## 관련 이슈
- [로봇 로컬라이제이션 navsat transform 노드가 발행되지 않을때](https://answers.ros.org/question/332905/robot_localization-navsat-transform-node-does-not-publish/)
- [오도메트리 없이 IMU와 GPS 융합을 수행하는 로봇 로컬라이제이션 패키지.](https://answers.ros.org/question/236588/imu-and-gps-fusion-without-odom-robot_localization/)
- [로봇 로컬라이제이션 패키지를 활용하여 IMU와 GPS 데이터를 결합하는 방법](https://answers.ros.org/question/200071/how-to-fuse-imu-gps-using-robot_localization/)
- [GPS navigation with mobile robot](https://question2738.rssing.com/chan-42656520/all_p5.html)

## ROS 로봇과 관련된 구현 예시
- Robot Localization using GPS <[website](https://wiki.nps.edu/display/RC/Localization+using+GPS%2C+IMU+and+robot_localization)>

## 사용자 인터페이스 데모
### Mapviz package
우리는 경로와 좌표를 시각화하기 위해 mapviz 패키지를 사용했습니다. 

#### 바이너리 설치:
```
sudo apt-get install ros-melodic-mapviz \
                       ros-melodic-mapviz-plugins \
                       ros-melodic-tile-map \
                       ros-melodic-multires-image		       
```
#### 소스 설치:
```
cd catkin_ws/src
git clone https://github.com/swri-robotics/mapviz.git
rosdep install --from-paths src --ignore-src
catkin_make
```
이전에 사용하던 구성 파일을 모두 삭제해주세요. 패널 내 플러그인의 순서가 중요합니다. Mapviz는 플러그인 패널에 나열된 순서대로 플러그인을 표시합니다. 만약 navsat 플러그인이 가장 먼저 나열되어 있다면, 그것을 먼저 표시하고 그 위에 tile_map 플러그인이 표시될 것입니다. 따라서 위치 정보를 볼 수 없을 수 있습니다.
```
sudo rm ~/.mapviz_config
```
하나의 터미널에서 다음을 실행하세요:
```
roscore
```
다른 터미널에서 `mapviz` 파일을 실행하세요. `mapviz.launch` 파일은 다음과 같아야 합니다:
```
<launch>

  <node pkg="mapviz" type="mapviz" name="mapviz"></node>

  <node pkg="swri_transform_util" type="initialize_origin.py" name="initialize_origin" >
    <param name="local_xy_frame" value="/map"/>
    <param name="local_xy_origin" value="auto"/>
    <!--<param name="local_xy_origin" value="swri"/>-->
    <rosparam param="local_xy_origins">
      [{ name: swri,
         latitude: 29.45196669,
         longitude: -98.61370577,
         altitude: 233.719,
         heading: 0.0},

       { name: back_40,
         latitude: 29.447507,
         longitude: -98.629367,
         altitude: 200.0,
         heading: 0.0}]
    </rosparam>
    <remap from="fix" to="/navsat/fix"/>
  </node>

  <node pkg="tf" type="static_transform_publisher" name="swri_transform" args="0 0 0 0 0 0 /map /origin 100"  />

</launch>
```
`mapviz.launch` 파일을 실행하려면 다음 명령을 사용하세요:
```
cd catkin_ws
source devel/setup.bash
roslaunch mapviz mapviz.launch
```
패널에서 첫 번째 상자 (고정 프레임 "Map" 및 대상 프레임 "None")는 그대로 두고, tile_map을 추가한 다음 navsat 플러그인을 추가하세요 (/navsat/fix 토픽 선택). 이것을 순차적으로 수행하세요.

#### Mapviz 테스트:
이제 `/navsat/fix` 토픽에 GPS를 게시할 샘플 bag 파일이 필요합니다. [여기](https://advdataset2019.wixsite.com/urbanloco/hong-kong)에서 rosbag을 다운로드하세요. 다른 터미널에서 rosbag을 실행하세요:
```
rosbag play CA-20190828184706_blur_align.bag
```
<p align="center">
    <img src="assets/mapviz_satellite.gif", width="800">
</p>

### Rviz 위성 패키지
#### 소스 설치 방법:
```
cd catkin_ws/src
git clone https://github.com/nobleo/rviz_satellite.git
catkin_make
```
`rviz.launch` 파일을 실행하려면 다음 명령을 사용하세요:
```
cd catkin_ws
source devel/setup.bash
roslaunch rviz_satellite demo.launch
```
#### Rviz 위성 테스트:
`/navsat/fix` 토픽에 GPS를 게시할 샘플 bag 파일이 필요합니다. [여기](https://advdataset2019.wixsite.com/urbanloco/hong-kong)에서 rosbag을 다운로드하세요. 다른 터미널에서 rosbag을 실행하세요:
```
rosbag play CA-20190828184706_blur_align.bag
```
<p align="center">
    <img src="assets/rviz_satellite.gif", width="800">
</p>

### Rosboard 패키지
#### 설치
#### 소스 설치:
```
cd catkin_ws/src
git clone https://github.com/dheera/rosboard.git
catkin_make
```
`rviz.launch` 파일을 실행하려면 다음 명령을 사용하세요:
```
cd catkin_ws
source devel/setup.bash
rosrun rosboard rosboard_node
```
#### Rosboard 테스트:
`/navsat/fix` 토픽에 GPS를 게시할 샘플 bag 파일이 필요합니다. [여기](https://advdataset2019.wixsite.com/urbanloco/hong-kong)에서 rosbag을 다운로드하세요. 다른 터미널에서 rosbag을 실행하세요:
```
rosbag play CA-20190828184706_blur_align.bag
```
로컬 브라우저에서 http://localhost:8888 주소로 이동하십시오 (로봇은 http://your-robot-ip:8888/). 왼쪽 상단 메뉴에서 시각화하려는 토픽을 추가하세요. 이렇게 하면 토픽을 시각화할 수 있어야 합니다.
<p align="center">
    <img src="assets/gps_nav.gif", width="800">
</p>


# 감사의 말
이 저장소를 만들면서 도움을 받은 웹사이트들에게 감사의 인사를 전합니다.
  - https://github.com/nickcharron
  - https://github.com/clearpathrobotics
  - https://github.com/swri-robotics
  - https://github.com/danielsnider/follow_waypoints
  - https://github.com/ros-geographic-info
  - https://github.com/nobleo/rviz_satellite
  - https://github.com/dheera/rosboard


