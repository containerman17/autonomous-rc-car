# autonomous-rc-car
An Autonomous RC car based on ROS packages and using visual-slam based navigation on Jetson Nano board + Arduino Uno + (D435 + T265) cameras

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04 on NVidia Jetson Nano board.
ROS Melodic. [ROS Installation](https://github.com/JetsonHacksNano/installROS)

### 1.2. **librealsense2**
Follow [librealsense Installation](https://github.com/JetsonHacksNano/installLibrealsense).

### 1.3. **RealSense ROS Wrapper**
Follow [ROS-RealSense Installation](https://github.com/JetsonHacksNano/installRealSenseROS).

### 1.3. **Arduino IDE**
Follow [Arduino Installation](https://github.com/JetsonHacksNano/installArduinoIDE).

### 1.4. **Install rosserial for ROS-Arduino communication**
Follow [Arduino Installation](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

### 1.5. **Install RTAB-Map ROS**
```
sudo apt install ros-melodic-rtabmap ros-melodic-rtabmap-ros
```
### 1.6. **Install Robot-Localization**
```
sudo apt install ros-melodic-robot-localization
```
## 2. Build & Install
Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
rm -rf *
git clone https://github.com/mhaboali/autonomous-rc-car.git
cd ../
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
source ~/catkin_ws/devel/setup.bash
```

## 3. Usage Instructions

### Start both T265 and D435i cameras nodes

```bash
roslaunch realsense2_camera cameras.launch
```

This will stream all cameras sensors and publish on the appropriate ROS topics.

### Set Camera Controls Using Dynamic Reconfigure Params
The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<p align="center"><img src="https://user-images.githubusercontent.com/40540281/55330573-065d8600-549a-11e9-996a-5d193cbd9a93.PNG" /></p>

### About Frame ID
The wrapper publishes static transformations(TFs). The Frame Ids are divided into 3 groups:
- ROS convention frames: follow the format of <tf_prefix>\_<\_stream>"\_frame" for example: camera_depth_frame, camera_infra1_frame, etc.
- Original frame coordinate system: with the suffix of <\_optical_frame>. For example: camera_infra1_optical_frame. Check the device documentation for specific coordinate system for each stream.
- base_link: For example: camera_link. A reference frame for the device. In D400 series, it is the depth frame. In T265, the pose frame.

