
# JORD


 <img src="https://img520.com/HFd7BP.jpg" width="1000" />


<p align="center">
  <img src="https://github.com/zhouwei1995/Gift/blob/main/1.gif" width="200" />
  <img src="https://github.com/zhouwei1995/Gift/blob/main/2.gif" width="200" />

  
  <img src="https://github.com/zhouwei1995/Gift/blob/main/3.gif" width="200" />
  <img src="https://github.com/zhouwei1995/Gift/blob/main/4.gif" width="200" />
</p>



## Recording Platform

 Sensor  | Model  | Rate | Specifications 
 ---- | ----- | ------  | ------
 LiDAR  | Velodyne VLP-16 | 10 | 16 Channels
 IMU  | EG320N | 125  | 6DoF
GPS  | NovAtel Npos220s | 10 | Dual Antenna

 <img src="https://img520.com/XkIJA3.jpg" width="800" />

 ## Recording Collection
<img src="https://github.com/zhouwei1995/Gift/blob/main/5.gif" width="400" />


## Dataset Download  
The JORD can be download as follows:
<p>
Link: https://pan.baidu.com/s/1JuZ_n7VYKpvC6FrufZmS2A?pwd=1h3u Code: 1h3u 


## Code
### 1. Place Recognition Task


#### Dependencies
The following has been verified to be compatible, although other configurations may work too:
* Ubuntu 18.04
* ROS Noetic(roscpp, rospy, std_msgs, sensor_msgs, pcl_ros)
* C++ 17
* CMake >= 3.0.2
* OpenCV >= 4.2.0
* PCL >= 1.7
* Eigen3 >= 3.3.7
* Boost >= 1.71.0
```
sudo apt install liboost-dev libpcl-dev libeigen3-dev
```
#### Compiling

```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/jiurobots/JORD.git
catkin make
```
#### Execution

```
./jm_lcd -d JORD -s 01 -n SC
```
* -d Indicates the data set name, such as KITTI,JORD, etc
* -s Indicates a sequence of data sets, such as 01,0203, 04, etc
* -n Indicates the algorithm name, such as SC,CSSC,ISC,NDD, etc

### 2. LiDAR SLAM Task


### 1) Run the launch file:
For example:
```
roslaunch lego_loam run.launch  
```

### 2) Play the JORD file:
The./pkg tool is used to play JORD data.


#### Dependencies
* Ubuntu 18.04
* ROS Noetic
* C++ 17
* PCL >= 1.7
* CMake >= 3.0.2
* ninja >= 1.10.0
* libjpe

#### Compiling
```shell
sudo apt update
sudo apt install git build-essential libpcl-dev libjpeg9-dev cmake ninja
cd pkg2.1
mkdir build 
cd build
cmake -G Ninja ..
ninja
```
#### Execution
```shell
./pkg2 play < dataset file name > [options]
```
Optional options:
|Command Line|Means|Demo| Comment |
| :--: | :--: | :--: | :--: |
| --speed | playback speed| --speed 0.5 | Too fast and you might not be able to decode it |
| --point-type | Point cloud data type | --point-type XYZIRT | available XYZ， XYZI， XYZIRT， XYZRGB， default XYZI|
|--rename | change topic | --rename /u2102 /velodyne_points | -

Other options can run: ./pkg2 play --help View 。

* Viewing packet Information
```shell
./pkg2 info < dataset file name >
```
## License 
All datasets and code on this page are copyright by us and published under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.



 
