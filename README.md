
# Off-Road LiDAR-Based SLAM: Dataset and Loop Closure Detection Framework


 <img src="https://github.com/jiurobots/JORD/blob/main/photos.jpg" width="1000" />

## Introduction
Simultaneous localization and mapping (SLAM) is a crucial element of autonomous driving technology. Presently, many SLAM methodologies based on light detection and ranging (LiDAR-based) are tailored for structured road environments. The highly irregular and unpredictable nature of off-road environments presents additional challenges for autonomous driving technology, but this environment is not sufficiently represented in the currently available datasets. To address this gap, we introduce the jlurobot off-road dataset (JORD), the first LiDAR-based SLAM dataset specifically designed for off-road environments. This dataset aims to advance research in autonomous driving technology under unstructured road conditions. Furthermore, we propose a novel two-stage loop closure detection (LCD) framework for dense forest off-road scenarios within the JORD. In the initial coarse matching stage, we extract tree trunk outlines via LiDAR point cloud segmentation and determine the centroid positions of the tree trunks. Subsequently, we create Delaunay triangle descriptor (DTD) and employ triangular graph matching to identify the candidate frame set. During the fine matching stage, we extract the maximum transformation scale information of the tree trunks and construct a maximum singular value descriptor (MSVD). Finally, we compute the similarity using the correlation coefficient. This framework not only provides guidance for research methodologies utilizing the JORD, but also represents a preliminary exploration into the application of graph techniques in LCD tasks. Extensive experiments validate the dataset's utility and the efficacy of our proposed method. 

<p align="center">
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/1.gif" width="200" />
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/2.gif" width="200" />

  
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/3.gif" width="200" />
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/4.gif" width="200" />
</p>



## Recording Platform

 Sensor  | Model  | Rate | Specifications 
 ---- | ----- | ------  | ------
 Mechanical LiDAR  | Velodyne VLP-16 | 10 | 16 Channels
 Solid-state LiDAR  | Livox HAP | 10  | 120×25
 IMU  | EG320N | 125  | 6DoF
GPS  | NovAtel Npos220s | 10 | Dual Antenna

 <img src="https://github.com/jiurobots/JORD/blob/main/img/sensors.jpg" width="800" />

 ## Recording Collection
<img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/5.gif" width="400" />


### Dataset Description  
 Sequence  | Path Length(m)  | Frame | Avg Speed(m/s) | Revisits/Total | Initial Frames | Loop Frames | Non-same Direction | Revisits (ratio) | Description
 ---- | ----- | ------  | ------  | ------  | ------  | ------  | ------  | ------  | ------  
 JORD01  | 374.4 | 3508 | 0.99 | 1018/3508 | 798-1868,534-683 | 2168-3080,3139-3243 | Y | (89%,913/1018) |Dense forest, Narrow and rough path 
 JORD02  | 872.3 | 8216 | 1.0 | 3360/8216 | 2634-2796,4152-7329 | 2934-3096,233-3914 | Y | (86%,2897/3360) |Dense forest, Narrow path, Tangled undergrowth 
 JORD03  | 654.2 | 5444 | 1.14 | 739/5444 | 0-960 | 4706-5444 | Y | (100%,739/739) |Sparse forest, Wide Muddy path
 JORD04  |  592.5 | 4120 |1.34 | 762/4210 | 1175-1899,869-954 | 2119-2775,3014-3118 | Y | (89%,913/1018) |Sparse forests,Wide path, Slopes
 JORD05  | 719.29 | 5157 | 1.30 | 77/5157 | 2365-2443 |  3343-3420 | Y | (100%,77/77) |Steep slopes, Rugge,mountains, Sparse forests 
 JORD06  | 1291.45 | 10388 | N | N | N | N |N | 0% |Wide lanes, Bends, Sparse forests
 JORD07  | 857.48 | 6621 | N | N | N | N | N | 0% |Extremely narrow path, Dense forests 
 JORD08  | 707.13 | 5693 | N | N | N | N | N | 0% |Rugged mountain path, Uphill, Sparse forest 
### Dataset Download  
The JORD can be download as follows:
<p>
链接: https://pan.baidu.com/s/1gr-3rPIEXNY0aFRnyaf1EQ?pwd=npc2 提取码: npc2 
 
### Folder Structure

<img src="https://github.com/jiurobots/JORD/blob/main/img/FileTree.jpg" width="200" />

### License 
All datasets and code on this page are copyright by us and published under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.



 
