// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//std lib
#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

//ros
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//opencv
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//local library
#include "iscGenerationClass.h"
//#include <iscloam/LoopInfo.h>  // 这个头文件没找到

ros::Publisher isc_pub;
ros::Publisher loop_info_pub;


std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;  // 点云队列
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;        // 里程计队列
std::mutex mutex_lock;

ISCGenerationClass iscGeneration;  // 回环过程实例化对象

double scan_period= 0.1;  // 扫描周期 10Hz

//receive odomtry
// 接收里程计数据
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();

}

//receive all point cloud
// 接收所有点云数据
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lock.lock();
    pointCloudBuf.push(msg);
    mutex_lock.unlock();
}

// 回环检测主要过程
void loop_closure_detection(){
    while(1){
        if(!pointCloudBuf.empty() && !odometryBuf.empty()){
            //align time stamp
            // 对齐时间戳
            mutex_lock.lock();
            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * scan_period){
                ROS_WARN("isc_generation: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",odometryBuf.front()->header.stamp.toSec(),pointCloudBuf.front()->header.stamp.toSec()); 
                odometryBuf.pop();  
                mutex_lock.unlock();
                continue;
            }

            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec() - 0.5 * scan_period){
                ROS_WARN("isc_generation: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",odometryBuf.front()->header.stamp.toSec(),pointCloudBuf.front()->header.stamp.toSec()); 
                pointCloudBuf.pop();  
                mutex_lock.unlock();
                continue;
            }

            // 处理所有输入数据，转换为对应格式
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);  // 转化输入的点云数据为PCL格式
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;  // 记录起始时间
            Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();  // 里程计信息转化为对应格式
            odom_in.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            odom_in.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            odometryBuf.pop();
            pointCloudBuf.pop();  
            mutex_lock.unlock();

            // 调用函数进行回环检测，相互调用实现回环检测过程
            iscGeneration.loopDetection(pointcloud_in, odom_in);

            // 输出isc图像topic
            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id  = "velodyne"; 
            out_msg.header.stamp  = pointcloud_time; 
            out_msg.encoding = sensor_msgs::image_encodings::RGB8; 
            out_msg.image    = iscGeneration.getLastISCRGB(); 
            isc_pub.publish(out_msg.toImageMsg());
            
            // 输出回环topic
            // iscloam::LoopInfo loop;  // 用的是那个没找到的头文件
            // loop.header.stamp = pointcloud_time;
            // loop.header.frame_id = "velodyne";
            // loop.current_id = iscGeneration.current_frame_id;
            // for(int i=0;i<(int)iscGeneration.matched_frame_id.size();i++){
            //     loop.matched_id.push_back(iscGeneration.matched_frame_id[i]);
            // }
            // loop_info_pub.publish(loop);

        }
        //sleep 2 ms every time
        //每次休眠 2ms
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }//end of while(1)
}



int ISC_main(int argc, char **argv)
{
    // ROS 初始化
    ros::init(argc, argv, "isc_gen");

    ros::NodeHandle nh("~");

    //rosnode init
    // 节点初始化，接收对应数据
    ros::Subscriber velodyne_sub= nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 10, pointCloudCallback);
    ros::Subscriber odom_sub= nh.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);

    // 待输出的数据信息，分别为回环信息和isc描述子信息
    // loop_info_pub = nh.advertise<iscloam::LoopInfo>("/loop_closure", 100);
    isc_pub = nh.advertise<sensor_msgs::Image>("/isc", 100);

    //read parameter
    // 读入参数
    int sector_width = 60;
    int ring_height = 60;
    double max_dis= 40.0;

    // 参数赋值
    nh.getParam("/sector_width", sector_width); 
    nh.getParam("/ring_height", ring_height); 
    //nh.getParam("/max_dis", max_dis); 
    nh.getParam("/scan_period", scan_period);  

    //init ISC
    // 回环过程初始化
    iscGeneration.init_param(ring_height,sector_width,max_dis);

    //open thread
    // 主过程
    std::thread loop_closure_detection_process{loop_closure_detection};

    ros::spin();

  return 0;
}
