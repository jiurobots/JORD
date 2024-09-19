// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _ISC_GENERATION_CLASS_H_
#define _ISC_GENERATION_CLASS_H_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>


//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ros
#include <ros/ros.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


//IF TRAVELLED DISTANCE IS LESS THAN THIS VALUE, SKIP FOR PLACE RECOGNTION
// 如果移动距离小于这个值，跳过地点识别？？？
#define SKIP_NEIBOUR_DISTANCE 20.0
//how much error will odom generate per frame 
// 里程计每一帧会产生多少误差
#define INFLATION_COVARIANCE 0.03

//define threshold for loop closure detection
// 定义回环检测的阈值
#define GEOMETRY_THRESHOLD 0.01  // 几何阈值（原0.67）
#define INTENSITY_THRESHOLD 0.01  // 强度阈值（原0.91）

// 将 cv::Mat 类型重命名为 ISCDescriptor ，用于存储ISC描述符的数据。
typedef cv::Mat ISCDescriptor; 

class ISCGenerationClass
{
    public:
        // 默认构造函数
        ISCGenerationClass();

        // 初始化函数，用于初始化回环的一些参数
        void init_param(int rings_in, int sectors_in, double max_dis_in);
        // 重构
        void init_param(int rings_in, int sectors_in, double max_dis_in, int intensityValueType);

        // 获取最近的单通道 ISC 描述符
        ISCDescriptor getLastISCMONO(void);  // ？？？好像没用到

        // 获得当前帧的isc信息，用于发布topic
        // 获取最近的 RGB ISC 描述符
        ISCDescriptor getLastISCRGB(void);
        // 回环检测过程入口，该方法接收一个点云数据current_pc和里程计数据odom，并检测是否存在环路。
        void loopDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc, Eigen::Isometry3d& odom);

        // 新增detectLoopClosureIDAndDis函数
        void makeAndSaveISC(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc);  // 新增
        // 新增detectLoopClosureIDAndDis函数
        std::pair<int, float> detectLoopClosureIDAndDis();  // 新增
        
        int current_frame_id;  // 当前帧ID
        std::vector<int> matched_frame_id;  // 匹配帧的 ID 数组
    private:
        int rings = 20;          // 二维数组行数 ring
        int sectors = 90;        // 二维数组列数 sector
        double ring_step = 0.0;    // 每一行的宽度 = 最大距离/行数
        double sector_step = 0.0;  // 每一列的宽度 = 360°/列数
        double max_dis = 60;     // 最大距离

        // 定义一个数组，值类型为Vec3b，是个三元组，表示图像像素的颜色信息
        std::vector<cv::Vec3b> color_projection;  // 颜色投影

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud;  // 当前点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr test_pc;  // 测试点云

        // 存储历史帧信息？？？
        std::vector<Eigen::Vector3d> pos_arr;     // 位姿数组
        std::vector<double> travel_distance_arr;  // 移动距离数组
        std::vector<ISCDescriptor> isc_arr;       // ISC 描述符数组

        // 初始化颜色信息
        void init_color(void);
        // 输出各项参数值
        void print_param(void);
        // 判断两帧的 ISC 描述符是否是回环
        bool is_loop_pair(ISCDescriptor& desc1, ISCDescriptor& desc2, double& geo_score, double& inten_score);
        // 计算过滤后点云的 ISC 描述符
        ISCDescriptor calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud);
        // 计算几何距离
        double calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
        // 计算强度距离
        double calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
        // 地面滤波，在z轴上过滤点云，将地面点从点云中移除。
        void ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out);

        // ========================
        int intensityValueType = 0;  // 0 for [0, 1]     1 for [1, 255]
};




#endif // _ISC_GENERATION_CLASS_H_

