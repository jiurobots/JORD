#pragma once

#include <vector>
#include <map>
#include <set>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

// PCL自定义Point类型
// 1. 定义Point结构体，该自定义点类型是基于PCL中的pcl::PointXYZ类型进行扩展的
struct PointXYZSCA
{
    PCL_ADD_POINT4D;  // 使用 PCL_ADD_POINT4D 宏，它将前三个字段（x、y、z）添加到pcl::PointXYZ的定义中，使其支持点的3D坐标。
    float scan_position;  // 自定义变量，扫描位置
    float curvature;  // 自定义变量，曲率
    float angle;  // 自定义变量，角度
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // 该宏确保该自定义点类型的内存对齐。
}EIGEN_ALIGN16;
// 2. 注册类型，注册点类型宏 (type, name, tag) (类型(不符合不会载入), 变量名, 文件中的变量名(不符合会报错))
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZSCA, 
    (float, x, x)(float, y, y)(float, z, z)(float, scan_position, scan_position)(float, curvature, curvature)(float, angle, angle))

typedef vector<vector<PointXYZSCA>> ScanEdgePoints;

namespace BoW3D
{
    #define EdgePointCloud pcl::PointCloud<PointXYZSCA>
    #define distXY(a) sqrt(a.x * a.x + a.y * a.y)  // 计算点 a 的平面距离，即点 a 在 x 和 y 方向上的距离。
    #define distOri2Pt(a) sqrt(a.x * a.x + a.y * a.y + a.z * a.z)  // 计算点 a 到原点的距离，即点 a 的欧氏距离。
    #define distPt2Pt(a, b) sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z))  // 计算点 a 和点 b 之间的距离，即两个点的欧氏距离。
    
    using std::atan2;
    using std::cos;
    using std::sin;
   
    class Frame;

    class LinK3D_Extractor
    {
        public:
            LinK3D_Extractor(int nScans_, float scanPeriod_, float minimumRange_, float distanceTh_, int matchTh_);

            ~LinK3D_Extractor(){}

            bool comp (int i, int j) 
            { 
                return cloudCurvature[i] < cloudCurvature[j]; 
            }

            // 移除范围过近的无效点，对距离小于阈值的点云进行滤除
            void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                        pcl::PointCloud<pcl::PointXYZ> &cloud_out);

            // 从输入的点云数据中提取边缘点，并将边缘点保存在 ScanEdgePoints 类型的容器 edgePoints 中
            void extractEdgePoint(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, ScanEdgePoints &edgePoints);

            // 将提取的边缘点根据其水平角度划分到不同的扇区中，然后将每个扇区中的边缘点存储在 ScanEdgePoints 类型的容器 sectorAreaCloud 中
            void divideArea(ScanEdgePoints &scanEdgePoints, ScanEdgePoints &sectorAreaCloud);

            // 求聚类簇到中心点的平均距离
            float computeClusterMean(vector<PointXYZSCA> &cluster);

            // 求聚类簇X、Y的平均值
            void computeXYMean(vector<PointXYZSCA> &cluster, pair<float, float> &xyMeans);

            // 将扇区中的边缘点进行聚类操作，将满足一定条件的边缘点聚合成一个簇，并将这些簇存储在 ScanEdgePoints 类型的容器 clusters 中
            void getCluster(const ScanEdgePoints &sectorAreaCloud, ScanEdgePoints &clusters);

            // 计算从一个点 ptFrom 到另一个点 ptTo 的方向向量，并将结果存储在 Eigen::Vector2f 类型的引用 direction 中
            void computeDirection(pcl::PointXYZI ptFrom,
                                  pcl::PointXYZI ptTo,
                                  Eigen::Vector2f &direction);

            // 从输入的聚类 clusters 中提取出关键点，并将有效的聚类存储到 validCluster 中，然后返回提取的关键点（有效簇的质心） keyPoints
            vector<pcl::PointXYZI> getMeanKeyPoint(const ScanEdgePoints &clusters,
                                                   ScanEdgePoints &validCluster);
            
            // 将输入的浮点数 in 四舍五入到小数点后一位
            float fRound(float in);

            // 计算关键点 keyPoints 的描述符，并将生成的描述符存储到 descriptors 中
            void getDescriptors(const vector<pcl::PointXYZI> &keyPoints, cv::Mat &descriptors);
            
            void match(vector<pcl::PointXYZI> &curAggregationKeyPt,
                       vector<pcl::PointXYZI> &toBeMatchedKeyPt,
                       cv::Mat &curDescriptors,
                       cv::Mat &toBeMatchedDescriptors,
                       vector<pair<int, int>> &vMatchedIndex);


            void filterLowCurv(ScanEdgePoints &clusters, ScanEdgePoints &filtered);

            void findEdgeKeypointMatch(ScanEdgePoints &filtered1,
                                       ScanEdgePoints &filtered2,
                                       vector<pair<int, int>> &vMatched,
                                       vector<pair<PointXYZSCA, PointXYZSCA>> &matchPoints);
            
            // 重载函数运算符 operator()。将传入的点云 pLaserCloudIn 通过 LinK3D_Extractor 进行一系列的处理，
            // 得到一些聚类的关键点 keyPoints、点云的描述符 descriptors 和边缘关键点 validCluster。
            void operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn,
                            vector<pcl::PointXYZI> &keyPoints,
                            cv::Mat &descriptors,
                            ScanEdgePoints &validCluster);

        private:
            int nScans;  // 激光雷达线束数量
            float scanPeriod;  // 激光雷达扫描周期，velodyne频率10Hz，周期0.1s
            float minimumRange;  // 过滤过近的点时使用的最小距离阈值（距离过近则被滤除）

            float distanceTh;  // 聚类时的距离阈值（如果当前边缘点满足一定条件，则将其加入当前簇中）
            int matchTh;  // 匹配时的阈值（相似性分数大于匹配阈值，认为匹配有效）
            int scanNumTh;  // 过滤扁平簇时的阈值（如果簇中不同扫描线位置的个数小于 scanNumTh ，说明该聚类太扁平，故过滤掉扁平的簇）
            int ptNumTh;  // 聚类时簇的最小限定数量（簇中点的数量太少则丢弃该簇）

            float cloudCurvature[400000];
    };
}
