

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "LidarIris.h"
#include "tictoc.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
//#include <opencv2/line_descriptor.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <mutex>
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <opencv2/opencv.hpp> 
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>



//xiu 过滤离群点
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>

// //xiu 去中心化
// #include <pcl/common/centroid.h>
// //xiu 平移
// #include <pcl/common/transforms.h>
// //xiu 滤除地面点
// #include <pcl/filters/passthrough.h>
//yawl 平移
// #define BOOST_TYPEOF_EMULATION
// #include <pcl/registration/icp.h>

using namespace Eigen;

using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;
using point_type = std::array<double, 80>;
template<typename Point, size_t DIM>
struct KDTree
{
public:
    void insert(const Point &point)
    {
       values.push_back(point);
       rebuild_tree();
    }
    void search_knn(const Point &point, int k, std::vector<Point> &result, std::vector<size_t> &result_index) const 
    {
        result_index.resize(k);
        result.resize(k);
        if(tree == nullptr){
            result_index.clear();
            result.clear();
            return;
        }
        std::vector<double> out_dists_sqr( k );
        nanoflann::KNNResultSet<double> knnsearch_result( k );//预定义
        knnsearch_result.init( &result_index[0], &out_dists_sqr[0] );//初始化
        tree->index->findNeighbors( knnsearch_result, point.data() /* query */, nanoflann::SearchParams(10) ); 
    }

    void rebuild_tree()
    {
        if(values.size() > last_count + 50)
        {
            if(tree != nullptr)
                delete tree;
            tree = new KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L1>(DIM, values, 10);
            last_count = values.size();
        }
    }

    size_t last_count = 0;
    KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L1> *tree = nullptr;
    std::vector<Point> values;
};



int main(int argc, const char *const *argv)
{
    rosbag::Bag bag;
    bag.open(argv[1]);

    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterScancontext;
    downSizeFilterScancontext.setLeafSize(0.4, 0.4, 0.4);
    LidarIris iris(4, 18, 1.6, 0.75, 0);


    KDTree<point_type, 80> kdtree;
    std::vector< LidarIris::FeatureDesc> history_points;
    std::vector<point_type> result;
    std::vector<size_t> result_index;
    std::queue<point_type> q;
    //输出所有结果
    static std::once_flag flag;
    std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("../ourss.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    std::ofstream loop_file;
    loop_file.open("../ourss.txt", std::ios::app);
    int i = 0;
    // for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("velodyne_points")))
    for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("/points_raw")))
    // for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("u2360")))
    {
        auto msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *raw_cloud);

            iris.filterpoint(raw_cloud);  //滤除离散点
            // // 去中心化
            iris.decentralization(raw_cloud);  //去中心化

            downSizeFilterScancontext.setInputCloud(raw_cloud);
            downSizeFilterScancontext.filter(*raw_cloud);

            cv::Mat1b li1 = LidarIris::GetIris(*raw_cloud);
            LidarIris::FeatureDesc fd1 = iris.GetFeature(li1);


            point_type point;
            for (int i = 0; i < 80; i++)
            {   
                double s = 0;
                for(int j = 0; j < 360 ; j++)
                {
                    s += li1.at<uint8_t>(i,j);
                }
                point[i] = s/360;
            }
            q.push(point);
            history_points.push_back(fd1);
            if(q.size() > 50)
            {
                kdtree.insert(q.front());
                q.pop();
            }
            if(i >= 50){
                kdtree.search_knn(point,10,result,result_index);
                //计算两个向量的L1距离
                // point_type history = history_points[result_index[0]];
                // LidarIris::FeatureDesc fd2 = history_points[result_index[0]];

                double min_dis = 1;
                int cur = -1;
                int bias;
                for(auto ind : result_index){
                    auto dis = iris.Compare(fd1, history_points[ind], &bias);
                    if(dis < min_dis){
                        min_dis = dis;
                        cur = ind;
                    }
                }
                // auto dis = iris.Compare(fd1, fd2, &bias);
                // std::cout << "dist: " << dist << std::endl;
                loop_file << i << "," << cur << "," << min_dis<< std::endl;
            }
                      
            i++;
        }
    }
    loop_file.close();
    bag.close();
    return 0;
}
