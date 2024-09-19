#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include "m2dp.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <mutex>
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <math.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp> 
using point_type = std::array<double, 192>;
using SCPointType = pcl::PointXYZ; 

namespace fs = boost::filesystem;


void getfiles(std::string path, std::vector<std::string>& files)
{
    fs::directory_iterator end_iter;
    for (fs::directory_iterator iter(path); iter != end_iter; ++iter)
    {
        if (fs::is_regular_file(iter->status()))
        {
            files.push_back(iter->path().string());
        }
    }
}



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
            tree = new KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L2>(DIM, values, 10);
            last_count = values.size();
        }
    }

    size_t last_count = 0;
    KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L2> *tree = nullptr;
    std::vector<Point> values;
};

int main(int argc, const char *const *argv)
{
   // rosbag::Bag bag;
   // bag.open(argv[1]);
    std::vector<std::string> files;
    std::string filepath = argv[1];
    getfiles(filepath,files);
    int size = files.size();
    KDTree<point_type,192> kdtree;
    std::vector<point_type> history_points;
    std::vector<point_type> result;
    std::vector<size_t> result_index;
    std::queue<point_type> q;
    //输出所有结果
    static std::once_flag flag;
        std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("./loop.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    std::ofstream loop_file;
    loop_file.open("./loop_m2dp.txt", std::ios::app);
    for (int i = 0; i<size;i++){
        pcl::PointCloud<SCPointType>::Ptr cloud(new pcl::PointCloud<SCPointType>);
        pcl::io::loadPCDFile(files[i], *cloud);
        M2DP m2dp(cloud);
        Eigen::Matrix<double, 1, 192> desM2dp;
        desM2dp = m2dp.get_m2dp_result();
        point_type A_m2dp_point;
        for(int i = 0; i < 192; i++)
        {
            A_m2dp_point[i] = desM2dp(0,i);
        }
        q.push(A_m2dp_point);
        history_points.push_back(A_m2dp_point);
        if(q.size() > 50)
        {
            kdtree.insert(q.front());
            q.pop();
        }
          kdtree.search_knn(A_m2dp_point,1,result,result_index);
            double dist = 0;
            //计算两个向量的L2距离
            point_type history = history_points[result_index[0]];
            for(int i = 0; i < 192; i++)
            {
                dist += pow(A_m2dp_point[i] - history[i],2);
            }
            dist = sqrt(dist);
            std::cout << "dist: " << dist << std::endl;
            loop_file << i << "," << "-1" << "," << dist << std::endl;
            dist = 0;
            


    }
    loop_file.close();
    return 0;
}
