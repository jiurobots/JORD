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
#include <boost/filesystem.hpp>
#include <mutex>
#include "KDTreeVectorOfVectorsAdaptor.h"

#include <math.h>
#include <opencv2/opencv.hpp> 
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>
namespace fs = boost::filesystem;
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;
using point_type = std::array<double, 640>;
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

void getFiles(std::string path, std::vector<std::string>& files)
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



int test_ESF(int argc, const char *const *argv)
{
    //rosbag::Bag bag;
    //bag.open(argv[1]);
    std::vector<std::string> files;
    std::string filePath = argv[1];
    
    KDTree<point_type, 640> kdtree;
    std::vector<point_type> history_points;
    std::vector<point_type> result;
    std::vector<size_t> result_index;
    std::queue<point_type> q;
    //输出所有结果
    static std::once_flag flag;
    std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("./loop_esf.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    std::ofstream loop_file;
    loop_file.open("./loop_esf.txt", std::ios::app);
    getFiles(filePath, files);
	int size = files.size();
    // for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("velodyne_points")))
    
    for (int i = 0; i < size; i ++){
    //for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("/points_raw")))
    // for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("u2360")))
   // {
        //auto msg = m.instantiate<sensor_msgs::PointCloud2>();
       // if (msg != nullptr)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(files[i], *raw_cloud);
           // pcl::fromROSMsg(*msg, *raw_cloud);
            pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
            pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
            esf.setInputCloud(raw_cloud);
            esf.compute(*esf_descriptor);
            //将esf_descriptor转换为y方向的vector
            std::vector<float> esf_descriptor_y;
            for (int i = 0; i < 640; i++)
            {
                esf_descriptor_y.push_back(esf_descriptor->points[0].histogram[i]);
            }
            //将esf_descriptor_x转换为point_type
            point_type point;
            for (int i = 0; i < 640; i++)
            {
                point[i] = esf_descriptor_y[i];
            }
            q.push(point);
            history_points.push_back(point);
            if(q.size() > 50)
            {
                kdtree.insert(q.front());
                q.pop();
            }
            kdtree.search_knn(point,1,result,result_index);
            double dist = 0;
            //计算两个向量的L1距离
            point_type history = history_points[result_index[0]];
            for(int i = 0; i < 640; i++)
            {
                dist += fabs(point[i] - history[i]);
            }
            std::cout << "dist: " << dist << std::endl;
            loop_file << i << "," << -1 << "," << dist<< std::endl;
            dist = 0;           
           // i++;
            //std::cout << esf_descriptor->points[0] << std::endl;  //存储640个特征点 
            //pcl::io::savePCDFileASCII("/home/jlurobot/comparecode/ESF/pointcloud.pcd", *esf_descriptor);
         
        }
    }
 //   loop_file.close();
  //  bag.close();
    return 0;
}