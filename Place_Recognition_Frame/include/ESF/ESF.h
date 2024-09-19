#pragma once
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
// #include <opencv2/line_descriptor.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>
#include <mutex>

#include "../tools/nanoflann.hpp"
#include "../tools/KDTreeVectorOfVectorsAdaptor.h"

#include "../tools/tictoc.h"
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>

using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;
using point_type = std::array<double, 640>;
template <typename Point, size_t DIM>
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
        if (tree == nullptr)
        {
            result_index.clear();
            result.clear();
            return;
        }
        std::vector<double> out_dists_sqr(k);
        nanoflann::KNNResultSet<double> knnsearch_result(k);        // 预定义
        knnsearch_result.init(&result_index[0], &out_dists_sqr[0]); // 初始化
        tree->index->findNeighbors(knnsearch_result, point.data() /* query */, nanoflann::SearchParams(10));
    }

    void rebuild_tree()
    {
        if (values.size() > last_count + 50)
        {
            if (tree != nullptr)
                delete tree;
            tree = new KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L1>(DIM, values, 10);
            last_count = values.size();
        }
    }

    size_t last_count = 0;
    KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L1> *tree = nullptr;
    std::vector<Point> values;
};

