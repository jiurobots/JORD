#pragma once

#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LinK3D_Extractor.h"


using namespace std;
using namespace Eigen;

namespace BoW3D
{
    class LinK3D_Extractor;

    class Frame
    {
        public:
            Frame();
            
            Frame(LinK3D_Extractor* pLink3dExtractor, pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn);
            
            ~Frame(){};
                        
        public: 
            static long unsigned int nNextId;

            long unsigned int mnId;  // 当前帧ID
            
            LinK3D_Extractor* mpLink3dExtractor;  // LinK3D_Extractor类对象

            ScanEdgePoints mClusterEdgeKeypoints;  // 边缘关键点（有效簇的质心）

            std::vector<pcl::PointXYZI> mvAggregationKeypoints;  // 有效的聚类
    
            cv::Mat mDescriptors;  // 当前帧的描述符       
    };

}

