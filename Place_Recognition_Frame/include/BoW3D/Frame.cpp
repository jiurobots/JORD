#include "Frame.h"
#include <thread>

namespace BoW3D
{
    // 这是一个类静态成员变量的定义和初始化，表示帧对象的全局ID计数器。它的类型是 long unsigned int，即无符号长整型。
    // 静态成员变量是类的所有对象共享的，因此它用于在所有帧对象中唯一地标识每个帧的ID。
    long unsigned int Frame::nNextId = 0;
   
    // Frame 类的构造函数实现了帧（Frame）对象的初始化。
    Frame::Frame(LinK3D_Extractor* pLink3dExtractor, pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn):mpLink3dExtractor(pLink3dExtractor)
    {
        // 将全局ID计数器 nNextId 的值赋给了当前帧对象的成员变量 mnId。
        // 然后，nNextId 的值会递增，以确保下一个帧对象的 mnId 与前一个帧对象的 mnId 是不同的。这样做的目的是为了给每个帧对象分配唯一的ID。
        mnId = nNextId++; 

        // 这是一个函数调用语句，调用了 mpLink3dExtractor 所指向的 LinK3D_Extractor 类对象的重载函数运算符 operator()。
        // LinK3D_Extractor 类中重载了运算符 operator()，使得其对象可以像函数一样被调用。
        // 这行代码的作用是将传入的点云 pLaserCloudIn 通过 LinK3D_Extractor 进行一系列的处理，
        // 得到一些聚类的关键点 mvAggregationKeypoints、点云的描述符 mDescriptors 和有效的边缘聚集关键点 mClusterEdgeKeypoints。
        (*mpLink3dExtractor)(pLaserCloudIn, mvAggregationKeypoints, mDescriptors, mClusterEdgeKeypoints);
    }

                
}