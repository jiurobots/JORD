#include <iostream>
#include "fstream"

#include "cssc/csscM.h"

#include <pcl/io/pcd_io.h>



int main(int argc, char ** argv){

    //demo for estimating similariyt between two LiDAR scans using CSSC descriptors.

    // 定义了两个智能指针 cloud1 和 cloud2，用于存储 LiDAR 点云数据。这里使用了自定义的 pcxyz 类型，可以推断出该类型用于存储包含XYZ坐标信息的点云数据。
    pcPtrxyz cloud1(new pcxyz), cloud2(new pcxyz);
    // 使用PCL库的pcl::io::loadPCDFile函数从文件加载LiDAR扫描的点云数据，加载的点云数据将存储在 cloud1 和 cloud2 中
    pcl::io::loadPCDFile("../data/64line-1.pcd", *cloud1);
    pcl::io::loadPCDFile("../data/64line-2.pcd", *cloud2);

    csscM cssc;  // 创建了一个CSSC对象，用于计算LiDAR点云之间的相似性
    // 调用CSSC对象的 calculateDistanceBtnPC 函数，传入两个点云数据 cloud1 和 cloud2，计算它们之间的相似性。
    // 该函数返回一个 std::pair<float, float> 类型的结果，其中first表示计算得到的相似性值。
    float similarity = cssc.calculateDistanceBtnPC(cloud1, cloud2).first;
    cout<<"similarity between two scans:  "<<similarity<<endl;  // 输出计算得到的相似性值

    return 0;
}
