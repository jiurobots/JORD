/*
 *                   江城子 . 程序员之歌
 * 
 *               十年生死两茫茫，写程序，到天亮。
 *                   千行代码，Bug何处藏。
 *               纵使上线又怎样，朝令改，夕断肠。
 * 
 *               领导每天新想法，天天改，日日忙。
 *                   相顾无言，惟有泪千行。
 *               每晚灯火阑珊处，夜难寐，加班狂。
 * 
 * 
 * @Author: Glory Huang
 * @Date: 2021-10-08 09:41:35
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-10-15 15:59:34
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/StdVector>
using namespace std;

class M2DP {
public:
    // 使用宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW，用于在类中重载 operator new，保证 Eigen 对象内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 构造函数，接受一个 pcl::PointCloud<pcl::PointXYZ>::Ptr 类型的指针作为参数
    M2DP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // 默认析构函数
    ~M2DP() = default;
    // 获取成员变量 m2dp_result 的值，返回类型是 Eigen::Matrix<double, 1, 192>
    Eigen::Matrix<double, 1, 192> get_m2dp_result()
    {
        return m2dp_result;
    }
    // 获取成员变量 A 的值，返回类型是 Eigen::Matrix<double, 64, 128>
    Eigen::Matrix<double, 64, 128> get_m2dp_A()
    {
        return A;
    }

private :
    // key parameter
    // 在这里定义了一些关键参数，下面是它们的初始值

    // 把点投影到二维平面后，将平面划分为l*t个bin格，对于每个bin，简单计算其中的点的数量，获得一个lt*1的签名向量
    // number of bins in theta, the 't' in paper
    int numT = 16;  // sector数量（列数）
    // number of bins in rho, the 'l' in paper
    int numR = 8;  // ring数量（行数）

    // 通过使用p个不同的方位角和q个不同的俯仰角来生成p*q个不同的二维平面
    // number of azimuth angles, the 'p' in paper
    int numP = 4;  // p个不同的方位角
    // number of elevation angles, the 'q' in paper
    int numQ = 16;  // q个不同的俯仰角

    // input pointcloud
    // 输入点云，使用 pcl::PointCloud<pcl::PointXYZ>::Ptr 类型的智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered{new pcl::PointCloud<pcl::PointXYZ>()};
    // 存储 PCA 后的点云数据，类型是 Eigen::MatrixXd
    Eigen::MatrixXd cloud_pca;
    // output m2dp result
    // 存储 M2DP 的结果，类型是 Eigen::Matrix<double, 1, 192>
    Eigen::Matrix<double, 1, 192> m2dp_result;

    vector<double> azimuthList;  // 存储方位角列表
    vector<double> elevationList;  // 存储俯仰角列表
    double maxRho = 0;  // 最大的 rho 值
    // 存储 Signature Matrix A，类型是 Eigen::Matrix<double, 64, 128>
    Eigen::Matrix<double, 64, 128> A;

    // main function, get the signature matrix A
    // 主要函数，获取 Signature Matrix A 的值
    Eigen::Matrix<double, 64, 128> GetSignatureMatrix();

    // 一些辅助函数，用于坐标转换等操作
    void cart2sph(double &azm, double &elv, double &r, Eigen::Vector3d vecN);
    void sph2cart(double azm, double elv, double r, Eigen::Matrix<double, 1, 3> &vecN);
    void cart2pol(double x, double y, Eigen::Vector2d &vecN);
    void pol2cart(double rho, double phi, Eigen::Vector2d &vecN);

    // 二维直方图函数，计算输入点的直方图
    void histogram2d(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points, vector<double> thetaList, vector<double> rhoList, Eigen::MatrixXd &hist);
};