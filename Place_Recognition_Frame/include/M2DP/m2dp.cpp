/*
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│
 *  ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\ │`~ ││
 *  │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│
 *  ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││
 *  │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│
 *  ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  ││
 *  │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│
 *  ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││
 *  │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│
 *  │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │
 *  │      └───┴─────┴───────────────────────┴─────┴───┘          │
 *  └─────────────────────────────────────────────────────────────┘
 * 
 * @Author: Glory Huang
 * @Date: 2021-10-08 09:37:24
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-10-15 15:59:45
 * @Page: http://gloryhry.github.io/
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <iostream>
#include <M2DP/m2dp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <cmath>

using namespace std;

M2DP::M2DP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // this->cloud = *new pcl::PointCloud<pcl::PointXYZ>();

    // 将传入的点云数据 cloud 复制到类的成员变量 this->cloud 中
    pcl::copyPointCloud(*cloud, *this->cloud);
    // 使用 PCA 对点云进行主成分分析，并将结果存储在 this->cloud_filtered 中
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    pca.project(*cloud, *this->cloud_filtered);
    // 将 this->cloud_filtered 中的所有点的 z 坐标取反（乘以 -1），以改变点云的方向
    for (int i = 0; i < this->cloud_filtered->points.size();i++)
    {
        this->cloud_filtered->points[i].z = -this->cloud_filtered->points[i].z;
    }
    // 调整 cloud_pca 的大小（行数，列数）以容纳主成分分析后的点云数据
    cloud_pca.resize(this->cloud_filtered->points.size(), 3);
    // 将主成分分析后的点云数据存储到 cloud_pca 中，每个点的 (x, y, z) 坐标分别存储在对应的列中
    for (int i = 0; i < this->cloud_filtered->points.size(); i++)
    {
        cloud_pca(i, 0) = this->cloud_filtered->points[i].x;
        cloud_pca(i, 1) = this->cloud_filtered->points[i].y;
        cloud_pca(i, 2) = this->cloud_filtered->points[i].z;
    }
    // 创建一个 numP 大小的 vector<double>，并填充 azimuthList，该 vector 存储方位角列表
    azimuthList = *new vector<double>(numP);
    for (int i = 0; i < numP; i++)
    {
        azimuthList[i] = -M_PI_2 + i * M_PI / (numP - 1);
    }
    // 创建一个 numQ 大小的 vector<double>，并填充 elevationList，该 vector 存储俯仰角列表
    elevationList = *new vector<double>(numQ);
    for (int i = 0; i < numQ; i++)
    {
        elevationList[i] = i * M_PI_2 / (numQ - 1);
    }
    // get the farthest point distance
    // 获取点云中最远点的距离，存储在 maxRho 中
    for (auto pt : this->cloud_filtered->points)
    {
        double temp_rho = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (temp_rho > maxRho)
        {
            maxRho = temp_rho;
        }
    }
    // main function, get the signature matrix A
    // 调用 GetSignatureMatrix 函数获取 Signature Matrix A 的值，并将结果存储在 A 中
    A = GetSignatureMatrix();
    // Eigen::JacobiSVD<Eigen::Matrix<double, 64, 128>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // 对 A 进行奇异值分解，分解结果存储在 u 和 v 中
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd u = svd.matrixU();
    Eigen::MatrixXd v = svd.matrixV();
    // 提取 u 和 v 的第一列，存储在 u_temp 和 v_temp 中
    Eigen::Matrix<double, 1, 64> u_temp;
    Eigen::Matrix<double, 1, 128> v_temp;
    u_temp = u.col(0);
    v_temp = v.col(0);
    // 将 u_temp 和 v_temp 拼接成一个 1x192 的矩阵，存储在 m2dp_result 中
    m2dp_result << u_temp, v_temp;
}

Eigen::Matrix<double, 64, 128> M2DP::GetSignatureMatrix()
{
    // 创建存储 theta 列表的 vector，并填充 thetaList
    vector<double> thetaList(numT + 1);
    for (int i = 0; i <= numT; i++)
    {   // 在给定范围内（-π 到 π）均匀地分配 numT 个角度值，用于表示方位角的 bin 值
        thetaList[i] = -M_PI + i * 2 * M_PI / (numT);
    }
    // 创建存储 rho 列表的 vector，并填充 rhoList
    vector<double> rhoList(numR + 1);
    for (int i = 0; i <= numR; i++)
    {   // m = r*l^2，则 r = m / (l^2)   （主要是为了先求出论文中的r）
        rhoList[i] = i * sqrt(maxRho) / numR;  // 相当于 i * sqrt(r)
        rhoList[i] = rhoList[i] * rhoList[i];  // i^2 * r
    }
    // make sure all points in bins
    // 确保 rhoList 中的最后一个元素足够大，以保证所有点都在 bins 中
    rhoList[rhoList.size() - 1] = rhoList[rhoList.size() - 1] + 0.001;
    
    // 创建一个 64x128 的矩阵 result_A，用于存储 Signature Matrix A 的结果
    Eigen::Matrix<double, 64, 128> result_A;  // 列数64 = 4 * 16（numP * numQ），行数128 = 16 * 8（numT * numR）
    int index_A = 0;
    // loop on azimuth
    // 循环遍历 azimuthList（方位角列表）
    for (int i = 0; i < azimuthList.size();i++)
    {
        auto azm = azimuthList[i];
        // loop on elevation
        // 循环遍历 elevationList（俯仰角列表）
        for (int j = 0; j < elevationList.size();j++)
        {
            auto elv = elevationList[j];
            // normal vector vecN of the selected 2D plane
            // 计算选定 2D 平面的法向量 vecN
            Eigen::Matrix<double, 1, 3> vecN;  // 用于存储一个1x3维的向量，表示球坐标系中的法向量
            sph2cart(azm, elv, 1, vecN);
            // distance of vector [1,0,0] to the surface with normal vector vecN
            // 计算向量 [1,0,0] 到具有法向量 vecN 的平面的距离
            Eigen::Matrix<double, 1, 3> op(1, 0, 0);  // 表示笛卡尔坐标系中的 x 轴的单位向量 [1, 0, 0]
            // 计算 op 向量在法向量 vecN 上的投影，得到一个 1x3 的矩阵 h，表示投影的结果
            Eigen::MatrixXd h = op * vecN.transpose();  // vecN.transpose() 是一个操作，用于计算给定向量 vecN 的转置
            // a new vector, c = h*vecN, so that vector [1,0,0]-c is the projection of x-axis onto the plane with normal vector vecN
            // 计算一个新的向量 c = h * vecN，从而向量 [1,0,0]-c 是 x 轴在法向量 vecN 所在平面上的投影
            Eigen::Matrix<double, 1, 3> c = h(0) * vecN;  // 根据投影结果和法向量 vecN，计算一个新的向量 c，它是投影向量的缩放版本，使其与法向量垂直
            // x-axis - c, the projection
            // 计算 x 轴在法向量 vecN 所在平面上的投影 px，和法向量 vecN 叉乘得到 y 轴 py
            Eigen::Matrix<double, 1, 3> px = op - c;  // 计算一个新的向量 px，它表示 op 向量在法向量 vecN 所在的平面上的投影
            // given the normal vector vecN and the projected x-axis px, the y- axis is cross(vecN,px)
            // 计算一个新的向量 py，它与法向量 vecN 和投影向量 px 垂直，用于构建一个新的坐标系
            Eigen::Matrix<double, 1, 3> py = vecN.cross(px);  // 其中cross(px) 是一个向量叉积（外积）的操作，用于计算向量 vecN 和向量 px 的叉积
            // projection of data onto space span{px,py}
            // 将数据点投影到空间 span{px, py} 上
            Eigen::MatrixXd pcx = cloud_pca * px.transpose();  // 将输入点云 cloud_pca 投影到新的坐标系上，得到一个矩阵 pcx，表示点云在新的 x 轴上的投影
            Eigen::MatrixXd pcy = cloud_pca * py.transpose();  // 将输入点云 cloud_pca 投影到新的坐标系上，得到一个矩阵 pcy，表示点云在新的 y 轴上的投影
            // pdata = np.array([pcx,pcy])
            // represent data in polar coordinates
            // vector<double> rho;
            // vector<double> theta;
            // 将数据点表示为极坐标
            // Eigen::Vector2d 是 Eigen C++ 库中的类型，表示一个 2 维向量。它由两个 double 类型的分量组成，通常用于存储 2D 空间中的坐标或向量。
            // Eigen::aligned_allocator 是 Eigen 库中定义的一个特定的分配器类型。确保存储在容器中的向量在内存中按照适当的方式对齐，以提高数据的访问效率和性能。
            std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points; // x: rho  y: theta
            for (int i = 0; i < pcx.rows(); i++)
            {
                Eigen::Vector2d temp;
                cart2pol(pcx(i), pcy(i), temp);
                points.push_back(temp);
            }
            // main function, count points in bins
            // 计算数据点在 thetaList 和 rhoList 中的直方图（bin 统计）
            Eigen::MatrixXd hist; //16*8    thetaList 17   rhoList 9
            histogram2d(points, thetaList, rhoList, hist);
            // 此时直方图中的每个元素将表示在对应区间内点的相对频率或概率，将直方图归一化到概率分布可以使得在不同规模的点云之间进行比较更加合理和准确
            hist = hist / cloud_filtered->points.size();
            // 将直方图的值存储在 result_A 中
            int hist_size = hist.cols() * hist.rows();
            for (int i = 0; i < hist_size; i++)
            {
                result_A(index_A, i) = hist(i);
            }
            index_A++;
        }
    }
    // 返回计算得到的 Signature Matrix A
    return result_A;
}

// 将笛卡尔坐标系（x, y, z）转换为球坐标系（azimuth, elevation, radius），传入的 vecN 参数是一个三维向量，表示笛卡尔坐标系中的一个点。
void M2DP::cart2sph(double &azm, double &elv, double &r, Eigen::Vector3d vecN)
{
    azm = atan2(vecN.y(), vecN.x());  // 计算方位角
    elv = atan2(vecN.z(), sqrt(vecN.x() * vecN.x() + vecN.y() * vecN.y()));  // 计算俯仰角
    r = sqrt(vecN.x() * vecN.x() + vecN.y() * vecN.y() + vecN.z() + vecN.z());  // 计算半径
}

// 将球坐标系（azimuth, elevation, radius）转换为笛卡尔坐标系（x, y, z），传入的 azm、elv 和 r 参数分别表示球坐标系中的方位角、俯仰角和半径。
void M2DP::sph2cart(double azm, double elv, double r, Eigen::Matrix<double, 1, 3> &vecN)
{
    double x, y, z;
    x = r * cos(elv) * cos(azm);
    y = r * cos(elv) * sin(azm);
    z = r * sin(elv);
    vecN << x, y, z;
}

// 将笛卡尔坐标系（x, y）转换为极坐标系（rho, phi），传入的 x 和 y 参数分别表示笛卡尔坐标系中点的 x 和 y 坐标。
void M2DP::cart2pol(double x, double y, Eigen::Vector2d &vecN)
{
    vecN.x() = sqrt(x * x + y * y); // 计算距离 rho
    vecN.y() = atan2(y, x);         // 计算角度 phi
}

// 将极坐标系（rho, phi）转换为笛卡尔坐标系（x, y），传入的 rho 和 phi 参数分别表示极坐标系中的距离（rho）和角度（phi）。
void M2DP::pol2cart(double rho, double phi, Eigen::Vector2d &vecN)
{
    double x, y;
    x = rho * cos(phi);
    y = rho * sin(phi);
    vecN << x, y;
}

void M2DP::histogram2d(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points, vector<double> thetaList, vector<double> rhoList, Eigen::MatrixXd &hist)
{
    int row, col;  // 于存储直方图矩阵的行数和列数（注意这里行和列反转了）
    row = thetaList.size() - 1;  // 计算直方图矩阵的行数，thetaList.size() 是边界数组 thetaList 的大小，减去 1 表示每个边界之间对应一个直方图区间
    col = rhoList.size() - 1;  // 计算直方图矩阵的列数，rhoList.size() 是边界数组 rhoList 的大小，减去 1 表示每个边界之间对应一个直方图区间
    hist = Eigen::MatrixXd::Zero(row, col);  // 创建一个新的动态大小的 Eigen 矩阵 hist，并将所有元素初始化为零
    // Points x: rho  y: theta
    for (auto pt : points)  // 遍历存储在 points 容器中的每个 2D 向量 pt
    {
        int row_index = -1, col_index = -1;  // 用于存储当前点在直方图矩阵中的行索引和列索引，初始值设为 -1 表示未找到合适的索引
        for (int i = 0; i <= row; i++)  // 在 thetaList 中寻找当前点 pt 的 theta 值所属的直方图区间的上界
        {   // 如果当前点的 theta 值小于当前遍历到的 thetaList[i]，则说明 pt 的 theta 值位于上一个直方图区间内，所以将 row_index 设置为 i - 1
            if (pt.y() < thetaList[i])
            {
                row_index = i - 1;
                break;
            }
        }
        for (int j = 0; j <= col; j++)  // 在 rhoList 中寻找当前点 pt 的 rho 值所属的直方图区间的上界
        {   // 如果当前点的 rho 值小于当前遍历到的 rhoList[j]，则说明 pt 的 rho 值位于上一个直方图区间内，所以将 col_index 设置为 j - 1
            if (pt.x() < rhoList[j])
            {
                col_index = j - 1;
                break;
            }
        }
        // 检查 row_index 和 col_index 是否有效，即它们是否在合法的索引范围内
        if (row_index >= 0 && row_index < row && col_index >= 0 && col_index < col)
        {
            hist(row_index, col_index)++;  // 将直方图矩阵中对应位置的元素加 1，表示该区间内有一个点
        }
    }
}