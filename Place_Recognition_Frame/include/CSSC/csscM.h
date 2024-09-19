
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZ xyz;  // 定义了一个类型别名xyz，表示pcl::PointXYZ类型，用于表示点云中的XYZ坐标
typedef pcl::PointCloud<xyz> pcxyz;  // 定义了一个类型别名pcxyz，表示pcl::PointCloud<xyz>类型，用于表示点云
typedef pcxyz::Ptr pcPtrxyz;  // 定义了一个类型别名pcPtrxyz，表示pcxyz::Ptr类型，用于表示点云的智能指针


#include "../tools/tictoc.h"
#include "../tools/nanoflann.hpp"
#include "../tools/KDTreeVectorOfVectorsAdaptor.h"
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


class csscM {

public:
    //obtain the state of each bin in divided support region
    // 获取点云中每个支持区域（support region）内每个 bin 的状态
    vector<vector<vector<int>>> getBinState(pcPtrxyz &cloud);  // 原函数，只适配于64线激光雷达
    vector<vector<vector<int>>> getBinState(pcPtrxyz &cloud, int nScans);  // 重构函数

    //generate CSSC descriptors from state of bins
    // 根据bin的状态生成CSSC描述符
    MatrixXd getCSSC(vector<vector<vector<int>>> &_state);

    //calculate similarity between two LiDAR scans
    // 计算两个LiDAR扫描之间的相似性
    std::pair<double, int> calculateDistanceBtnPC (pcPtrxyz &cloud1, pcPtrxyz &cloud2 );
    std::pair<double, int> calculateDistanceBtnPC ( pcPtrxyz &cloud1, pcPtrxyz &cloud2, int nScans );

    //reset the parameters of the CSSC descriptor
    // 重新设置CSSC描述符的参数
    void resize(int rows_, int cols_, int hors_);

    // 防止编译时出现重定义错误
    std::vector<float> eig2stdvec( MatrixXd _eigmat );


    MatrixXd getFingerprint(vector<vector<vector<int>>> &_state);

    // User-side API
    void makeAndSaveCSSCAndFingerprint( pcPtrxyz &cloud, int nScans );

    std::pair<int, float> detectLoopClosureIDAndDis();


private:
    // 用于表示CSSC描述符的参数，分别表示描述符的行数、高度和列数上的维度
    int i_cols_ = 20, i_rows_=8, i_hor_ = 40;  // Nd = 20、Nh = 40、Nv = 8

public:
    const double PC_MAX_RADIUS = 80.0;
    const double PC_UNIT_SECTORANGLE = 360.0 / double(i_hor_);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(i_cols_);

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_keys_;

    KeyMat polarcontext_keys_mat_;
    KeyMat polarcontext_keys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    // tree
    const int    NUM_EXCLUDE_RECENT = 50; // simply just keyframe gap, but node position distance-based exclusion is ok. 
    const int    NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

    // config 
    const int    TREE_MAKING_PERIOD_ = 50; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
    int          tree_making_period_conter = 0;

    // loop thres
    const double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // 注意这里阈值改大点，否则PR曲线画不了
    const double CSSC_DIST_THRES = 1000000.13;

};
