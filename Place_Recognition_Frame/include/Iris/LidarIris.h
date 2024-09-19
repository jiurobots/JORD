#ifndef _LIDAR_IRIS_H_
#define _LIDAR_IRIS_H_

#include <vector>

#include <flann/flann.hpp>  // 新增
// #include <cv_bridge/cv_bridge.h  // 新增

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class LidarIris
{
public:
    struct FeatureDesc
    {
        // cv::Mat1b 是 OpenCV 中的数据类型，表示一个单通道的无符号 8 位整数矩阵。
        // 在这里，cv::Mat1b 用于表示虹膜图像，即每个像素点的值都是一个无符号 8 位整数，范围在 0 到 255 之间。
        cv::Mat1b img;
        cv::Mat1b T;
        cv::Mat1b M;
        // 新增一个成员v，以便避免UpdateFrame时重复计算
        std::vector<float> v;
    };

    LidarIris(int nscale, int minWaveLength, float mult, float sigmaOnf, int loop_event) : _nscale(nscale),
                                                                                         _minWaveLength(minWaveLength),
                                                                                         _mult(mult),
                                                                                         _sigmaOnf(sigmaOnf),
                                                                                         _loopEvent(loop_event),
                                                                                         // 为了能编译通过，添加该语句
                                                                                         vecList(flann::Index<flann::L2<float>>(flann::KDTreeIndexParams(4)))
    {
    }
    // 禁用拷贝构造函数和赋值运算符
    LidarIris(const LidarIris &) = delete;
    LidarIris &operator=(const LidarIris &) = delete;
    // 静态成员函数，用于获取点云中的虹膜特征
    static cv::Mat1b GetIris(const pcl::PointCloud<pcl::PointXYZ> &cloud);
    // 成员函数，用于比较两个特征描述的相似性
    float Compare(const FeatureDesc &img1, const FeatureDesc &img2, int *bias = nullptr);

    // 成员函数，获取输入图像的特征描述
    FeatureDesc GetFeature(const cv::Mat1b &src);
    // 成员函数，对输入的图像进行Log-Gabor滤波
    std::vector<cv::Mat2f> LogGaborFilter(const cv::Mat1f &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf);
    // 成员函数，计算两组Hamming Distance
    void GetHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias);
    
    // 静态成员函数，实现矩阵的行循环移位
    static inline cv::Mat circRowShift(const cv::Mat &src, int shift_m_rows);
    // 静态成员函数，实现矩阵的列循环移位
    static inline cv::Mat circColShift(const cv::Mat &src, int shift_n_cols);
    // 静态成员函数，实现矩阵的循环移位
    static cv::Mat circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols);

    // ==================================

    LidarIris(int nscale, int minWaveLength, float mult, float sigmaOnf, int matchNum,  int loop_event) : _nscale(nscale),
                                                                                         _minWaveLength(minWaveLength),
                                                                                         _mult(mult),
                                                                                         _sigmaOnf(sigmaOnf),
                                                                                         _matchNum(matchNum),
                                                                                         _loopEvent(loop_event),
                                                                                         vecList(flann::Index<flann::L2<float>>(flann::KDTreeIndexParams(4))),
                                                                                         // indicesBuffer(std::vector<int>(matchNum)),
                                                                                         // distsBuffer(std::vector<float>(matchNum)),
                                                                                         indices(flann::Matrix<int>(new int[matchNum], 1, matchNum)),
                                                                                         dists(flann::Matrix<float>(new float[matchNum], 1, matchNum))
    {
    }

    FeatureDesc GetFeature(const cv::Mat1b &src, std::vector<float> &vec);
    void UpdateFrame(const cv::Mat1b &frame, int frameIndex, float *matchDistance, int *matchIndex);
    std::pair<int, float> UpdateFrame(); // 新增
    void makeAndSaveLidarIrisAndKeys(pcl::PointCloud<pcl::PointXYZ> &_scan_down); // 新增

private:
    // 私有成员函数，实现LoG特征编码
    void LoGFeatureEncode(const cv::Mat1b &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf, cv::Mat1b &T, cv::Mat1b &M);

    int _nscale;  // 滤波器个数（默认为4）
    int _minWaveLength;  // 18
    float _mult;  // 1.6
    float _sigmaOnf;  // 0.75
    int _matchNum;

    // ===================================
    int _loopEvent;  // 新增

    flann::Index<flann::L2<float>> vecList;
    std::vector<FeatureDesc> featureList;
    std::vector<int> frameIndexList;
    flann::Matrix<int> indices;
    flann::Matrix<float> dists;
    // std::vector<int> indicesBuffer;
    // std::vector<float> distsBuffer;
    std::vector<cv::Mat1b> polarcontexts_; // 新增

};

#endif
