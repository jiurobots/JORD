#include "Iris/LidarIris.h"
#include "../tools/fftm.hpp"

// 生成 80*360 的虹膜图像矩阵
cv::Mat1b LidarIris::GetIris(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // 创建一个大小为 80*360 的虹膜图像矩阵，初始化为全零
    cv::Mat1b IrisMap = cv::Mat1b::zeros(80, 360);

    // 遍历点云中的每个点，使用点的坐标信息计算虹膜图像的值
    for (pcl::PointXYZ p : cloud.points)
    {
        // 计算点到原点的距离
        float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
        // 计算点的偏航角（yaw）并将弧度转换为角度（度数）
        float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
        // 将距离和角度的信息转换为虹膜图像中的相应坐标，并限制在一定范围内，以防越界
        int Q_dis = std::min(std::max((int)floor(dis), 0), 79);
        int Q_arc = std::min(std::max((int)ceil(p.z + 5), 0), 7);
        int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), 359);
        // 使用位运算将对应位置的像素值设置为 1（位运算中的<<表示左移，将1移动到指定的位置）
        IrisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
    }


    // 16-line
    // for (pcl::PointXYZ p : cloud.points)
    // {
    //     float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
    //     float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 15;
    //     float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
    //     int Q_dis = std::min(std::max((int)floor(dis), 0), 79);
    //     int Q_arc = std::min(std::max((int)floor(arc / 4.0f), 0), 7);
    //     int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), 359);
    //     IrisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
    // }


    // 64-line
    // for (pcl::PointXYZ p : cloud.points)
    // {
    //     float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
    //     float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 24;
    //     float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
    //     int Q_dis = std::min(std::max((int)floor(dis), 0), 79);
    //     int Q_arc = std::min(std::max((int)floor(arc / 4.0f), 0), 7);
    //     int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), 359);
    //     IrisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
    // }


    return IrisMap;
}


float LidarIris::Compare(const LidarIris::FeatureDesc &img1, const LidarIris::FeatureDesc &img2, int *bias)
{
    // 根据匹配模式进行不同的处理
    if(_loopEvent==2)  // 正向反向回环都有（最初的Compare版本）
    {
        // 对第一张图像进行 FFT 匹配，得到第一个匹配矩形
        auto firstRect = FFTMatch(img2.img, img1.img);
        int firstShift = firstRect.center.x - img1.img.cols / 2;
        // 计算第一个匹配矩形的 Hamming 距离
        float dis1;
        int bias1;
        GetHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
        
        // 对第二张图像进行一定位移后的处理
        auto T2x = circShift(img2.T, 0, 180);
        auto M2x = circShift(img2.M, 0, 180);
        auto img2x = circShift(img2.img, 0, 180);
        // 对第二张图像进行 FFT 匹配，得到第二个匹配矩形
        auto secondRect = FFTMatch(img2x, img1.img);
        int secondShift = secondRect.center.x - img1.img.cols / 2;
        // 计算第二个匹配矩形的 Hamming 距离
        float dis2 = 0;
        int bias2 = 0;
        GetHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
        
        // 根据 Hamming 距离选择合适的匹配结果
        if (dis1 < dis2)
        {
            if (bias)
                *bias = bias1;
            return dis1;
        }
        else
        {
            if (bias)
                *bias = (bias2 + 180) % 360;
            return dis2;
        }
    }
    if(_loopEvent==1)  // 只有反向回环
    {
        // 对第二张图像进行一定位移后的处理
        auto T2x = circShift(img2.T, 0, 180);
        auto M2x = circShift(img2.M, 0, 180);
        auto img2x = circShift(img2.img, 0, 180);

        // 对第二张图像进行 FFT 匹配，得到匹配矩形
        auto secondRect = FFTMatch(img2x, img1.img);
        int secondShift = secondRect.center.x - img1.img.cols / 2;
        
        // 计算匹配矩形的 Hamming 距离
        float dis2 = 0;
        int bias2 = 0;
        GetHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);

        // 返回结果
        if (bias)
            *bias = (bias2 + 180) % 360;
        return dis2;
    }
    if(_loopEvent==0)  // 只有正向回环
    {
        // 对第一张图像进行 FFT 匹配，得到匹配矩形
        auto firstRect = FFTMatch(img2.img, img1.img);
        int firstShift = firstRect.center.x - img1.img.cols / 2;

        // 计算匹配矩形的 Hamming 距离
        float dis1;
        int bias1;
        GetHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);

        // 返回结果
        if (bias)
            *bias = bias1;
        return dis1;
    }
}

std::vector<cv::Mat2f> LidarIris::LogGaborFilter(const cv::Mat1f &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf)
{
    int rows = src.rows;  // 行数 80
    int cols = src.cols;  // 列数 360
    // cv::Mat2f 是一个OpenCV中用于表示复数图像的数据类型。在这里，cv::Mat2f 表示一个由两个 float 数值组成的复数图像，其中一个数值表示实部，另一个数值表示虚部。
    // 通常在频域图像处理中，比如傅里叶变换和滤波操作，会涉及到复数图像。实部和虚部可以分别代表频域的实部和虚部分量。
    // 在代码中，LogGaborFilter 函数返回的就是一个 std::vector<cv::Mat2f>，其中每个元素是一个复数图像，表示一个尺度下的 Log-Gabor 滤波后的结果。复数图像的每个像素点都包含实部和虚部两个浮点数值。
    // 在 LidarIris 类中，复数图像的使用可能是为了处理频域特征，比如在图像的 Log-Gabor 特征提取过程中。
    cv::Mat2f filtersum = cv::Mat2f::zeros(1, cols);
    std::vector<cv::Mat2f> EO(nscale);
    int ndata = cols;
    if (ndata % 2 == 1)
        ndata--;
    cv::Mat1f logGabor = cv::Mat1f::zeros(1, ndata);
    cv::Mat2f result = cv::Mat2f::zeros(rows, ndata);
    cv::Mat1f radius = cv::Mat1f::zeros(1, ndata / 2 + 1);
    radius.at<float>(0, 0) = 1;
    // 设置半径
    for (int i = 1; i < ndata / 2 + 1; i++)
    {
        radius.at<float>(0, i) = i / (float)ndata;
    }
    double wavelength = minWaveLength;
    // 应用多尺度 Log Gabor 滤波器
    for (int s = 0; s < nscale; s++)
    {
        double fo = 1.0 / wavelength;
        double rfo = fo / 0.5;

        // 计算 Log Gabor 滤波器
        cv::Mat1f temp; //(radius.size());
        cv::log(radius / fo, temp);
        cv::pow(temp, 2, temp);
        cv::exp((-temp) / (2 * log(sigmaOnf) * log(sigmaOnf)), temp);
        temp.copyTo(logGabor.colRange(0, ndata / 2 + 1));
        //
        logGabor.at<float>(0, 0) = 0;
        cv::Mat2f filter;
        cv::Mat1f filterArr[2] = {logGabor, cv::Mat1f::zeros(logGabor.size())};
        cv::merge(filterArr, 2, filter);
        filtersum = filtersum + filter;
        // 对输入图像进行频域滤波
        for (int r = 0; r < rows; r++)
        {
            cv::Mat2f src2f;
            cv::Mat1f srcArr[2] = {src.row(r).clone(), cv::Mat1f::zeros(1, src.cols)};
            cv::merge(srcArr, 2, src2f);
            cv::dft(src2f, src2f);
            cv::mulSpectrums(src2f, filter, src2f, 0);
            cv::idft(src2f, src2f);
            src2f.copyTo(result.row(r));
        }
        EO[s] = result.clone();
        wavelength *= mult;
    }
    // 循环移位，处理频谱中的零频率分量
    filtersum = circShift(filtersum, 0, cols / 2);
    return EO;
}

void LidarIris::LoGFeatureEncode(const cv::Mat1b &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf, cv::Mat1b &T, cv::Mat1b &M)
{
    cv::Mat1f srcFloat;
    // 使用OpenCV中的 convertTo 函数，将输入的图像 src 转换为浮点数类型（32位单通道浮点数图像）。
    // 在这它的作用是将一个8位单通道的灰度图像转换为32位单通道浮点数图像，以便在图像处理过程中能够执行更精确的计算。
    // CV_32FC1 是OpenCV中用于表示图像类型的标志之一，其中 CV_32F 表示图像数据的类型是32位浮点数，而 C1 表示是单通道图像。因此，CV_32FC1 表示单通道的32位浮点数图像。
    // 在 LidarIris 类的上下文中，它可能是为了在后续的处理中使用更精确的浮点数数据进行图像滤波和处理。
    src.convertTo(srcFloat, CV_32FC1);
    auto list = LogGaborFilter(srcFloat, nscale, minWaveLength, mult, sigmaOnf);
    std::vector<cv::Mat1b> Tlist(nscale * 2), Mlist(nscale * 2);
    // 对 Log Gabor 滤波器响应进行编码
    for (int i = 0; i < list.size(); i++)
    {
        cv::Mat1f arr[2];
        cv::split(list[i], arr);
        Tlist[i] = arr[0] > 0;
        Tlist[i + nscale] = arr[1] > 0;
        cv::Mat1f m;
        cv::magnitude(arr[0], arr[1], m);
        Mlist[i] = m < 0.0001;
        Mlist[i + nscale] = m < 0.0001;
    }
    // 将编码结果连接成一个矩阵
    cv::vconcat(Tlist, T);
    cv::vconcat(Mlist, M);
}

LidarIris::FeatureDesc LidarIris::GetFeature(const cv::Mat1b &src)
{
    FeatureDesc desc;
    desc.img = src;
    // 获取图像的 LoG 特征描述
    LoGFeatureEncode(src, _nscale, _minWaveLength, _mult, _sigmaOnf, desc.T, desc.M);
    return desc;
}

// 计算两个特征描述符之间的 Hamming 距离
// Hamming 距离是一种衡量两个等长字符串之间在对应位置上不同字符数量的度量。
void LidarIris::GetHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias)
{
    dis = NAN;  // 初始距离为 NaN
    bias = -1;  // 初始偏移量为 -1
    // 在给定的 scale 周围循环遍历一定范围的位移
    for (int shift = scale - 2; shift <= scale + 2; shift++)
    {
        // 对矩阵 T1 和 M1 进行循环位移，得到 T1s 和 M1s
        cv::Mat1b T1s = circShift(T1, 0, shift);
        cv::Mat1b M1s = circShift(M1, 0, shift);
        // 创建一个蒙版，通过将 M1s 和 M2 进行按位或操作
        cv::Mat1b mask = M1s | M2;
        // 计算蒙版中的位数
        int MaskBitsNum = cv::sum(mask / 255)[0];
        // 计算在排除蒙版中的位的情况下，总共的位数
        int totalBits = T1s.rows * T1s.cols - MaskBitsNum;
        // 计算矩阵 T1s 和 T2 的按位异或结果，并排除蒙版中的位，得到 C
        cv::Mat1b C = T1s ^ T2;
        C = C & ~mask;
        // 计算不同位的数量
        int bitsDiff = cv::sum(C / 255)[0];
        // 如果总位数为 0，则将距离设为 NaN
        if (totalBits == 0)
        {
            dis = NAN;
        }
        else
        {
            // 计算当前的 Hamming 距离
            float currentDis = bitsDiff / (float)totalBits;
            // 更新最小距离和相应的偏移量
            if (currentDis < dis || isnan(dis))
            {
                dis = currentDis;
                bias = shift;
            }
        }
    }
    return;
}

inline cv::Mat LidarIris::circRowShift(const cv::Mat &src, int shift_m_rows)
{
    if (shift_m_rows == 0)
        return src.clone();
    shift_m_rows %= src.rows;
    int m = shift_m_rows > 0 ? shift_m_rows : src.rows + shift_m_rows;
    cv::Mat dst(src.size(), src.type());
    src(cv::Range(src.rows - m, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, m), cv::Range::all()));
    src(cv::Range(0, src.rows - m), cv::Range::all()).copyTo(dst(cv::Range(m, src.rows), cv::Range::all()));
    return dst;
}

inline cv::Mat LidarIris::circColShift(const cv::Mat &src, int shift_n_cols)
{
    if (shift_n_cols == 0)
        return src.clone();
    shift_n_cols %= src.cols;
    int n = shift_n_cols > 0 ? shift_n_cols : src.cols + shift_n_cols;
    // 添加下面的if判断，否则回报错：(692)
    // terminate called after throwing an instance of 'cv::Exception'
    // what():  OpenCV(4.2.0) ../modules/core/src/matrix_wrap.cpp:1659: error: (-215:Assertion failed) !fixedSize() in function 'release'
    // github上有人给出原因是：Sometimes n equals to src.cols, we shell not call copyTo func.
    if(n == src.cols)
        return src.clone();
    cv::Mat dst(src.size(), src.type());
    src(cv::Range::all(), cv::Range(src.cols - n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
    src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
    return dst;
}

cv::Mat LidarIris::circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols)
{
    return circColShift(circRowShift(src, shift_m_rows), shift_n_cols);
}



// ==========================================================

// 重载 GetFeature 函数
LidarIris::FeatureDesc LidarIris::GetFeature(const cv::Mat1b &src, std::vector<float> &vec)
{
    cv::Mat1f temp;
    src.convertTo(temp, CV_32FC1);
    // cv::reduce((temp != 0) / 255, temp, 1, CV_REDUCE_AVG);  // 最后一个参数，将CV_REDUCE_AVG改为1， 或者添加头文件#include <cv_bridge/cv_bridge.h>
    cv::reduce((temp != 0) / 255, temp, 1, 1);
    vec = temp.isContinuous() ? temp : temp.clone();
    FeatureDesc desc;
    desc.img = src;
    LoGFeatureEncode(src, _nscale, _minWaveLength, _mult, _sigmaOnf, desc.T, desc.M);
    desc.v = vec;  // 新增
    return desc;
}

// 根据输入帧的特征，更新特征数据库并进行帧匹配
void LidarIris::UpdateFrame(const cv::Mat1b &frame, int frameIndex, float *matchDistance, int *matchIndex)
{
    // first: calc feature
    // 第一步：计算特征
    std::vector<float> vec;
    auto feature = GetFeature(frame, vec);  // 从输入帧中提取特征
    flann::Matrix<float> queries(vec.data(), 1, vec.size());  // 将特征数据转换为 flann::Matrix 格式
    if (featureList.size() == 0)  // 如果特征数据库为空，构建一个 FLANN 索引并将当前特征添加进去
    {
        if (matchDistance)  // 如果 matchDistance 不为 nullptr，将其设置为 NAN
            *matchDistance = NAN;
        if (matchIndex)  // 如果 matchIndex 不为 nullptr，将其设置为 -1
            *matchIndex = -1;
        vecList.buildIndex(queries);  // 建立特征数据库的索引
    }
    else
    {
        // second: search in database
        // 第二步：在数据库中搜索
        vecList.knnSearch(queries, indices, dists, _matchNum, flann::SearchParams(32));  // 使用 k 最近邻搜索找到最相似的帧
        //thrid: calc matches
        // 第三步：计算匹配度
        std::vector<float> dis(_matchNum);  // 存储匹配距离的数组
        for (int i = 0; i < _matchNum; i++)
        {
            dis[i] = Compare(feature, featureList[indices[0][i]]);  // 计算输入帧与数据库帧的匹配距离
        }
        int minIndex = std::min_element(dis.begin(), dis.end()) - dis.begin();  // 找到最小匹配距离的索引
        if (matchDistance)  // 如果 matchDistance 不为 nullptr，将其设置为最小匹配距离
            *matchDistance = dis[minIndex];
        if (matchIndex)  // 如果 matchIndex 不为 nullptr，将其设置为匹配帧的索引
            *matchIndex = frameIndexList[indices[0][minIndex]];
        // forth: add frame to database
        // 第四步：将当前帧添加到数据库
        vecList.addPoints(queries);  // 将当前帧的特征添加到特征数据库中
    }
    featureList.push_back(feature);  // 将当前帧的特征添加到特征列表中
    frameIndexList.push_back(frameIndex);  // 将当前帧的索引添加到帧索引列表中
}

std::pair<int, float> LidarIris::UpdateFrame()
{
    int matchIndex = -1;
    float matchDistance = NAN;
    const cv::Mat1b &frame = polarcontexts_.back();
    int frameIndex = polarcontexts_.size() - 1;
    // first: calc feature
    std::vector<float> vec;
    auto feature = GetFeature(frame, vec);
    // v.data() 返回特征数据的首个元素的指针，1表示要添加的数据点的数量，v.size() 返回特征数据的元素数量，也就是数据点的维度。
    flann::Matrix<float> queries(vec.data(), 1, vec.size());
    if (featureList.size() == 0)
    {
        vecList.buildIndex(queries);
        // 注意：vecList.buildIndex(queries) 时并没有将 queries 添加到 vecList 中。
        // 实际上，这里的 queries 参数是用于在构建搜索数据结构的索引时提供参考的数据，而不是将其添加到数据结构中的操作。
    }
    else if (featureList.size() < 50)
    {
        matchIndex = -1;
    }
    else
    {
        // second: search in database
        auto &v = featureList[featureList.size() - 49].v;
        vecList.addPoints({v.data(), 1, v.size()});
        int count = vecList.knnSearch(queries, indices, dists, _matchNum, flann::SearchParams(32));
        // thrid: calc matches
        std::vector<float> dis(_matchNum);
        // for (int i = 0; i < _matchNum; i++)
        int loop_cnt = std::min(_matchNum, count);
        for (int i = 0; i < loop_cnt; i++)
        {
            dis[i] = Compare(feature, featureList[indices[0][i]]);
        }
        // std::min_element(vt.begin(), vt.end()) 的作用是在 vt 容器的一部分范围[vt.begin()，vt.end())内寻找最小元素。
        // std::next(vt.begin(), cnt) 的作用是返回从指定迭代器 vt.begin() 开始向前或向后（根据 cnt 的正负）移动指定数量步长 cnt 后的新迭代器。
        // 最后，通过找到最小元素的迭代器与 dis.begin() 相减即可得到最小元素在 dis 向量中的索引。
        int minIndex = std::min_element(dis.begin(), std::next(dis.begin(), loop_cnt)) - dis.begin();
        matchDistance = dis[minIndex];
        matchIndex = frameIndexList[indices[0][minIndex]];
        // forth: add frame to database
        // vecList.addPoints(queries);  // 将当前帧的特征添加到特征数据库中
    }

    if(matchIndex == -1)
        matchDistance = 0.00;

    featureList.push_back(feature);
    frameIndexList.push_back(frameIndex);

    return {matchIndex, matchDistance};
}

void LidarIris::makeAndSaveLidarIrisAndKeys(pcl::PointCloud<pcl::PointXYZ> &_scan_down)
{
    cv::Mat1b sc = GetIris(_scan_down); // v1
    polarcontexts_.push_back(sc);
}


