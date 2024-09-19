#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h> 
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "Frame.h"
#include "LinK3D_Extractor.h"


using namespace std;

// 定义了一个名为BoW3D的类，用于管理三维点云特征的哈希表。
// 它包含了一些辅助函数用于计算哈希值，以及用于自定义std::pair作为键的哈希函数pair_hash。
// 此类还包含了用于更新、回环校正和检索3D点云特征的函数。
namespace BoW3D
{
    class Frame;
    class LinK3D_extractor;
    
    // 这是一个用于哈希组合的辅助函数，用于将一个值和当前的哈希种子组合在一起。
    // 这样做是为了保证哈希结果的随机性和不确定性。它采用位运算和异或操作，通过组合当前的哈希种子和给定的值来生成新的哈希种子。
    template <typename T>
    inline void hash_combine(std::size_t &seed, const T &val) 
    {
        // std::hash<T>()(val)：计算给定值 val 的哈希值
        // ^：异或操作，将当前的哈希种子和哈希值进行异或
        // 0x9e3779b9：一个常数，用于增加哈希的随机性。使用该值是因为它是一个很好的奇异常数（magic number），在哈希函数中的作用是帮助增加哈希的随机性，减少哈希冲突的可能性。
        // (seed << 6)：将当前的哈希种子向左移动 6 位，类似于 seed * 2^6
        // (seed >> 2)：将当前的哈希种子向右移动 2 位，类似于 seed / 2^2
        seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        // 该函数使用 std::hash 来计算给定值 val 的哈希值，并将其与当前的哈希种子进行异或，并加上一些常数以增加哈希的随机性。
        // 这样做的结果是，当使用 hash_combine 对多个值进行组合时，不同的输入值将产生不同的哈希种子，从而增加哈希的随机性和分布性，避免哈希冲突。
    }
    // hash_val（重载版本1）
    template <typename T> 
    inline void hash_val(std::size_t &seed, const T &val) 
    {
        hash_combine(seed, val);
    }
    // hash_val（重载版本2）
    template <typename T1, typename T2>
    inline void hash_val(std::size_t &seed, const T1 &val1, const T2 &val2) 
    {
        hash_combine(seed, val1);
        hash_val(seed, val2);
    }
    // hash_val（重载版本3）
    // inline 表明该函数是内联函数，编译器在编译时会尝试在调用点直接展开函数的内容，以减少函数调用的开销。
    template <typename T1, typename T2>
    inline std::size_t hash_val(const T1 &val1, const T2 &val2) 
    {
        std::size_t seed = 0;  // 声明一个无符号整数类型的变量 seed，并初始化为 0，用于存储最终的哈希值。
        hash_val(seed, val1, val2);
        return seed;
    }

    // 这个自定义结构体 pair_hash 通常用于在使用自定义数据类型的容器（例如 unordered_map 和 unordered_set）时，提供哈希函数的实现。
    // 在这个结构中，重载了圆括号运算符 operator()，用于将std::pair作为键，计算给定键值对 std::pair<T1, T2> 的哈希值。
    // 这样可以使容器在插入和查找元素时能够正确地计算哈希值，以提高数据结构的效率和性能。
    struct pair_hash 
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &p) const {
            // 调用了一个名为 hash_val 的函数，用于计算给定键值对 p 的哈希值
            return hash_val(p.first, p.second);
        }
    };

    // 继承自std::unordered_map，是一个哈希表（unordered_map），用于存储3D点云特征（维度值，维度ID；帧ID、描述符ID）
    // 其键是类型为 pair<float, int> 的值，值是类型为 unordered_set<pair<int, int>, pair_hash> 的无序集合。
    // 这个无序集合的键是类型为 pair<int, int> 的值，也就是键值对。
    class BoW3D: public unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>  //Dimension value, Dimension ID; Frame ID, Descriptor ID
    {
        public:
            BoW3D(LinK3D_Extractor* pLinK3D_Extractor, float thr_, int thf_, int num_add_retrieve_features_);

            ~BoW3D(){}
            
            // 更新3D点云特征
            void update(Frame* pCurrentFrame);

            // 在3D点云特征中进行回环校正（loop correction）
            int loopCorrection(Frame* currentFrame, Frame* matchedFrame, vector<pair<int, int>> &vMatchedIndex, Eigen::Matrix3d &R, Eigen::Vector3d &t);
            
            // 在3D点云特征中检索匹配，即检索与给定帧相关的匹配帧
            void retrieve(Frame* pCurrentFrame, int &loopFrameId, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt);
            void retrieve(Frame* pCurrentFrame, int &loopFrameId, int &matchKeypointNums, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt);

        private:
            LinK3D_Extractor* mpLinK3D_Extractor;

            std::pair<int, int> N_nw_ofRatio; //Used to compute the ratio in our paper.          //用于计算我们的论文中的比率

            vector<Frame*> mvFrames;

            float thr; //Ratio threshold in our paper.            // 我们论文中的比率阈值
            int thf; //Frequency threshold in our paper.          // 我们论文中的频率阈值  
            int num_add_retrieve_features; //The number of added or retrieved features for each frame.   // 每个帧要添加或检索的特征数量
    };
}
