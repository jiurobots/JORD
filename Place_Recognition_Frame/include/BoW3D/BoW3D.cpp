#include "BoW3D.h"
#include <fstream>

using namespace std;


namespace BoW3D
{
    BoW3D::BoW3D(LinK3D_Extractor* pLinK3D_Extractor, float thr_, int thf_, int num_add_retrieve_features_): 
            mpLinK3D_Extractor(pLinK3D_Extractor), 
            thr(thr_), 
            thf(thf_), 
            num_add_retrieve_features(num_add_retrieve_features_)
    {
       N_nw_ofRatio = std::make_pair(0, 0); 
    }
    
    void BoW3D::update(Frame* pCurrentFrame)
    {
        // 将当前帧pCurrentFrame添加到mvFrames中，mvFrames是保存帧的容器
        mvFrames.emplace_back(pCurrentFrame);

        cv::Mat descriptors = pCurrentFrame->mDescriptors;  // 获取当前帧的描述符矩阵
        long unsigned int frameId = pCurrentFrame->mnId;  // 获取当前帧的ID

        size_t numFeature = descriptors.rows;  // 获取当前帧的特征数量

        if(numFeature < (size_t)num_add_retrieve_features)  // 如果当前帧的特征数量小于num_add_retrieve_features
        {
            for(size_t i = 0; i < numFeature; i++)  // 遍历每个特征
            {
                float *p = descriptors.ptr<float>(i);  // 获取第i个特征的指针
                for(size_t j = 0; j < (size_t)descriptors.cols; j++)  // 遍历每个维度
                {
                    if(p[j] != 0)  // 如果第i个特征的第j个维度不为0
                    {
                        // 查找map中是否存在word(p[j], j)
                        unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>::iterator it; 
                        pair<float, int> word= make_pair(p[j], j);
                        it = this->find(word);

                        if(it == this->end())  // 如果map中不存在word
                        {
                            // 创建一个unordered_set，用于存储描述子word的位置信息(frameId, i)
                            unordered_set<pair<int,int>, pair_hash> place;
                            place.insert(make_pair(frameId, i));
                            (*this)[word] = place;  // 将word和对应的位置信息插入到map中

                            // 更新N_nw_ofRatio
                            N_nw_ofRatio.first++;  // 表示词汇表中的单词数量
                            N_nw_ofRatio.second++; // 表示迄今为止看到的位置总数
                        }
                        else
                        {
                            // 如果word已经存在于map中，将当前帧的位置信息(frameId, i)插入到对应的unordered_set中
                            (*it).second.insert(make_pair(frameId, i));
                            // 更新N_nw_ofRatio
                            N_nw_ofRatio.second++;  // 表示迄今为止看到的位置总数
                        }

                    }
                }
            }
        }
        else
        {
            for(size_t i = 0; i < (size_t)num_add_retrieve_features; i++)
            {
                float *p = descriptors.ptr<float>(i);
                for(size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    if(p[j] != 0)
                    {
                        unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>::iterator it;

                        pair<float, int> word= make_pair(p[j], j);
                        it = this->find(word);

                        if(it == this->end())
                        {
                            unordered_set<pair<int,int>, pair_hash> place;
                            place.insert(make_pair(frameId, i));
                            (*this)[word] = place;

                            N_nw_ofRatio.first++;
                            N_nw_ofRatio.second++;
                        }
                        else
                        {
                            (*it).second.insert(make_pair(frameId, i));

                            N_nw_ofRatio.second++;
                        }
                    }
                }
            }
        }
    }
       
    // 该函数实现了在 BoW3D 特征数据库中检索与当前帧相似的帧（Loop Closure）
    void BoW3D::retrieve(Frame* pCurrentFrame, int &loopFrameId, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt)
    {
        int frameId = pCurrentFrame->mnId;  // 获取当前帧的 ID

        cv::Mat descriptors = pCurrentFrame->mDescriptors;  // 获取当前帧的特征描述符矩阵
        size_t rowSize = descriptors.rows;  // 描述符矩阵的行数，即特征点的个数
        
        map<int, int> mScoreFrameID;  // 用于存储匹配得分与候选匹配帧ID之间的映射关系

        // 下面的if语句直接for(size_t i = 0; i < min(rowSize, (size_t)num_add_retrieve_features); i++)不就行了？？？
        if(rowSize < (size_t)num_add_retrieve_features)  // 当特征点的个数小于设定的num_add_retrieve_features时
        {
            // 对每个特征描述符的每个特征维度进行遍历
            for(size_t i = 0; i < rowSize; i++)  // 遍历每个特征点
            {
                unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;  // 存储特定维度值和维度ID的得分
                                
                float *p = descriptors.ptr<float>(i);  // 获取第i个特征点的描述符指针

                int countValue = 0;

                for(size_t j = 0; j < (size_t)descriptors.cols; j++)  // 遍历该特征点的每个维度
                {
                    countValue++;

                    if(p[j] != 0)  // 如果该维度的值不为0
                    {                   
                        pair<float, int> word = make_pair(p[j], j);  // 形成 DimValu-DimID（特征值-维度ID）的'Word'
                        // 通过 this->find(word) 在 BoW3D 特征数据库中找到包含该特征维度的帧和描述符的位置集合Place Set
                        auto wordPlacesIter = this->find(word);

                        if(wordPlacesIter == this->end())  // 如果没有找到
                        {
                            continue;  // 继续查找下一个维度值
                        }
                        else  // 如果找到了   // 计算当前帧与数据库中每个含有 word 的帧的匹配得分，通过 mPlaceScore 存储该得分。
                        {
                            // 首先计算论文中提到的比率ratio = N_{set} / (N/n_w)，用它来判断是否应该在统计位置数量的时候保留当前单词所在的位置集合
                            double averNumInPlaceSet = N_nw_ofRatio.second / N_nw_ofRatio.first;  // 迄今为止看到的位置总数/词汇表中的单词数量
                            int curNumOfPlaces = (wordPlacesIter->second).size();  // word 对应的 place set 中包含的 place 总个数
                            double ratio = curNumOfPlaces / averNumInPlaceSet;  // 论文中的ratio

                            if(ratio > thr)  // 如果比率大于阈值thr，则继续查找下一个维度值
                            {
                                continue;
                            }

                            for(auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                            {
                                //The interval between the loop and the current frame should be at least 300.
                                // 跳过与当前帧相隔小于300的帧
                                // if(frameId - (*placesIter).first < 300)

                                // 和SC保持一致，最近50帧不考虑回环
                                // if(frameId - (*placesIter).first < 50)
                                if(frameId - (*placesIter).first < 300)
                                {
                                    continue;
                                }

                                auto placeNumIt = mPlaceScore.find(*placesIter);  // 在mPlaceScore中查找是否已经记录了该帧对应的得分
                                
                                if(placeNumIt == mPlaceScore.end())  // 如果没找到，则插入新的帧对应得分
                                {
                                    mPlaceScore[*placesIter] = 1;
                                }
                                else  // 如果找到了，则在原有基础上增加得分
                                {
                                    mPlaceScore[*placesIter]++;
                                }
                            }
                        }
                    }
                }

                for(auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
                {
                    if((*placeScoreIter).second > thf)  // 如果帧得分大于阈值thf
                    {
                       mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;  // 记录帧得分与帧ID的映射关系
                    }
                }
            }
        }
        else  // 当特征点的个数大于等于设定的num_add_retrieve_features时
        {
            for(size_t i = 0; i < (size_t)num_add_retrieve_features; i++)  // 遍历设定的num_add_retrieve_features个特征点
            {
                unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;
                
                float *p = descriptors.ptr<float>(i);

                int countValue = 0;

                for(size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    countValue++;

                    if(p[j] != 0)
                    {
                        pair<float, int> word = make_pair(p[j], j);    

                        auto wordPlacesIter = this->find(word);

                        if(wordPlacesIter == this->end())
                        {
                            continue;
                        }
                        else
                        {
                            double averNumInPlaceSet = (double) N_nw_ofRatio.second / N_nw_ofRatio.first;
                            int curNumOfPlaces = (wordPlacesIter->second).size();

                            double ratio = curNumOfPlaces / averNumInPlaceSet;

                            if(ratio > thr)
                            {
                                continue;
                            }
                            
                            for(auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                            {
                                //The interval between the loop and the current frame should be at least 300.
                                // if(frameId - (*placesIter).first < 300)

                                // 和SC保持一致，最近50帧不考虑回环
                                if(frameId - (*placesIter).first < 50)
                                {
                                    continue;
                                }

                                auto placeNumIt = mPlaceScore.find(*placesIter);                    
                                
                                if(placeNumIt == mPlaceScore.end())
                                {                                
                                    mPlaceScore[*placesIter] = 1;
                                }
                                else
                                {
                                    mPlaceScore[*placesIter]++;                                    
                                }                                                              
                            }                       
                        }                            
                    }
                }

                for(auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
                {
                    if((*placeScoreIter).second > thf) 
                    {
                       mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;
                    }
                }                                   
            }                           
        }

        if(mScoreFrameID.size() == 0)  // 如果得分帧ID的映射关系为空，则表示没有找到匹配的帧
        {
            return;
        }

        for(auto it = mScoreFrameID.rbegin(); it != mScoreFrameID.rend(); it++)  // 从后往前遍历，即按照key从大到小遍历
        {          
            int loopId = (*it).second;  // 获取匹配帧的ID

            Frame* pLoopFrame = mvFrames[loopId];  // 根据匹配帧的ID获取该帧的指针
            vector<pair<int, int>> vMatchedIndex;  // 存储的是帧内匹配好的关键点下标
            
            // 调用match函数，进行特征点匹配，得到匹配特征点的索引
            mpLinK3D_Extractor->match(pCurrentFrame->mvAggregationKeypoints, pLoopFrame->mvAggregationKeypoints, pCurrentFrame->mDescriptors, pLoopFrame->mDescriptors, vMatchedIndex);               

            int returnValue = 0;
            Eigen::Matrix3d loopRelativeR;
            Eigen::Vector3d loopRelativet;

            // 调用loopCorrection函数，根据匹配的特征点，进行回环校正，得到校正后的相对位姿
            returnValue = loopCorrection(pCurrentFrame, pLoopFrame, vMatchedIndex, loopRelativeR, loopRelativet);

            //The distance between the loop and the current should less than 3m.
            // 判断回环校正是否成功，并且相对位移在一定范围内，满足条件则表示找到回环帧
            if(returnValue != -1 && loopRelativet.norm() < 3 && loopRelativet.norm() > 0) 
            {
                loopFrameId = (*it).second;  // 记录回环帧的ID
                loopRelR = loopRelativeR;  // 记录回环帧的相对旋转矩阵
                loopRelt = loopRelativet;  // 记录回环帧的相对位移向量
                
                return;  // 找到一个就直接返回
            }  
        } 
    }

    // 重载函数
    void BoW3D::retrieve(Frame* pCurrentFrame, int &loopFrameId, int &matchKeypointNums, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt)
    {
        int frameId = pCurrentFrame->mnId;  // 获取当前帧的 ID

        cv::Mat descriptors = pCurrentFrame->mDescriptors;  // 获取当前帧的特征描述符矩阵
        size_t rowSize = descriptors.rows;  // 描述符矩阵的行数，即特征点的个数
        
        map<int, int> mScoreFrameID;  // 用于存储匹配得分与候选匹配帧ID之间的映射关系

        // 对每个特征描述符的每个特征维度进行遍历
        for(size_t i = 0; i < min(rowSize, (size_t)num_add_retrieve_features); i++)  // 遍历每个特征点
        {
            unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;  // 存储特定维度值和维度ID的得分
                            
            float *p = descriptors.ptr<float>(i);  // 获取第i个特征点的描述符指针

            int countValue = 0;

            for(size_t j = 0; j < (size_t)descriptors.cols; j++)  // 遍历该特征点的每个维度
            {
                countValue++;

                if(p[j] != 0)  // 如果该维度的值不为0
                {                   
                    pair<float, int> word = make_pair(p[j], j);  // 形成 DimValu-DimID（特征值-维度ID）的'Word'
                    // 通过 this->find(word) 在 BoW3D 特征数据库中找到包含该特征维度的帧和描述符的位置集合Place Set
                    auto wordPlacesIter = this->find(word);

                    if(wordPlacesIter == this->end())  // 如果没有找到
                    {
                        continue;  // 继续查找下一个维度值
                    }
                    else  // 如果找到了   // 计算当前帧与数据库中每个含有 word 的帧的匹配得分，通过 mPlaceScore 存储该得分。
                    {
                        // 首先计算论文中提到的比率ratio = N_{set} / (N/n_w)，用它来判断是否应该在统计位置数量的时候保留当前单词所在的位置集合
                        double averNumInPlaceSet = N_nw_ofRatio.second / N_nw_ofRatio.first;  // 迄今为止看到的位置总数/词汇表中的单词数量
                        int curNumOfPlaces = (wordPlacesIter->second).size();  // word 对应的 place set 中包含的 place 总个数
                        double ratio = curNumOfPlaces / averNumInPlaceSet;  // 论文中的ratio

                        if(ratio > thr)  // 如果比率大于阈值thr，则继续查找下一个维度值
                        {
                            continue;
                        }

                        for(auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                        {
                            //The interval between the loop and the current frame should be at least 300.
                            // 跳过与当前帧相隔小于300的帧
                            // if(frameId - (*placesIter).first < 300)

                            // 和SC保持一致，最近50帧不考虑回环
                            // if(frameId - (*placesIter).first < 50)
                            if(frameId - (*placesIter).first < 300)
                            {
                                continue;
                            }

                            auto placeNumIt = mPlaceScore.find(*placesIter);  // 在mPlaceScore中查找是否已经记录了该帧对应的得分
                                
                            if(placeNumIt == mPlaceScore.end())  // 如果没找到，则插入新的帧对应得分
                            {
                                mPlaceScore[*placesIter] = 1;
                            }
                            else  // 如果找到了，则在原有基础上增加得分
                            {
                                mPlaceScore[*placesIter]++;
                            }
                        }
                    }
                }
            }

            for(auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
            {
                if((*placeScoreIter).second > thf)  // 如果帧得分大于阈值thf
                {
                    mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;  // 记录帧得分与帧ID的映射关系
                }
            }
        }

        if(mScoreFrameID.size() == 0)  // 如果得分帧ID的映射关系为空，则表示没有找到匹配的帧
        {
            return;
        }

        for(auto it = mScoreFrameID.rbegin(); it != mScoreFrameID.rend(); it++)  // 从后往前遍历，即按照key从大到小遍历
        {          
            int loopId = (*it).second;  // 获取匹配帧的ID

            Frame* pLoopFrame = mvFrames[loopId];  // 根据匹配帧的ID获取该帧的指针
            vector<pair<int, int>> vMatchedIndex;  // 存储的是帧内匹配好的关键点下标
            
            // 调用match函数，进行特征点匹配，得到匹配特征点的索引
            mpLinK3D_Extractor->match(pCurrentFrame->mvAggregationKeypoints, pLoopFrame->mvAggregationKeypoints, pCurrentFrame->mDescriptors, pLoopFrame->mDescriptors, vMatchedIndex);               

            int returnValue = 0;
            Eigen::Matrix3d loopRelativeR;
            Eigen::Vector3d loopRelativet;

            // 调用loopCorrection函数，根据匹配的特征点，进行回环校正，得到校正后的相对位姿
            returnValue = loopCorrection(pCurrentFrame, pLoopFrame, vMatchedIndex, loopRelativeR, loopRelativet);

            //The distance between the loop and the current should less than 3m.
            // 判断回环校正是否成功，并且相对位移在一定范围内，满足条件则表示找到回环帧
            if(returnValue != -1 && loopRelativet.norm() < 3 && loopRelativet.norm() > 0) 
            {

                loopFrameId = (*it).second;  // 记录回环帧的ID
                matchKeypointNums = vMatchedIndex.size();  // 记录匹配到的特征点的数量
                loopRelR = loopRelativeR;  // 记录回环帧的相对旋转矩阵
                loopRelt = loopRelativet;  // 记录回环帧的相对位移向量
                
                return;  // 找到一个就直接返回
            }
        }

        return ;
    }

    int BoW3D::loopCorrection(Frame* currentFrame, Frame* matchedFrame, vector<pair<int, int>> &vMatchedIndex, Eigen::Matrix3d &R, Eigen::Vector3d &t)
    {
        // 如果匹配到的特征点对数量小于等于30个，则认为无法进行环检测校正，直接返回-1，表示校正失败
        if(vMatchedIndex.size() <= 30)
        {
            return -1;
        }

        // 对当前帧和匹配帧的边缘特征点进行低曲率过滤，将曲率较低的点移除，保留曲率较高的边缘特征点
        ScanEdgePoints currentFiltered;
        ScanEdgePoints matchedFiltered;
        mpLinK3D_Extractor->filterLowCurv(currentFrame->mClusterEdgeKeypoints, currentFiltered);
        mpLinK3D_Extractor->filterLowCurv(matchedFrame->mClusterEdgeKeypoints, matchedFiltered);

        // 根据之前的边缘特征点匹配索引，找到当前帧和匹配帧中匹配的边缘特征点对
        vector<std::pair<PointXYZSCA, PointXYZSCA>> matchedEdgePt;
        mpLinK3D_Extractor->findEdgeKeypointMatch(currentFiltered, matchedFiltered, vMatchedIndex, matchedEdgePt);
        
        // 根据找到的匹配边缘特征点对，构建用于RANSAC算法的源点云(source)和目标点云(target)
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());

        // 构建点对应关系
        pcl::CorrespondencesPtr corrsPtr (new pcl::Correspondences()); 

        for(int i = 0; i < (int)matchedEdgePt.size(); i++)
        {
            std::pair<PointXYZSCA, PointXYZSCA> matchPoint = matchedEdgePt[i];

            pcl::PointXYZ sourcePt(matchPoint.first.x, matchPoint.first.y, matchPoint.first.z);            
            pcl::PointXYZ targetPt(matchPoint.second.x, matchPoint.second.y, matchPoint.second.z);
            
            source->push_back(sourcePt);
            target->push_back(targetPt);

            pcl::Correspondence correspondence(i, i, 0);
            corrsPtr->push_back(correspondence);
        }

        // 使用RANSAC算法去除源点云和目标点云中的外点，保留符合约束的点对
        pcl::Correspondences corrs;
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> Ransac_based_Rejection;
        Ransac_based_Rejection.setInputSource(source);
        Ransac_based_Rejection.setInputTarget(target);
        double sac_threshold = 0.4;
        Ransac_based_Rejection.setInlierThreshold(sac_threshold);
        Ransac_based_Rejection.getRemainingCorrespondences(*corrsPtr, corrs);

        // 如果保留的点对数量小于等于100个，则认为无法进行环检测校正，直接返回-1，表示校正失败
        if(corrs.size() <= 100)
        {
            return -1;
        }
        
        // 计算源点云(source)和目标点云(target)的质心，用于后续去中心化处理
        Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d p2 = p1;
        int corrSize = (int)corrs.size();
        for(int i = 0; i < corrSize; i++)
        {  
            pcl::Correspondence corr = corrs[i];         
            p1(0) += source->points[corr.index_query].x;
            p1(1) += source->points[corr.index_query].y;
            p1(2) += source->points[corr.index_query].z; 

            p2(0) += target->points[corr.index_match].x;
            p2(1) += target->points[corr.index_match].y;
            p2(2) += target->points[corr.index_match].z;
        }
        
        // 计算质心
        Eigen::Vector3d center1 = Eigen::Vector3d(p1(0)/corrSize, p1(1)/corrSize, p1(2)/corrSize);
        Eigen::Vector3d center2 = Eigen::Vector3d(p2(0)/corrSize, p2(1)/corrSize, p2(2)/corrSize);
        
        // 用于存储去中心化后的点对
        vector<Eigen::Vector3d> vRemoveCenterPt1, vRemoveCenterPt2; 
        for(int i = 0; i < corrSize; i++)
        {
            pcl::Correspondence corr = corrs[i];
            pcl::PointXYZ sourcePt = source->points[corr.index_query];
            pcl::PointXYZ targetPt = target->points[corr.index_match];

            // 去中心化处理：将源点云和目标点云分别进行去中心化处理，使得质心为原点
            Eigen::Vector3d removeCenterPt1 = Eigen::Vector3d(sourcePt.x - center1(0), sourcePt.y - center1(1), sourcePt.z - center1(2));
            Eigen::Vector3d removeCenterPt2 = Eigen::Vector3d(targetPt.x - center2(0), targetPt.y - center2(1), targetPt.z - center2(2));
        
            vRemoveCenterPt1.emplace_back(removeCenterPt1);
            vRemoveCenterPt2.emplace_back(removeCenterPt2);
        }

        // 根据去中心化后的点对计算矩阵w，用于后续SVD分解求解旋转矩阵R
        Eigen::Matrix3d w = Eigen::Matrix3d::Zero();

        for(int i = 0; i < corrSize; i++)
        {
            w += vRemoveCenterPt1[i] * vRemoveCenterPt2[i].transpose();
        }      

        // 使用SVD分解求解旋转矩阵R和平移向量t：对矩阵w进行奇异值分解（SVD），得到矩阵U、V以及对角矩阵D，通过U和V求解旋转矩阵R，通过质心计算平移向量t
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU|Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        
        R = V * U.transpose();
        t = center2 - R * center1;

        return 1;  // 返回1表示校正成功，同时返回计算得到的旋转矩阵R和平移向量t，用于进一步的环检测处理
    }

}
