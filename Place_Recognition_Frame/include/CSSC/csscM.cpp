#include "csscM.h"

// 对一个Eigen库中的MatrixXd矩阵进行循环右移（循环位移）操作，并返回位移后的矩阵。
MatrixXd circshift1( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction
    // 检查位移数量是否为非负数
    assert(_num_shift >= 0);

    // 若位移数量为0，则直接返回输入矩阵的副本
    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return
    }

    // 创建一个零矩阵作为位移后的矩阵
    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );

    // 对输入矩阵进行循环右移操作
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    // 返回位移后的矩阵
    return shifted_mat;
}

// 计算两个MatrixXd类型矩阵 _dsc1 和 _dsc2 之间的余弦距离（cosine distance）。
// 余弦距离是一种用于衡量向量相似性的指标，取值范围在[0, 2]之间，值越接近0表示两个向量越相似，值越接近2表示两个向量越不相似。
// 余弦相似度 = 1 - 余弦距离。余弦相似度的取值范围是[-1,1]，相同两个向量的之间的相似度为1。
double cosdistance ( MatrixXd &_dsc1, MatrixXd &_dsc2 )
{
    int num_eff_cols = 0;  // 记录有效列数
    double sum_sector_similarity = 0;  // 记录所有有效列的余弦相似度之和

    for ( int col_idx = 0; col_idx < _dsc1.cols(); col_idx++ )
    {
        VectorXd col_dsc1 = _dsc1.col(col_idx);  // 获取_dsc1的当前列
        VectorXd col_dsc2 = _dsc2.col(col_idx);  // 获取_dsc2的当前列

        // 若某列向量的范数为0（即该列向量是零向量），则跳过计算，因为零向量与任何向量的余弦相似度都是0。
        if( col_dsc1.norm() == 0 || col_dsc2.norm() == 0 )
            continue;
        // 计算当前列的余弦相似度，并累加到总相似度之和中
        double sector_similarity = col_dsc1.dot(col_dsc2) / (col_dsc1.norm() * col_dsc2.norm());
        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        // 记录有效列数，以便后面计算平均相似度
        num_eff_cols = num_eff_cols + 1;
    }
    // 计算平均余弦相似度
    double sc_sim = sum_sector_similarity / num_eff_cols;
    // 返回余弦距离（1减去平均余弦相似度），作为两个矩阵之间的相似性指标。
    return 1.0 - sc_sim;
}

// 将Eigen库中的MatrixXd矩阵转换为std::vector<float>类型的向量，并返回转换后的向量。
std::vector<float> csscM::eig2stdvec( MatrixXd _eigmat )
{
    // 创建一个std::vector<float>类型的向量vec，并用_eigmat的数据填充它
    // 在函数内部，_eigmat.data()返回指向矩阵数据的指针，而_eigmat.size()返回矩阵的总元素个数。
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );  // 相当于[begin, end]
    // 返回转换后的std::vector<float>向量
    return vec;
}

// 原函数，只适配于64线激光雷达，64线激光雷达垂直视场角为[-24, 2]
// 将输入的点云数据按照一定规则划分到一个三维状态矩阵中。
vector<vector<vector<int>>> csscM::getBinState(pcPtrxyz &cloud) {
    // 状态矩阵的维度是i_rows_ * i_cols_ * i_hor_，表示 高*行*列 。
    vector<vector<vector<int>>> state(i_rows_, vector<vector<int>>(i_cols_, vector<int>(i_hor_)));
//  64-line LiDAR vertical FOV [-24,2]
    float h_gap = (float)360/i_hor_;  // 列
    float d_gap = (float)80/i_cols_;  // 行
    float v_gap = (float)32/i_rows_;  // 高
    for (pcl::PointXYZ p : cloud->points)  // 遍历输入的点云数据
    {
        // 计算距离
        float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
        // 计算竖直角度（先求反正切值，然后将弧度值转换为角度值，最后加上 24 将计算得到的角度值进行偏移调整）
        float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 24;
        // 计算水平角度
        float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;

        // 计算索引值
        int Q_dis = std::min(std::max((int)floor(dis / d_gap), 0), i_cols_ - 1);  // 行索引
        int Q_arc = std::min(std::max((int)floor(arc / v_gap), 0), i_rows_ - 1);  // 高索引
        int Q_yaw = std::min(std::max((int)floor(yaw / h_gap), 0), i_hor_ - 1);  // 列索引
        // 将对应位置的计数加1，表示在该状态下有一个新的点
        state[Q_arc][Q_dis][Q_yaw] =  state[Q_arc][Q_dis][Q_yaw] + 1;
    }
    // 返回计算得到的状态矩阵。这个矩阵反映了输入点云数据在不同距离、竖直角度和水平角度下的分布情况。
    return state;
}

// 修改过的函数可根据nScans参数适配于16、32、64线激光雷达，16线激光雷达垂直视场角为[-15, 15]，32线激光雷达垂直视场角为[-25, 15]，64线激光雷达垂直视场角为[-24, 2]
// 将输入的点云数据按照一定规则划分到一个三维状态矩阵中。
vector<vector<vector<int>>> csscM::getBinState(pcPtrxyz &cloud, int nScans) {
    // 状态矩阵的维度是i_rows_ * i_cols_ * i_hor_，表示 高*行*列 。
    vector<vector<vector<int>>> state(i_rows_, vector<vector<int>>(i_cols_, vector<int>(i_hor_)));
    float h_gap = (float)360/i_hor_;  // 列
    float d_gap = (float)80/i_cols_;  // 行
    float v_gap = (float)32/i_rows_;  // 高
    for (pcl::PointXYZ p : cloud->points)  // 遍历输入的点云数据
    {
        // 计算距离
        float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
        // 计算竖直角度（先求反正切值，然后将弧度值转换为角度值，最后加上 15、25 或者 24 将计算得到的角度值进行偏移调整）
        float arc = (atan2(p.data[2], dis) * 180.0f / M_PI);
        if(nScans == 16) {
            arc += 15;
        } else if(nScans == 32) {
            arc += 25;
        } else if(nScans == 64) {
            arc += 24;
        }
        // 计算水平角度
        float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;

        // 计算索引值
        int Q_dis = std::min(std::max((int)floor(dis / d_gap), 0), i_cols_ - 1);  // 行索引
        int Q_arc = std::min(std::max((int)floor(arc / v_gap), 0), i_rows_ - 1);  // 高索引
        int Q_yaw = std::min(std::max((int)floor(yaw / h_gap), 0), i_hor_ - 1);  // 列索引
        // 将对应位置的计数加1，表示在该状态下有一个新的点
        state[Q_arc][Q_dis][Q_yaw] =  state[Q_arc][Q_dis][Q_yaw] + 1;
    }
    // 返回计算得到的状态矩阵。这个矩阵反映了输入点云数据在不同距离、竖直角度和水平角度下的分布情况。
    return state;
}

// 根据输入的三维状态矩阵 _state 计算并返回一个 MatrixXd 类型的 CSSC 描述符矩阵
MatrixXd csscM::getCSSC( vector<vector<vector<int>>> &_state ) {
    MatrixXd m(i_cols_, i_hor_);  // 创建一个大小为i_cols_ * i_hor_的MatrixXd矩阵（行*列）
    // 创建一个三维矩阵weight，用于存储点密度权重
    vector<vector<vector<float>>> weight(i_rows_, vector<vector<float>>(i_cols_, vector<float>(i_hor_)));
    //calculate the point density weight
    // 计算点密度权重。在计算过程中，它会遍历每个bin，根据bin中的计数值计算点密度权重。
    for (int row_idx = 0; row_idx < i_rows_; ++row_idx) {  // 高度
        for (int col_idx = 0; col_idx < i_cols_; col_idx++) {  // 行数
            vector<int> cnts;
            for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {  // 列数
                cnts.push_back(_state[row_idx][col_idx][hor_idx]);
            }
            std::sort(cnts.begin(), cnts.end());
            int median = cnts[20];  // 获得中位数
            for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {  // 列数
                if (median ==0 || _state[row_idx][col_idx][hor_idx] > 2*median){
                    weight[row_idx][col_idx][hor_idx] = 1;
                }
                else
                    weight[row_idx][col_idx][hor_idx] = (float)_state[row_idx][col_idx][hor_idx]/(2*median);
            }
        }
    }

    //calculate each element in the CSSC descriptor
    // 计算CSSC描述符矩阵的每个元素
    for ( int col_idx = 0; col_idx < i_cols_; col_idx++ ) {  // 行数
        for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {  // 列数
            float element = 0;
            for (int row_idx = 0; row_idx < i_rows_; ++row_idx) {  // 高度
                if (_state[row_idx][col_idx][hor_idx] > 1)
                    //each element equals to products point density and elevation weight
                    // 每个元素等于点密度和高程权重的乘积
                    element += weight[row_idx][col_idx][hor_idx] * pow(2,row_idx);  // 直接使用幂函数作为高程权重
            }
            m(col_idx, hor_idx) = element;
        }
    }
    return m;  // 返回计算得到的CSSC描述符矩阵
}

// 通过计算两个CSSC描述符之间的余弦距离来判断两个LiDAR扫描之间的相似性。
std::pair<double, int> csscM::calculateDistanceBtnPC ( pcPtrxyz &cloud1, pcPtrxyz &cloud2 ){
    vector<vector<vector<int>>> state1 = getBinState(cloud1);
    vector<vector<vector<int>>> state2 = getBinState(cloud2);

    float min_distance = 1;
    int trans = 0;
    MatrixXd m1 = getCSSC(state1);
    MatrixXd m2 = getCSSC(state2);
    //here we use force way to calculate coarse yaw angle. And similarity is obtained.
    // 这里我们使用暴力的方法来计算粗略的偏航角。并获得相似性。
    for (int i = 0; i < i_hor_; ++i) {
        MatrixXd transM2 = circshift1(m2, i);
        float dis = cosdistance(m1, transM2);
        if (dis < min_distance){
            min_distance = dis;
            trans = i;
        }
    }
    return make_pair(min_distance, trans);  // 返回的是[最小相似分数, 计算的粗略偏航角]
}

// 重构函数
std::pair<double, int> csscM::calculateDistanceBtnPC ( pcPtrxyz &cloud1, pcPtrxyz &cloud2, int nScans ){
    vector<vector<vector<int>>> state1 = getBinState(cloud1, nScans);
    vector<vector<vector<int>>> state2 = getBinState(cloud2, nScans);

    float min_distance = 1;
    int trans = 0;
    MatrixXd m1 = getCSSC(state1);
    MatrixXd m2 = getCSSC(state2);
    //here we use force way to calculate coarse yaw angle. And similarity is obtained.
    // 这里我们使用暴力的方法来计算粗略的偏航角。并获得相似性。
    for (int i = 0; i < i_hor_; ++i) {
        MatrixXd transM2 = circshift1(m2, i);
        float dis = cosdistance(m1, transM2);
        if (dis < min_distance){
            min_distance = dis;
            trans = i;
        }
    }
    return make_pair(min_distance, trans);  // 返回的是[最小相似分数, 计算的粗略偏航角]
}

// 用于重新设置CSSC描述符的参数。它将i_rows_、i_cols_和i_hor_重新设置为指定的高度数、行数和列数
void csscM::resize(int rows_, int cols_, int hors_) {
    i_rows_ = rows_;
    i_cols_ = cols_;
    i_hor_ = hors_;
}



// ----------------------------- 以下为自己根据论文完善的代码 ---------------------------------

MatrixXd csscM::getFingerprint(vector<vector<vector<int>>> &_state)
{
    MatrixXd m(1, i_rows_ * 2);

    for (int row_idx = 0; row_idx < i_rows_; ++row_idx) {  // 高度
        vector<int> ring_cnts;
        for (int col_idx = 0; col_idx < i_cols_; col_idx++) {  // 行数
            int occupied_nums = 0;  // 占用bin的个数
            for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {  // 列数
                if(_state[row_idx][col_idx][hor_idx])
                    occupied_nums++;
            }
            ring_cnts.push_back(occupied_nums);
        }
        int Sum = 0;  // 该层该ring中占用bin的个数的总数
        int MI = 0;  // 最大值对应的下标
        double Ave = 0.0;  // 计算occupied_nums的平均值
        double SD = 0.0;  // 标准差
        for (int col_idx = 0; col_idx < i_cols_; col_idx++) {  // 行数
            Sum += ring_cnts[col_idx];
            if(ring_cnts[col_idx] > ring_cnts[MI])
                MI = col_idx;
        }
        Ave = (double)Sum * 1.0 / i_cols_;
        // 计算标准差SD
        double sumSquaredDiff = 0.0;
        for (int num : ring_cnts) {
            double diff = (double)num * 1.0 - Ave;
            sumSquaredDiff += diff * diff;
        }
        double variance = sumSquaredDiff / ring_cnts.size();
        SD = sqrt(variance);

        m(0, row_idx * 2 + 0) = Ave;
        m(0, row_idx * 2 + 1) = SD;
    }

    return m;
}


void csscM::makeAndSaveCSSCAndFingerprint( pcPtrxyz &cloud, int nScans )
{
    // Step 1. 对输入点云计算CSSC矩阵
    vector<vector<vector<int>>> _state = getBinState(cloud, nScans);
    Eigen::MatrixXd m_cssc = getCSSC(_state);  // 提取CSSC全局特征描述符

    // Step 2. 使用计算的CSSC矩阵，计算Fingerprint
    Eigen::MatrixXd m_key = getFingerprint(_state);
    // 把Fingerprint的Eigen向量，转成std::vector的数据格式（数组）
    // 最终就是使用Fingerprint作为CSSC的Key，在历史帧中查询相同的Fingerprint来得到候选匹配，然后计算CSSC距离
    std::vector<float> polarcontext_key_vec = csscM::eig2stdvec( m_key );

    // Step 3. 把这帧的数据存到类成员变量中，即存到数据库中，供后面查找匹配
    polarcontexts_.push_back( m_cssc );  // 当前帧点云检测完毕后，将 CSSC 描述符存放于 polarcontexts_
    polarcontext_keys_.push_back( m_key );  // 保存Fingerprint
    polarcontext_keys_mat_.push_back( polarcontext_key_vec );  // 保存 vector 类型的 Fingerprint

    return ;
}

std::pair<int, float> csscM::detectLoopClosureIDAndDis()
{

    int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    // 1.首先将最新的关键帧的CSSC描述符与Fingerprint取出来, 进行回环检测
    auto curr_key = polarcontext_keys_mat_.back(); // current observation (query)
    auto curr_desc = polarcontexts_.back(); // current observation (query)

    // 数据库中关键帧数量太少，则不检测回环
    if( polarcontext_keys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result; // Early return 
    }

    // 2.经过一段时间之后，将历史关键帧的Fingerprint重新构造KD-tree
    if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0)  // 频率控制
    {
        TicToc t_tree_construction;

        // std::vector<std::vector<float> > 类型
        polarcontext_keys_to_search_.clear();
        // 构造用于搜索的Fingerprint集合    assign()  将区间[first,last)的元素赋值到当前的vector容器中
        // 这里减去 NUM_EXCLUDE_RECENT 也就是 不考虑最近的若干帧
        // 最近50帧很难构成回环，因此构造KD-tree的数据不包括最近的50帧
        polarcontext_keys_to_search_.assign( polarcontext_keys_mat_.begin(), polarcontext_keys_mat_.end() - NUM_EXCLUDE_RECENT );

        // KDTreeVectorOfVectorsAdaptor<>的 unique_ptr
        // 重新构造KD-tree
        polarcontext_tree_.reset();
        // TODO: 构建kdtree的细节 ?????????????????????????????????????????
        polarcontext_tree_ = std::make_unique<InvKeyTree>(i_rows_ * 2 /* dim */, polarcontext_keys_to_search_, 10 /* max leaf */ );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;


    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // 3.使用KD-tree进行KNN的最近邻查找，即在KD-tree中搜索与当前闭环检测帧的Fingerprint距离最近的10个最相似的候选帧
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE );  // 10个最相似候选帧的索引
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );  // 10个最相似候选帧的距离，保存候选关键帧的Fingerprint的距离

    // 使用KD-tree找候选Scan Context
    TicToc t_tree_search;
    // 找 NUM_CANDIDATES_FROM_TREE 个最近关键帧
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    // 初始化    用 candidate_indexes 和  out_dists_sqr 数组的数组名地址初始化          设置搜索结果存放的数组
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    // 调用接口查找与当前待搜索的Ring-Key距离最近的10个候选帧
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");

    // 4.遍历最相似候选帧，计算候选CSSC的距离(相似度)，从中找到距离最近(相似度最高)的那个
    TicToc t_calc_dist;
    double candidate_dist = 10000000;
    int candidate_align = 0;
    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {
        // 每个相似候选帧的CSSC矩阵
        MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
        // 当前帧和CSSC矩阵计算相似得分，返回结果是 <最近的sc距离， _sc2右移的列数>

        
        // 这里我们使用暴力的方法来计算粗略的偏航角。并获得相似性。
        for (int i = 0; i < i_hor_; ++i) {
            MatrixXd trans_polarcontext_candidate = circshift1(polarcontext_candidate, i);
            float dis = cosdistance(curr_desc, trans_polarcontext_candidate);
            if (dis < candidate_dist){
                candidate_dist = dis;
                candidate_align = i;
            }
        } 

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;
            nn_idx = candidate_indexes[candidate_iter_idx];  // 找到最匹配的关键帧的索引
        }
    }
    t_calc_dist.toc("Distance calc");

    /* 
     * loop threshold check
     */
    // 5.回环判断，根据阈值判断是否检测到回环，即计算的最小距离要小于设定的阈值
    if( min_dist < CSSC_DIST_THRES )
    {
        loop_id = nn_idx;
        // std::cout.precision(3); 
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;

    }
    else
    {
        std::cout.precision(3);
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // 6.搜索结果保存在 knnsearch_result 中

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = (nn_align * PC_UNIT_SECTORANGLE) * M_PI / 180.0;
    //std::pair<int, float> result {loop_id, yaw_diff_rad};
    std::pair<int, float> result {loop_id, min_dist};

    return result;
}




