#include "SC_I/IScancontext.h"

/*
步骤总结：
 1.给定一帧点云，划分成20个环(ring)，每个环分成60等份(sector)，一共1200个格子(bin)
 2.每个格子存里面点的最大高度值（z值），这样一帧点云就用一个二维图像表示了，想象成一个带高度的俯视图，或者地形图，记为Scan Context
 3.Scan Context进一步计算每一列的均值，得到一个1x60的向量，记为Sector-Key；计算每一行的均值，得到一个20x1的向量，记为Ring-Key
 4.用所有历史帧的Ring-Key构造KD-tree，并且执行KNN搜索，得到初始的候选匹配帧。
   为什么用Ring-Key搜索呢？因为它的维度低，并且具有旋转不变性，因此它是有代表性的，而Sector-Key没有旋转不变性，因此并没有代表性。
 5.对于候选匹配Scan Context，首先要左右循环偏移一下，对齐，实际会用Sector-Key去对齐，得到一个偏移量
 6.对候选匹配Scan Context，施加偏移量，然后作比较。
*/
/*
一、构建Scan Context
二、计算Scan Context的每一行的均值作为RingKey
三、将所有历史帧的RingKey构建Kd-tree查找候选关键帧
四、遍历候选关键帧，选择距离最近的帧作为匹配帧。
计算距离的步骤:
1.计算每一帧的Sectorkey
2.使用Sectorkey，计算当前帧和闭环帧的偏移量shfit
3.使用偏移量对齐两帧
4.计算两帧的每一列距离的平均值作为总的距离值
5.距离最小且满足阈值的候选关键帧即为闭环关键帧返回索引id
五、再用icp匹配
*/





void ISCManager::coreImportTest (void)
{
    cout << "iscancontext lib is successfully imported." << endl;
} // coreImportTest


float ISCManager::rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
}

float ISCManager::deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}


float ISCManager::xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
} // xy2theta


/**
 * @brief  对矩阵进行循环右移
 *
 * @param[in] _mat  输入矩阵
 * @param[in] _num_shift   循环右移的列数
 * @return MatrixXd  移动之后最终的矩阵
 */
MatrixXd ISCManager::circshift( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift


std::vector<float> ISCManager::eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} // eig2stdvec


/**
 * @brief 输入两个_sc矩阵，计算他们之间的Scan Context距离
 *
 * @param[in] _sc1
 * @param[in] _sc2
 * @return double
 */
double ISCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    // 遍历两个Scan Context矩阵的所有列
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);

        // 如果其中有一列一个点云都没有，那么直接不比较
        if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
            continue; // don't count this sector pair.

        // 求两个列向量之间的 cos(\theta)
        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    // 越相似，cos越大，得分越大
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;  // 然后1-cos，变成如果越相似，则值越小

} // distDirectSC


// _vkey1、_vkey2是两个sector key
// 对_vkey2做循环偏移，计算与_vkey1最佳匹配时的偏移量
/**
 * @brief 输入两个sector key，寻找让他们两个最匹配的水平偏移
 *
 * @param[in] _vkey1
 * @param[in] _vkey2
 * @return int  _vkey2右移几个sector，结果和_vkey1最匹配
 */
int ISCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        // 矩阵的列，循环右移shift个单位
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        // 直接相减，sector key是1xN的矩阵，即一个行向量
        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();  // 算范数
        // 查找最小的偏移量，即记录下距离最小时对应的循环偏移量
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey


// 计算两个Scan Context之间的距离(相似度)的函数。这里使用Sector-Key进行加速计算，此处与论文中的不同。
/**
 * @brief 输入两个Scan Context矩阵，计算它们之间的相似度得分
 *
 * @param[in] _sc1
 * @param[in] _sc2
 * @return std::pair<double, int>  <最小的SC距离，此时_sc2应该右移几列>
 */
std::pair<double, int> ISCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    // Step 1. 使用Sector-Key快速对齐，把矩阵的列进行移动
    // 1. fast align using variant key (not in original IROS18)
    // 计算Sector-Key,也就是Sector最大高度均值组成的数组，1xN
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    // 这里将_vkey2循环右移，然后跟_vkey1作比较，找到一个最相似（二者做差最小）的时候，记下循环右移的量
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    // 上面用Sector-Key匹配，找到一个初始的偏移量，但肯定不是准确的，再在这个偏移量左右扩展一下搜索空间
    // 注意这个SEARCH_RADIUS是区间的一半，即左右偏移。这里是0.5 * 10% * 60 = 3，也就是左右扩展3列
    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );  // 这里其实+ii的时候不用再加_sc1.cols()
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // Step 2. 对_sc2循环右移，计算最相近的Scan Context
    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        // _sc2循环右移几位，注意这里是对矩阵进行右移，而不是移动Sector-Key
        // 把Scan Context循环右移一下
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        // 计算两个Scan Context之间的距离
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    // 返回最小Scan Context距离，偏移量
    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext


// 一、构建Scan Context
/**
 * @brief 输入一帧点云，生成Scan Context
 *
 * @param[in] _scan_down 输入点云, 类型SCPointType = pcl::PointXYZI
 * @return 数值计算库Eigen::MatrixXd, 生成的Scan-Context矩阵
 */
MatrixXd ISCManager::makeScancontext( pcl::PointCloud<SCPointType> & _scan_down )
{
    TicToc t_making_desc;

    // 这帧点云的激光点的数量
    int num_pts_scan_down = _scan_down.points.size();

    // main
    // Step 1. 创建bin的矩阵（其中Scan Context的数据结构），并把每个bin内点云最大高度初始化为-1000
    const int NO_POINT = -1000;  // 标记格子中是否有点，如果没有点高度设置成-1000, 是一个肯定没有的值

    // Scan Context是一个ring行、sector列的二维矩阵(20, 60)，和论文中一致
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // 极坐标 within 2d plane
    int ring_idx, sctor_idx;  // Scan Context中的索引，注意是从1开始的
    // Step 2. 遍历每个激光点，往bin中赋值点云最大高度
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        // 使用雷达安装高度外参，将点云移动至z方向大于零的区域，同时将直角坐标系转换为极坐标系
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        // 让高度大于0，所有点的高度都加2，不影响匹配结果
        // 疑问：这里把所有的高度+2，让高度>0，为什么要这么做？
        // 解答：目前感觉就是对于地面机器人这种场景，安装高度不变，然后把安装高度加上去之后，让安装高度之下的点云的高度从负数也都变成正数
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).
        pt.intensity = _scan_down.points[pt_idx].intensity;
        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);  // 距离
        azim_angle = xy2theta(pt.x, pt.y);  // 角度，0~360°

        // if range is out of roi, pass
        // 距离超过80米的点不考虑
        if( azim_range > PC_MAX_RADIUS )
            continue;

        // 计算这个点落到那个bin中，下标从1开始数。注意下面先min再max，其实就是把结果限制到1~PC_NUM之间
        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // taking maximum z
        // 用z值，也就是高度来更新这个格子bin，存最大的高度。
        // 注意之这里为什么-1，以为数组的索引从0开始，上面的索引是[1, PC_NUM]，而在编程中数组的索引应该是[0, PC_NUM-1]
        if ( desc(ring_idx-1, sctor_idx-1) < pt.intensity ) // -1 means cpp starts from 0
            desc(ring_idx-1, sctor_idx-1) = pt.intensity; // update for taking maximum value at that bin
    }

    // Step 3. 把bin中没有点的那些，高度设置成0
    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
} // SCManager::makeScancontext


/*
 * 用scan context来表达一帧点云之后，要做的就是用scan context在历史帧里面找到最相近的scan context。
 * 那为了加速查找，通常都是把历史数据构造成kd-tree，直接用scan context，太大了，查找也会变慢（相似度计算耗时）。
 * 进一步的，把scan context再降维，得到一个ring key（向量），用这玩意去构造kd-tree，以及搜索。
 * ring key是个啥呢，它是scan context每一行算一个均值，组成的20x1的矩阵。
 * 直观上ring key就是每个环的最大高度均值，组合在一起，可以想到ring key跟scan context是一个一对多的关系。
 * 但是没关系，通过ring key做一个初步快速的筛选，找到很多候选scan context，然后再精细比较scan context得到最后的结果。
 */


// 二、计算Scan Context的每一行的均值作为Ring-Key，提取的是行向量的均值
/**
 * @brief 输入构造的Scan Context矩阵，计算Ring-Key。其实就是对于矩阵的每一行（对应每一个环），计算这一行的平均值（即计算一个环中点云最大高度的平均值）
 *
 * @param[in] _desc
 * @return MatrixXd， ring行的向量（Nx1），每个值都是每个环的平均值
 */
MatrixXd ISCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: rowwise mean vector
    */
    // 每行计算一个均值
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        // 计算平均值。注意这个命名很有意思，说ring-key是不变的key，这是由于ring具有旋转不变性
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
} // SCManager::makeRingkeyFromScancontext


/*
 * 通过上面这个ring key在kd-tree里面找到了最邻近的几帧，有对应的Scan Context，接下来就要用当前帧的Scan Context与这些Scan Context进行比较，计算距离。
 * 这里呢，又要引入一个sector key，与ring key对应，这里对每列计算一个均值，组成一个1x60的矩阵。
 */
/*
 * 为什么要按列计算均值呢？因为在场景中同一个位置，旋转一下，看向不同的方向，得到的Scan Context是不一样的（局部ildar系，坐标轴旋转了），对应到Scan Context上的表现就是矩阵左右偏移了。
 * 如果直接用Scan Context去比，显然就认为两帧是不匹配的，但是实际上是在同一个位置。
 * 所以就要对Scan Context左右循环偏移，找一个最佳的匹配位置，然后用偏移后的Scan Context去作比较。
 * 那么实际上用sector key去计算偏移量，然后施加到Scan Context上，最后用偏移的Scan Context做比较。
 */

// Sector-Key的生成函数，提取的是列向量的均值
/**
 * @brief 输入构造的Scan Context矩阵，计算Sector-Key。其实就是对于矩阵的每一列（对应每一个扇区），计算这一列的平均值（即计算一个扇区中点云最大高度的平均值）
 *
 * @param[in] _desc
 * @return MatrixXd， sector列的向量（1xM），每个值都是每个扇区的平均值
 */
MatrixXd ISCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: columnwise mean vector
    */
    // 每列计算一个均值
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        // 计算平均值，这里说sector是变化的key，因为旋转一下之后，variant_key中相当于不同位置之间进行了交换
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} // SCManager::makeSectorkeyFromScancontext


// 算法的第一个接口，描述子生成，对每一个关键帧都进行描述子的提取：根据一帧点云，生成Scan Context，并计算Ring-Key和Sector-Key
/**
 * @brief 输入一帧点云，生成Scan Context，并计算Ring-Key和Sector-Key，然后存到数据库中（每一个帧都用这个函数生成全局描述子）
 *
 * @param[in] _scan_down 输入点云   SCPointType = pcl::PointXYZI
 * @details 提取Scan Context描述子的全部流程
 */
void ISCManager::makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down )
{
    // Step 1. 对输入点云计算Scan Context矩阵（相当于带高度的俯视图）
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1 提取Scan Context全局特征描述符

    // Step 2. 使用计算的Scan Context矩阵，计算Ring-Key和Sector-Key
    // 每个环ring的最大高度平均值
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );  // 求 Ring-Key 特征。Ring-Key 旋转不变
    // 每个轴向sector的最大高度平均值
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );  // 提取Sector-Key 与论文不同，粗略确定平移范围。Sector-Key 旋转变化
    // 把Ring-Key的Eigen向量，转成std::vector的数据格式（数组）
    // 最终就是使用Ring-Key作为Scan Context的Key，在历史帧中查询相同的Ring-Key来得到候选匹配，然后计算Scan Context距离
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

    // Step 3. 把这帧的数据存到类成员变量中，即存到数据库中，供后面查找匹配
    polarcontexts_.push_back( sc );  // 当前帧点云检测完毕后，将Scan Context描述符存放于 polarcontexts_
    polarcontext_invkeys_.push_back( ringkey );  // 保存Ring-Key
    polarcontext_vkeys_.push_back( sectorkey );  // 保存Sector-Key
    polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );  // 保存 vector类型的Ring-Key

    // cout <<polarcontext_vkeys_.size() << endl;

} // SCManager::makeAndSaveScancontextAndKeys
/*
 该函数计算了3种描述子:
 1.sc描述子:
 一个二维的矩阵, 初始化为:
 MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
 即 PC_NUM_RING 行, PC_NUM_SECTOR 列.
 矩阵每个元素的取值为对应(ring, sector) 区域点云的特征, 论文中直接用该区域的点云高度的最大值作为特征, 也可以用平均高度或者其他计算方式来作为特征.
 2.ringkey描述子
 一个列向量:Eigen::MatrixXd invariant_key(_desc.rows(), 1);           // 一个列向量    列数为ring 的个数
 直接对sc描述子的每一行求均值, 作用是 提供旋转不变性.
 3.sectorkey描述子
 一个行向量, 通过对sc描述子的每一列求均值得到.
 主要在求解旋转方向时用到.
 */


// 第二个接口，执行回环检测（闭环检测），即相似性检测
/**
 * @brief 检测闭环对，就是检测数据库中最新的那个点云帧和历史上所有帧之间的回环关系
 *
 * @return std::pair<int, float>
 */
std::pair<int, float> ISCManager::detectLoopClosureID ( void )
{
    int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    // 1.首先将最新的关键帧的Scan Context描述符与Ring-Key取出来, 进行回环检测
    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
    auto curr_desc = polarcontexts_.back(); // current observation (query)

    /* 
     * step 1: candidates from ringkey tree_
     */
    // 数据库中关键帧数量太少，则不检测回环
    if( polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result; // Early return 
    }

    // 2.经过一段时间之后，将历史关键帧的Ring-Key重新构造KD-tree
    // tree_ reconstruction (not mandatory to make everytime) 使用KD-tree对Ring-Key进行查找
    if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost 频率控制
    {
        TicToc t_tree_construction;

        // std::vector<std::vector<float> > 类型
        polarcontext_invkeys_to_search_.clear();
        // 构造用于搜索的Ring-Key集合    assign()  将区间[first,last)的元素赋值到当前的vector容器中
        // 这里减去 NUM_EXCLUDE_RECENT 也就是 不考虑最近的若干帧
        // 最近50帧很难构成回环，因此构造KD-tree的数据不包括最近的50帧
        polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT );

        // KDTreeVectorOfVectorsAdaptor<>的 unique_ptr
        // 重新构造KD-tree
        polarcontext_tree_.reset();
        // TODO: 构建kdtree的细节 ?????????????????????????????????????????
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;
        
    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // 3.使用KD-tree进行KNN的最近邻查找，即在KD-tree中搜索与当前闭环检测帧的Ring-Key距离最近的10个最相似的候选帧
    // knn search
    // NUM_CANDIDATES_FROM_TREE = 10  表示10个候选关键帧
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE );  // 10个最相似候选帧的索引
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );  // 10个最相似候选帧的距离，保存候选关键帧的Ring-Key的距离

    // 使用KD-tree找候选Scan Context
    TicToc t_tree_search;
    // 找 NUM_CANDIDATES_FROM_TREE 个最近关键帧
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    // 初始化    用 candidate_indexes 和  out_dists_sqr 数组的数组名地址初始化          设置搜索结果存放的数组
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    // 调用接口查找与当前待搜索的Ring-Key距离最近的10个候选帧
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");

    /* 
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */
    // 4.遍历最相似候选帧，计算候选Scan Context的距离(相似度)，从中找到距离最近(相似度最高)的那个
    TicToc t_calc_dist;   
    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {
        // 每个相似候选帧的Scan Context矩阵
        MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
        // 当前帧和Scan Context矩阵计算相似得分，返回结果是 <最近的sc距离， _sc2右移的列数>
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

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
    if( min_dist < SC_DIST_THRES )
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
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    //std::pair<int, float> result {loop_id, yaw_diff_rad};
    std::pair<int, float> result {loop_id, min_dist};

    return result;

} // SCManager::detectLoopClosureID

