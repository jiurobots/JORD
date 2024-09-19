#include "NDD/NDD.h"


namespace myNDD
{

    float rad2deg(float radians)
    {
        return radians * 180.0 / M_PI;
    }

    float deg2rad(float degrees)
    {
        return degrees * M_PI / 180.0;
    }

    float xy2theta(const float &_x, const float &_y)
    {
        if (_x >= 0 & _y >= 0)
            return (180 / M_PI) * atan(_y / _x);

        if (_x < 0 & _y >= 0)
            return 180 - ((180 / M_PI) * atan(_y / (-_x)));

        if (_x < 0 & _y < 0)
            return 180 + ((180 / M_PI) * atan(_y / _x));

        if (_x >= 0 & _y < 0)
            return 360 - ((180 / M_PI) * atan((-_y) / _x));
    } // xy2theta

    MatrixXd circshift(MatrixXd &_mat, int _num_shift)
    {
        // shift columns to right direction
        assert(_num_shift >= 0);

        if (_num_shift == 0)
        {
            MatrixXd shifted_mat(_mat);
            return shifted_mat; // Early return
        }

        MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
        for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
        {
            int new_location = (col_idx + _num_shift) % _mat.cols();
            shifted_mat.col(new_location) = _mat.col(col_idx);
        }

        return shifted_mat;

    } 

    std::vector<float> eig2stdvec(MatrixXd _eigmat)
    {
        std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
        return vec;
    } 

    double NDDManager::CorrBtnNDD(MatrixXd &_ndd1, MatrixXd &_ndd2)
    {
        
        //  余弦相似性  F1 = 94%  EP = 92% KITTI 00
        // int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
        // double sum_sector_similarity = 0;
        // // 遍历两个nddan Context矩阵的所有列
        // for (int col_idx = 0; col_idx < _ndd1.cols(); col_idx++)
        // {
        //     VectorXd col_ndd1 = _ndd1.col(col_idx);
        //     VectorXd col_ndd2 = _ndd2.col(col_idx);

        //     // 如果其中有一列一个点云都没有，那么直接不比较
        //     if (col_ndd1.norm() == 0 | col_ndd2.norm() == 0)
        //         continue; // don't count this sector pair.

        //     // 求两个列向量之间的 cos(\theta)
        //     double sector_similarity = col_ndd1.dot(col_ndd2) / (col_ndd1.norm() * col_ndd2.norm());

        //     sum_sector_similarity = sum_sector_similarity + sector_similarity;
        //     num_eff_cols = num_eff_cols + 1;
        // }

        // double ndd_sim = sum_sector_similarity / num_eff_cols;
        // return 1.0 - sc_sim; // 然后1-cos，变成如果越相似，则值越小
         
        //  相关系数相似度   F1 =  0.968% EP = 0.922  KITTI 00

        // Eigen::MatrixXd a = _ndd1;
        // Eigen::MatrixXd b = _ndd2;

        // double a_mean = a.mean();
        // double b_mean = b.mean();

        // a.array() -= a_mean;
        // b.array() -= b_mean;

        // double corr_similarity = (a.array() * b.array()).sum() /
        //                          (std::sqrt((a.array() * a.array()).sum()) * std::sqrt((b.array() * b.array()).sum()));
        // return 1-corr_similarity;
        
        //  KITTI 00  F1 = 0.9687  EP = 0.933  
        Eigen::MatrixXd a_mix = _ndd1;
        Eigen::MatrixXd b_mix = _ndd2;

        double a_mean = a_mix.mean();
        double b_mean = b_mix.mean();

        Eigen::MatrixXd a = a_mix.array() - a_mean;
        a = (a.array() == -a_mean).select(Eigen::MatrixXd::Zero(a.rows(), a.cols()), a.array());

        Eigen::MatrixXd b = b_mix.array() - b_mean;
        b = (b.array() == -b_mean).select(Eigen::MatrixXd::Zero(b.rows(), b.cols()), b.array());

        double corr_similarity = (a.array() * b.array()).sum() /
                                 (std::sqrt((a.array() * a.array()).sum()) * std::sqrt((b.array() * b.array()).sum()));

        return 1-corr_similarity;

    } 

    int NDDManager::fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2)
    {
        int argmin_vkey_shift = 0;
        double min_veky_diff_norm = 10000000;
        for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
        {
            // 矩阵的列，循环右移shift个单位
            MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

            // 直接相减，sector key是1xN的矩阵，即一个行向量
            MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

            double cur_diff_norm = vkey_diff.norm(); // 算范数
            // 查找最小的偏移量，即记录下距离最小时对应的循环偏移量
            if (cur_diff_norm < min_veky_diff_norm)
            {
                argmin_vkey_shift = shift_idx;
                min_veky_diff_norm = cur_diff_norm;
            }
        }

        return argmin_vkey_shift;
    }

    std::pair<double, int> NDDManager::distanceBtnNDD(MatrixXd &_ndd1, MatrixXd &_ndd2)
    {
        MatrixXd vkey_ndd1 = SearchingKeyFromNDD(_ndd1);
        MatrixXd vkey_ndd2 = SearchingKeyFromNDD(_ndd2);
        int argmin_vkey_shift = fastAlignUsingVkey(vkey_ndd1, vkey_ndd2);
        const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _ndd1.cols()); // a half of search range
        std::vector<int> shift_idx_search_space{argmin_vkey_shift};
        for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
        {
            shift_idx_search_space.push_back((argmin_vkey_shift + ii + _ndd1.cols()) % _ndd1.cols()); // 这里其实+ii的时候不用再加_ndd1.cols()
            shift_idx_search_space.push_back((argmin_vkey_shift - ii + _ndd1.cols()) % _ndd1.cols());
        }
        std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());
        int argmin_shift = 0;
        double min_ndd_dist = 10000000;
        for (int num_shift : shift_idx_search_space)
        {

            MatrixXd ndd2_shifted = circshift(_ndd2, num_shift);
            double cur_ndd_dist = CorrBtnNDD(_ndd1, ndd2_shifted);
            if (cur_ndd_dist < min_ndd_dist)
            {
                argmin_shift = num_shift;
                min_ndd_dist = cur_ndd_dist;
            }
        }
        return make_pair(min_ndd_dist, argmin_shift);
    } 

    MatrixXd NDDManager::makeNDD(pcl::PointCloud<NDDPointType> &_nddan_down)
    {      
        pcl::PointCloud<NDDPointType>::Ptr nddan_down_ptr(new pcl::PointCloud<NDDPointType>(_nddan_down));
        pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
        voxelGrid.setInputCloud(nddan_down_ptr);
        voxelGrid.setLeafSize(gridStep, gridStep, gridStep);
        // Perform voxel grid downsampling
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
        voxelGrid.filter(*downsampledCloud);

        // 这里用未采样的点云进行PCA，得到更准确的主成分，
        // 下面点云划分时用采样后的点云，速度更快，也可以不用降采样的点云，速度较慢
        pcl::PCA<pcl::PointXYZI> pca;
        pca.setInputCloud(nddan_down_ptr);

        // Get the eigenvalues and eigenvectors
        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
        Eigen::Vector3f eigenValues = pca.getEigenValues();

        // Compute the feature direction 
        Eigen::Vector3f fd;
        if (eigenValues(1) / 3 > eigenValues(2))
        {
            fd = eigenVectors.col(2);
        }
        else if (eigenValues(0) / 3 > eigenValues(1))
        {
            fd = eigenVectors.col(0);
        }
        else
        {
            fd << 1, 0, 0;
        }

        // Compute the rotation matrix R
        Eigen::Matrix3f R;
        R.col(0) = eigenVectors.col(0);
        R.col(1) = eigenVectors.col(1);
        R.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
        // Apply R to fd
        fd = R * fd;
        // Compute yaw angle
        float yaw = std::atan(fd(1) / fd(0)) + M_PI / 2;

        // Compute rotation matrix A
        Eigen::MatrixXd A(3, 3);
        A << std::cos(yaw), std::sin(yaw), 0,
            -std::sin(yaw), std::cos(yaw), 0,
            0, 0, 1;

        // Apply A to ptcloud.Location
        // cout << A << endl;

        Eigen::MatrixXd ptcloud_matrix(downsampledCloud->size(), 3);
        for (size_t i = 0; i < downsampledCloud->size(); ++i)
        {
            ptcloud_matrix.row(i) << downsampledCloud->at(i).x, downsampledCloud->at(i).y, downsampledCloud->at(i).z;
        }
        Eigen::MatrixXd ptcloud_tran_Matrix = ptcloud_matrix * A;
        pcl::PointCloud<pcl::PointXYZI> ptcloud_tran;
        for (int i = 0; i < ptcloud_tran_Matrix.rows(); ++i)
        {
            pcl::PointXYZI point;
            point.x = ptcloud_tran_Matrix(i, 0);
            point.y = ptcloud_tran_Matrix(i, 1);
            point.z = ptcloud_tran_Matrix(i, 2);
            ptcloud_tran.push_back(point);
        }
        Eigen::VectorXd theta(ptcloud_tran.size());
        Eigen::VectorXd r(ptcloud_tran.size());

        // Calculate theta and r using cartesian to polar conversion
        for (size_t i = 0; i < ptcloud_tran.size(); ++i)
        {
            double _x = ptcloud_tran.points[i].x;
            double _y = ptcloud_tran.points[i].y;
            if (_x >= 0 & _y >= 0)
                theta(i) = (180 / M_PI) * atan(_y / _x);
            if (_x < 0 & _y >= 0)
                theta(i) = 180 - ((180 / M_PI) * atan(_y / (-_x)));

            if (_x < 0 & _y < 0)
                theta(i) = 180 + ((180 / M_PI) * atan(_y / _x));

            if (_x >= 0 & _y < 0)
                theta(i) = 360 - ((180 / M_PI) * atan((-_y) / _x));
            r(i) = std::sqrt(_x * _x + _y * _y);
        }
        double r_step = PC_MAX_RADIUS / PC_NUM_RING;
        double s_step = 360.0 / PC_NUM_SECTOR;

        Eigen::VectorXd r_index(ptcloud_tran.size());
        Eigen::VectorXd s_index(ptcloud_tran.size());
        for (int i = 0; i < ptcloud_tran.size(); i++)
        {
            r_index(i) = std::max(std::min(PC_NUM_RING, int(ceil(r(i) / r_step))), 1);
            s_index(i) = std::max(std::min(PC_NUM_SECTOR, int(ceil(theta(i) / s_step))), 1);
        }
        MatrixXd dendd = MatrixXd::Zero(PC_NUM_RING * 2, PC_NUM_SECTOR);
        for (int i = 0; i < PC_NUM_RING; ++i)
        {
            for (int j = 0; j < PC_NUM_SECTOR; ++j)
            {
                std::vector<pcl::PointXYZI> point;
                Eigen::MatrixXd points_in_bin_cov(3, 3);
                Eigen::VectorXd points_in_bin_mean(3);
                // double nddore;
                // double entropy;
                for (int k = 0; k < ptcloud_tran.size(); ++k)
                {
                    if (r_index(k) == i + 1 && s_index(k) == j + 1)
                    {
                        point.push_back(ptcloud_tran[k]);
                    }
                }
                // cout<<point.size()<<endl;
                if (point.size() > 4)
                {
                    Eigen::MatrixXd point_matrix(point.size(), 3);
                    for (int p = 0; p < point.size(); ++p)
                    {
                        point_matrix(p, 0) = point[p].x;
                        point_matrix(p, 1) = point[p].y;
                        point_matrix(p, 2) = point[p].z;
                    }

                    Eigen::VectorXd points_in_bin_mean = point_matrix.colwise().mean();

                    Eigen::MatrixXd points_in_bin_cov = ((point_matrix.rowwise() - points_in_bin_mean.transpose()).transpose() *
                                                         (point_matrix.rowwise() - points_in_bin_mean.transpose())) /
                                                        (point.size() - 1);
                    EigenSolver<Matrix3d> es(points_in_bin_cov);
                    Matrix3d V = es.eigenvectors().real();
                    Vector3d S = es.eigenvalues().real();

                    if (S(2) < 0.001 * S(0))
                    {
                        S(2) = 0.001 * S(0);
                        points_in_bin_cov = V * S.asDiagonal() * V.transpose();
                    }
                    LLT<MatrixXd> chol(points_in_bin_cov);
                    if (chol.info() != Success)
                    {
                        continue;
                    }
                    int N = 4;
                    double entropy = (N / 2) * (1 + log(2 * M_PI)) + 0.5 * log(points_in_bin_cov.determinant());

                    MatrixXd q = point_matrix.rowwise() - points_in_bin_mean.transpose();
                    MatrixXd gaussianValue = (-q * points_in_bin_cov.inverse() * q.transpose()) / 2.0;
                    VectorXd exp_gaussianValue = gaussianValue.diagonal().array().exp();
                    double nddore = exp_gaussianValue.sum();
                    dendd(i, j) = entropy;
                    dendd(i + PC_NUM_RING, j) = nddore;
                }
                else
                {
                    dendd(i, j) = 0.0;
                    dendd(i + PC_NUM_RING, j) = 0.0;
                }
                //  cout << entropy << "-------" << nddore << endl;
            }
        }
        return dendd;
    } 

    MatrixXd NDDManager::AlignmentKeyFromNDD(Eigen::MatrixXd &_desc)
    {
      
        Eigen::MatrixXd invariant_key(_desc.rows(), 1);
        for (int row_idx = 0; row_idx < _desc.rows(); row_idx++)
        {
            Eigen::MatrixXd curr_row = _desc.row(row_idx);
            invariant_key(row_idx, 0) = curr_row.mean();
        }

        return invariant_key;
    } 

    MatrixXd NDDManager::SearchingKeyFromNDD(Eigen::MatrixXd &_desc)
    {

        Eigen::MatrixXd variant_key(1, _desc.cols());
        for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
        {
            Eigen::MatrixXd curr_col = _desc.col(col_idx);
            variant_key(0, col_idx) = curr_col.mean();
        }

        return variant_key;
    } 

    void NDDManager::makeAndSaveNDDAndKeys(pcl::PointCloud<NDDPointType> &_nddan_down)
    {
        
        Eigen::MatrixXd ndd = makeNDD(_nddan_down); 
        Eigen::MatrixXd ringkey = AlignmentKeyFromNDD(ndd); 
        Eigen::MatrixXd sectorkey = SearchingKeyFromNDD(ndd); 
        std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);
        polarcontexts_.push_back(ndd);                                
        polarcontext_invkeys_.push_back(ringkey);                  
        polarcontext_vkeys_.push_back(sectorkey);                    
        polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec); 
    } 
   
    std::pair<int, float> NDDManager::detectLoopClosureID(void)
    {
        int loop_id{-1}; 
        auto curr_key = polarcontext_invkeys_mat_.back(); 
        auto curr_dendd = polarcontexts_.back();          

        if (polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
        {
            std::pair<int, float> result{loop_id, 0.0};
            return result; // Early return
        }

        if (tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) 
        {
            TicToc t_tree_construction;
            polarcontext_invkeys_to_search_.clear();
            polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT);
            polarcontext_tree_.reset();
            polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */);
            t_tree_construction.toc("Tree construction");
        }
        tree_making_period_conter = tree_making_period_conter + 1;

        double min_dist = 10000000; // init with somthing large
        int nn_align = 0;
        int nn_idx = 0;
        std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE); 
        std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);    
        TicToc t_tree_search;
        nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
        knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
        polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10));
        t_tree_search.toc("Tree search");
        TicToc t_calc_dist;
        for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
        {
            MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
            std::pair<double, int> ndd_dist_result = distanceBtnNDD(curr_dendd, polarcontext_candidate);

            double candidate_dist = ndd_dist_result.first;
            int candidate_align = ndd_dist_result.second;

            if (candidate_dist < min_dist)
            {
                min_dist = candidate_dist;
                nn_align = candidate_align;

                nn_idx = candidate_indexes[candidate_iter_idx]; 
            }
        }
        t_calc_dist.toc("Distance calc");

        if (min_dist < ndd_DIST_THRES)
        {
            loop_id = nn_idx;
            // std::cout.precision(3);
            cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << "." << endl;
            cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
        }
        else
        {
            std::cout.precision(3);
            cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << "." << endl;
            cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
        }

        float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
        // std::pair<int, float> result {loop_id, yaw_diff_rad};
        std::pair<int, float> result{loop_id, min_dist};

        return result;
    }
} 