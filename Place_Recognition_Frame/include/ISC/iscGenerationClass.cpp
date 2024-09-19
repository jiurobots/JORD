// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "iscGenerationClass.h"
// #define INTEGER_INTENSITY

// 默认构造函数，为空
ISCGenerationClass::ISCGenerationClass()
{
    
}

// 回环初始化，包括 ring，sector 等参数赋值
void ISCGenerationClass::init_param(int rings_in, int sectors_in, double max_dis_in){
    rings = rings_in;              // 行数
    sectors = sectors_in;          // 列数
    max_dis = max_dis_in;          // 最大距离
    ring_step = max_dis / rings;     // 行宽度
    sector_step = 2 * M_PI / sectors;  // 列宽度
    print_param();  // 输出初始化后参数的值
    init_color();   // 对颜色初始化

    current_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}

// 初始化颜色投影，将不同的颜色值保存在 color_projection 向量中，以供后续使用。
void ISCGenerationClass::init_color(void){
    // cv::Vec3b 是 OpenCV 中表示 RGB（Red, Green, Blue）颜色的数据类型。它是一个 3 通道的向量，每个通道都用一个 8 位的整数表示，分别代表红色、绿色和蓝色分量的强度。这种数据类型常用于表示图像的像素颜色信息。
    // 在 cv::Vec3b 中，索引 0 对应红色通道，索引 1 对应绿色通道，索引 2 对应蓝色通道。每个通道的值范围在 0 到 255 之间，表示颜色的强度。
    // 在下面的代码中，color_projection 向量存储了多个 cv::Vec3b 类型的颜色值，这些颜色值用于颜色映射，根据不同的数值映射到不同的颜色，以便在后续的处理中使用。
    for(int i=0;i<1;i++){//RGB format
        color_projection.push_back(cv::Vec3b(0,i*16,255));
    }
    for(int i=0;i<15;i++){//RGB format
        color_projection.push_back(cv::Vec3b(0,i*16,255));
    }
    for(int i=0;i<16;i++){//RGB format
        color_projection.push_back(cv::Vec3b(0,255,255-i*16));
    }
    for(int i=0;i<32;i++){//RGB format
        color_projection.push_back(cv::Vec3b(i*32,255,0));
    }
    for(int i=0;i<16;i++){//RGB format
        color_projection.push_back(cv::Vec3b(255,255-i*16,0));
    }
    for(int i=0;i<64;i++){//RGB format
        color_projection.push_back(cv::Vec3b(i*4,255,0));
    }
    for(int i=0;i<64;i++){//RGB format
        color_projection.push_back(cv::Vec3b(255,255-i*4,0));
    }
    for(int i=0;i<64;i++){//RGB format
        color_projection.push_back(cv::Vec3b(255,i*4,i*4));
    }
    // 初始化后的 color_projection 数组总共包含 1 + 15 + 16 + 32 + 16 + 64 + 64 + 64 = 268 个颜色
    // color_projection 数组中的颜色是根据不同的分段逐渐变化的，产生了平滑的渐变效果
    // 前面的颜色段使用蓝色（Blue）为主要颜色，从纯蓝色开始逐渐添加绿色分量，直到达到青色（Cyan）
    // 后面的颜色段从橙色（Orange）开始，向黄色（Yellow）过渡，然后到达红色（Red）
    // 因此，整体来说，颜色变化是从蓝色到绿色、青色、橙色、黄色，最终到达红色，并且在每个颜色段之间都有平滑的渐变。
}

// 输出各项参数
void ISCGenerationClass::print_param(){
    std::cout << "The ISC parameters are:" << std::endl;
    std::cout << "number of rings:\t" << rings << std::endl;
    std::cout << "number of sectors:\t" << sectors << std::endl;
    std::cout << "maximum distance:\t" << max_dis << std::endl;
}

// 计算当前点云的isc信息
ISCDescriptor ISCGenerationClass::calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud){
    // 创建一个大小为 sectors*rings 的全零 ISCDescriptor，用于存储强度信息
    ISCDescriptor isc = cv::Mat::zeros(cv::Size(sectors, rings), CV_8U);
    // 遍历筛选后的点云
    for(int i = 0; i < (int)filtered_pointcloud->points.size(); i++) {
        
        // // 输出警告信息，提醒是否启用了整数强度值选项
        // ROS_WARN_ONCE("intensity is %f, if intensity showed here is integer format between 1-255, please uncomment #define INTEGER_INTENSITY in iscGenerationClass.cpp and recompile", (double) filtered_pointcloud->points[i].intensity);
        // std::cout << "强度为: " << (double) filtered_pointcloud->points[i].intensity << std::endl;
        // std::cout << "如果这里显示的强度是1-255之间的整数格式，请在iscGenerationClass.cpp中取消注释#define INTEGER_INTENSITY并重新编译" << std::endl;
        
        // 计算点到雷达距离，筛选范围内的点进行操作
        double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
        if(distance >= max_dis)  // 如果距离超过最大距离，跳过当前点
            continue;
        // 计算点的水平角度，用于后续确定所在列
        double angle = M_PI + std::atan2(filtered_pointcloud->points[i].y,filtered_pointcloud->points[i].x);
        // 计算当前点在二维数组中位置的索引 std::floor()函数作用是向下取整
        int ring_id = std::floor(distance / ring_step);  // 所在行
        int sector_id = std::floor(angle / sector_step);  // 所在列
        // 如果索引超出数组范围，跳过当前点
        if(ring_id >= rings)
            continue;
        if(sector_id >= sectors)
            continue;
        
        // 读取点的强度信息，根据宏定义选择是否将点的强度值乘以 255 并取整
        // 在一些情况下，强度值可能是在 0 到 1 之间的浮点数，而在其他情况下，它可能是在 1 到 255 之间的整数。
// #ifndef INTEGER_INTENSITY
//         int intensity_temp = (int) (255*filtered_pointcloud->points[i].intensity);
// #else
//         int intensity_temp = (int) (filtered_pointcloud->points[i].intensity);
// #endif
        // 修改
        int intensity_temp = 0;
        if(intensityValueType == 0) {
            intensity_temp = (int) (255*filtered_pointcloud->points[i].intensity);
        } else if(intensityValueType == 1) {
            intensity_temp = (int) (filtered_pointcloud->points[i].intensity);
        }

        // 在二维数组中存储当前区域最大的强度信息
        if(isc.at<unsigned char>(ring_id, sector_id) < intensity_temp)
            isc.at<unsigned char>(ring_id, sector_id) = intensity_temp;
    }
    return isc;
}

// 没用到
ISCDescriptor ISCGenerationClass::getLastISCMONO(void){
    return isc_arr.back();
}

// 将最后一个 ISCDescriptor 中的强度值转换为颜色，即创建一个对应彩色版本的 ISCDescriptor 描述符
ISCDescriptor ISCGenerationClass::getLastISCRGB(void){
    //ISCDescriptor isc = isc_arr.back();
    // 创建一个与颜色通道数和ISCDescriptor的尺寸相同的彩色ISCDescriptor
    ISCDescriptor isc_color = cv::Mat::zeros(cv::Size(sectors, rings), CV_8UC3);
    // 遍历最后一个ISCDescriptor的每个像素
    for (int i = 0; i < isc_arr.back().rows; i++) {
        for (int j = 0;j < isc_arr.back().cols;j++) {
            // 1. 获取最后一个ISCDescriptor在当前位置的强度值
            // 2. 使用强度值作为索引从color_projection中获取对应的颜色
            // 3. 将颜色赋值给彩色ISCDescriptor的当前位置
            isc_color.at<cv::Vec3b>(i, j) = color_projection[isc_arr.back().at<unsigned char>(i, j)];
        }
    }
    return isc_color;
}

// 回环检测主过程的入口
void ISCGenerationClass::loopDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc, Eigen::Isometry3d& odom){
    // 对当前点云进行地面过滤
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    ground_filter(current_pc, pc_filtered);

    // 计算当前点云的描述符信息
    ISCDescriptor desc = calculate_isc(pc_filtered);
    // 获取当前的位置信息
    Eigen::Vector3d current_t = odom.translation();
    //dont change push_back sequence
    // 更新行驶距离信息
    if(travel_distance_arr.size()==0) {
        travel_distance_arr.push_back(0);
    } else {
        // .array().square().sum(): 这一系列操作将差值向量中的每个分量取平方，然后对所有分量进行求和。这相当于计算了位置差向量的欧几里德范数的平方。
        double dis_temp = travel_distance_arr.back() + std::sqrt((pos_arr.back() - current_t).array().square().sum());
        travel_distance_arr.push_back(dis_temp);
    }
    pos_arr.push_back(current_t);  // 将当前位置信息添加到数组中
    isc_arr.push_back(desc);  // 将当前描述符添加到数组中

    current_frame_id = pos_arr.size() - 1;  // 记录当前帧ID
    matched_frame_id.clear();  // 清空匹配的帧ID数组

    //search for the near neibourgh pos
    int best_matched_id = 0;  // 最佳匹配帧的ID
    double best_score = 0.0;  // 最佳匹配得分
    // 遍历历史帧，寻找最佳的回环匹配
    for(int i = 0; i < (int)pos_arr.size(); i++){
        // 移动距离变化 △travel_distance？？？
        
        double delta_travel_distance = travel_distance_arr.back()- travel_distance_arr[i];
        // ？？？
        double pos_distance = std::sqrt((pos_arr[i]-pos_arr.back()).array().square().sum());
        // 如果行驶距离差大于阈值且位置差小于行驶距离差的膨胀系数，则进行匹配判断
        if(delta_travel_distance > SKIP_NEIBOUR_DISTANCE && pos_distance<delta_travel_distance*INFLATION_COVARIANCE){
            double geo_score = 0;     // 几何相似度分数
            double inten_score = 0;  // 强度相似度分数
            // 判断是否是回环，如果是，更新得分和匹配帧ID
            if(is_loop_pair(desc,isc_arr[i],geo_score,inten_score)){
                // 如果几何和强度相似度分数之和大于当前最佳得分，则更新最佳匹配信息
                if(geo_score + inten_score > best_score) {
                    best_score = geo_score + inten_score;
                    best_matched_id = i;
                }
            }
        }
    }
    // 如果找到最佳匹配帧，则将其添加到匹配帧ID数组中
    if(best_matched_id != 0){
        matched_frame_id.push_back(best_matched_id);
        // ROS_INFO("received loop closure candidate: current: %d, history %d, total_score%f",current_frame_id,best_matched_id,best_score);
        std::cout << "检测到回环匹配：  当前帧：  " << current_frame_id << ",  匹配帧：  " << best_matched_id << ",  相似度分数：  " << best_score << std::endl;
    }
}

// 判断输入的两帧是否为回环
bool ISCGenerationClass::is_loop_pair(ISCDescriptor& desc1, ISCDescriptor& desc2, double& geo_score, double& inten_score){

    int angle = 0;  // 初始化角度为0
    // 计算这两帧的几何距离得分，并获取对齐的角度
    geo_score = calculate_geometry_dis(desc1, desc2, angle);
    // 如果几何距离得分高于阈值，再进行强度判断
    if(geo_score > GEOMETRY_THRESHOLD) {
        // 计算强度距离得分，并传入之前求得的对齐角度 angle
        inten_score = calculate_intensity_dis(desc1, desc2, angle);
        // 如果强度得分高于阈值，则两得分均满足，为回环
        if(inten_score > INTENSITY_THRESHOLD){
            // 返回true，表示这两帧是一个回环候选
            return true;
        }
    }
    // 如果以上条件不满足，返回false，表示这对图像不是回环候选
    return false;
}

// 计算两个 ISC 描述符之间的几何距离得分
// 它遍历每个扇区，在每个扇区内遍历可能的对齐扇区，然后比较对应位置的值是否相同，如果相同则增加匹配计数器。
// 在每个扇区的匹配计数器值计算完成后，如果当前扇区的匹配计数器值大于之前的相似度值，就更新相似度值并保存对齐的角度。
// 最终，计算并返回标准化的几何距离得分，该得分在 0 到 1 之间（值越大表示相似性越高），表示两个描述符之间的几何相似度。
double ISCGenerationClass::calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double similarity = 0.0;  // 初始化相似度分数为0.0
    // 按照列数来遍历每个步长
    for(int i = 0; i < sectors; i++) {
        int match_count = 0;  // 匹配计数器初始化为0
        // 对于每个扇区，遍历所有可能的扇区
        for(int p = 0; p < sectors; p++) {
            // 计算新的列索引，实现循环滚动
            int new_col = p + i >= sectors ? p + i - sectors : p + i;
            // 遍历每一个环
            for(int q = 0; q < rings; q++) {
                // 如果两个描述符的对应位置的值相同（都是 1 或都是 0），则匹配计数器加 1
                if((desc1.at<unsigned char>(q, p) == true && desc2.at<unsigned char>(q, new_col) == true) 
                    || (desc1.at<unsigned char>(q, p) == false && desc2.at<unsigned char>(q, new_col) == false)){
                    match_count++;
                }
            }
        }
        // 如果当前扇区的匹配计数器值大于之前的相似度值
        if(match_count > similarity){
            similarity = match_count;  // 更新相似度值
            angle = i;  // 更新对齐的角度
        }
    }
    // 返回相似度值除以总的扇区和环数，得到标准化的几何距离得分
    return similarity / (sectors * rings);
}

// 计算两个 ISC 描述符之间的强度相似度得分，其中 angle 表示描述符之间对齐的角度。
// 通过在相邻的角度范围内计算像素强度的差异，来评估两个描述符的强度相似性。
// 这个相似性度量被归一化为范围在 0 到 1 之间，其中 1 表示强度完全相同，0 表示强度完全不同。
/*
double ISCGenerationClass::calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double difference = 1.0;  // 初始化差异值为最大值
    double angle_temp = angle;  // 存储传入的角度值
    // 遍历相邻角度范围内的值（-10 到 +10）
    for(int i = angle_temp - 10; i < angle_temp + 10; i++) {
        int match_count = 0;  // 匹配像素数量
        int total_points = 0;  // 总像素数量
        // 遍历每个扇区
        for(int p = 0; p < sectors; p++) {
            int new_col = p + i;
            // 对新的列索引，实现循环滚动
            if(new_col >= sectors)
                new_col = new_col-sectors;
            if(new_col < 0)
                new_col = new_col + sectors;

            for(int q = 0; q < rings; q++) {
                match_count += abs(desc1.at<unsigned char>(q, p) - desc2.at<unsigned char>(q, new_col));
                total_points++;
            }
        }
        // 计算当前相邻角度范围内的像素强度差异
        double diff_temp = ((double)match_count) / (sectors * rings * 255);
        // 更新最小差异值
        if(diff_temp < difference)
            difference = diff_temp;

    }
    // 返回强度相似性（值越大表示相似性越高）
    return 1 - difference;
}
*/

// 根据论文重构，余弦距离版本
double ISCGenerationClass::calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double difference = -1;  // 初始化差异值为最小值
    double angle_temp = angle;  // 存储传入的角度值
    // 遍历相邻角度范围内的值（-10 到 +10）
    for(int i = angle_temp - 10; i < angle_temp + 10; i++) {

        // 创建一个大小为 sectors*rings 的全零 ISCDescriptor，用于存储强度信息
        Eigen::MatrixXd desc1_new = Eigen::MatrixXd::Zero(rings, sectors);
        Eigen::MatrixXd desc2_new = Eigen::MatrixXd::Zero(rings, sectors);

        // 遍历每个扇区
        for(int p = 0; p < sectors; p++) {
            int new_col = p + i;
            // 对新的列索引，实现循环滚动
            if(new_col >= sectors)
                new_col = new_col-sectors;
            if(new_col < 0)
                new_col = new_col + sectors;

            for(int q = 0; q < rings; q++) {
                desc1_new(q, p) = desc1.at<unsigned char>(q, p);
                desc2_new(q, p) = desc2.at<unsigned char>(q, new_col);
            }
        }

        int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
        double sum_sector_similarity = 0;
        // 遍历两个Scan Context矩阵的所有列
        for ( int col_idx = 0; col_idx < desc1_new.cols(); col_idx++ )
        {
            Eigen::VectorXd col_isc1 = desc1_new.col(col_idx);
            Eigen::VectorXd col_isc2 = desc2_new.col(col_idx);

            // 如果其中有一列一个点云都没有，那么直接不比较
            if( col_isc1.norm() == 0 | col_isc2.norm() == 0 )
                continue; // don't count this sector pair.

            // 求两个列向量之间的 cos(\theta)
            double sector_similarity = col_isc1.dot(col_isc2) / (col_isc1.norm() * col_isc2.norm());

            sum_sector_similarity = sum_sector_similarity + sector_similarity;
            num_eff_cols = num_eff_cols + 1;
        }

        // 越相似，cos越大，得分越大
        double isc_sim = sum_sector_similarity / num_eff_cols;
        if(isc_sim > difference)
            difference = isc_sim;
    }

    // 返回余弦相似性（值越大表示相似性越高）
    return difference;
}

// 使用 PassThrough 滤波器来过滤掉不在指定范围内的点云数据，这里的目的是去除地面点
void ISCGenerationClass::ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out){
    // 创建一个 PassThrough 滤波器对象
    pcl::PassThrough<pcl::PointXYZI> pass;  // 声明直通滤波
    // 设置输入点云
    pass.setInputCloud (pc_in);  // 传入点云数据
    // 设置滤波器字段为 z，即对点云的 z 值进行滤波
    pass.setFilterFieldName ("z");  // 设置操作的坐标轴
    // 设置 z 值的范围，只保留 z 值在 -0.9 到 30.0 之间的点
    pass.setFilterLimits (-0.9, 30.0);  // 设置坐标范围
    // 应用滤波器，将满足条件的点存储到 pc_out 中
    pass.filter (*pc_out);  // 进行滤波输出
}


// ================================

// 回环初始化，包括 ring，sector 等参数赋值
void ISCGenerationClass::init_param(int rings_in, int sectors_in, double max_dis_in, int intensity_value_type){
    rings = rings_in;              // 行数
    sectors = sectors_in;          // 列数
    max_dis = max_dis_in;          // 最大距离
    ring_step = max_dis / rings;     // 行宽度
    sector_step = 2 * M_PI / sectors;  // 列宽度
    intensityValueType = intensity_value_type;  // 强度类型
    print_param();  // 输出初始化后参数的值
    init_color();   // 对颜色初始化

    current_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}


void ISCGenerationClass::makeAndSaveISC(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc)
{
    // 对当前点云进行地面过滤
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    ground_filter(current_pc, pc_filtered);

    // 计算当前点云的描述符信息
    ISCDescriptor desc = calculate_isc(pc_filtered);

    isc_arr.push_back(desc);  // 将当前描述符添加到数组中
}

std::pair<int, float> ISCGenerationClass::detectLoopClosureIDAndDis()
{
    ISCDescriptor desc = isc_arr.back();
    current_frame_id = isc_arr.size() - 1;  // 记录当前帧ID
    matched_frame_id.clear();  // 清空匹配的帧ID数组
    int best_matched_id = -1;  // 最佳匹配帧的ID
    double best_score = 0.0;  // 最佳匹配得分

    double min_score = 10000.0;  // add

    if(isc_arr.size() <= 50) {
        return {-1, 0.00};
    }

    // 遍历历史帧，寻找最佳的回环匹配
    for(int i = 0; i < (int)isc_arr.size() - 50; i++) {
        double geo_score = 0;     // 几何相似度分数
        double inten_score = 0;  // 强度相似度分数
        // 判断是否是回环，如果是，更新得分和匹配帧ID
        if(is_loop_pair(desc, isc_arr[i], geo_score, inten_score)) {
            // 如果几何和强度相似度分数之和大于当前最佳得分，则更新最佳匹配信息

            // if(geo_score + inten_score > best_score) {
            //     best_score = geo_score + inten_score;
            //     best_matched_id = i;
            // }

            if(1 - inten_score < min_score) {
                min_score = 1 - inten_score;
                best_matched_id = i;
            }
        }
    }
    // 如果找到最佳匹配帧，则将其添加到匹配帧ID数组中
    if(best_matched_id != -1){
        matched_frame_id.push_back(best_matched_id);
        // ROS_INFO("received loop closure candidate: current: %d, history %d, total_score%f",current_frame_id,best_matched_id,best_score);
        // std::cout << "检测到回环匹配：  当前帧：  " << current_frame_id << ",  匹配帧：  " << best_matched_id << ",  相似度分数：  " << best_score << std::endl;
        std::cout << "检测到回环匹配：  当前帧：  " << current_frame_id << ",  匹配帧：  " << best_matched_id << ",  相似度分数：  " << min_score << std::endl;
    }

    // return {best_matched_id, (2.0 - best_score) / 2.0};
    return {best_matched_id, min_score};
}
