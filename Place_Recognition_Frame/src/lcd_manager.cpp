#include "lcd_manager.h"

template<typename Point, size_t DIM>
struct KDTree
{
public:

    void insert(const Point &point)
    {
       values.push_back(point);
       rebuild_tree();
    }

    void search_knn(const Point &point, int k, std::vector<Point> &result, std::vector<size_t> &result_index) const 
    {
        result_index.resize(k);
        result.resize(k);
        if(tree == nullptr){
            result_index.clear();
            result.clear();
            return;
        }
        std::vector<double> out_dists_sqr( k );
        nanoflann::KNNResultSet<double> knnsearch_result( k );//预定义
        knnsearch_result.init( &result_index[0], &out_dists_sqr[0] );//初始化
        tree->index->findNeighbors( knnsearch_result, point.data() /* query */, nanoflann::SearchParams(10) ); 
    }

    void rebuild_tree()
    {
        if(values.size() > last_count + 50)
        {
            if(tree != nullptr)
                delete tree;
            tree = new KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L2>(DIM, values, 10);
            last_count = values.size();
        }
    }

    size_t last_count = 0;
    KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L2> *tree = nullptr;
    std::vector<Point> values;
};


// ============================= KITTI =============================


void LCDManager::KITTI_test(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    NDTMC ndtmc_manager(1.0);  // 创建NDTMC对象，传入分辨率参数

    for (size_t cloud_i = 0; cloud_i < 3; cloud_i++)
    {
        string bin_path = file_list[cloud_i].file_name;
        int bin_number = file_list[cloud_i].order_number;


        // ndtmc_manager.readKittiBin(bin_path);

        pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
        pcl::PointCloud<PointType> laserCloudIn1 = *pc_to_read;
        pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);
        ndtmc_manager.makeAndSaveNDTScanContextAndKeys(current_cloud);


        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        string lidar_filename_path = bin_path;
        ifstream inputfile;
        inputfile.open(lidar_filename_path, ios::binary);
        if (!inputfile)
        {
            cerr << "ERROR: Cannot open file " << lidar_filename_path << "! Aborting..." << endl;
            return;
        }
        inputfile.seekg(0, ios::beg);
        for (int i = 0; inputfile.good() && !inputfile.eof(); i++)
        {
            pcl::PointXYZI p;
            inputfile.read((char *)&p.x, 3 * sizeof(float));
            inputfile.read((char *)&p.intensity, sizeof(float));
            pc_in->points.push_back(p);
        }

        pcl::PointCloud<PointType> laserCloudIn2 = *pc_in;

        std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/NDTMC_KITTI_null.txt";
        std::ofstream loop_file(loop_file_name, std::ios::app);

        if(loop_file.is_open()) {
            for(int pt_idx = 0; pt_idx < pc_in->points.size(); pt_idx++) {
                loop_file << "No." << pt_idx << std::fixed << std::setprecision(8) << "  x = " << pc_in->points[pt_idx].x << ", y = " << pc_in->points[pt_idx].y << ", z = " << pc_in->points[pt_idx].z << ", i = " << pc_in->points[pt_idx].intensity << std::endl;
            }
        }
        loop_file.close();


        pcl::PointCloud<PointType>::Ptr current_cloud2 = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);
        pcl::PointCloud<PointType> laserCloudIn3 = *current_cloud2;

        

        for(int pt_idx = 0; pt_idx < laserCloudIn1.points.size(); pt_idx++) {
            std::cout << "==========================================" << std::endl;
            std::cout << "idx = " << pt_idx << std::endl;
            std::cout << std::fixed << std::setprecision(8) << "laserCloudIn1-------->" << " x = " << laserCloudIn1.points[pt_idx].x << ", y = " << laserCloudIn1.points[pt_idx].y << ", z = " << laserCloudIn1.points[pt_idx].z << ", i = " << laserCloudIn1.points[pt_idx].intensity << std::endl;
            std::cout << std::fixed << std::setprecision(8) << "laserCloudIn2-------->" << " x = " << laserCloudIn2.points[pt_idx].x << ", y = " << laserCloudIn2.points[pt_idx].y << ", z = " << laserCloudIn2.points[pt_idx].z << ", i = " << laserCloudIn2.points[pt_idx].intensity << std::endl;
            std::cout << std::fixed << std::setprecision(8) << "laserCloudIn3-------->" << " x = " << laserCloudIn3.points[pt_idx].x << ", y = " << laserCloudIn3.points[pt_idx].y << ", z = " << laserCloudIn3.points[pt_idx].z << ", i = " << laserCloudIn3.points[pt_idx].intensity << std::endl;
            std::cout << std::fixed << std::setprecision(8) << "laserCloudIn0-------->" << " x = " << ndtmc_manager.pc_in->points[pt_idx].x << ", y = " << ndtmc_manager.pc_in->points[pt_idx].y << ", z = " << ndtmc_manager.pc_in->points[pt_idx].z << ", i = " << ndtmc_manager.pc_in->points[pt_idx].intensity << std::endl;
        }
    }
    

    return ;
}


void LCDManager::KITTI_ESF(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/ESF_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        KDTree< std::array<double, 640>, 640> kdtree;
        std::vector< std::array<double, 640> > history_points;
        std::vector< std::array<double, 640> > result;
        std::vector<size_t> result_index;
        std::queue< std::array<double, 640> > qe;  // 缓冲队列，不考虑前50帧
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
            pcl::ESFEstimation<PointType, pcl::ESFSignature640> ESF;
            ESF.setInputCloud(pc_to_read);
            ESF.compute(*esf_descriptor);
            // 将esf_descriptor转换为y方向的vector
            std::vector<float> esf_descriptor_y;
            for (int i = 0; i < 640; i++)
            {
                esf_descriptor_y.push_back(esf_descriptor->points[0].histogram[i]);
            }
            // 将esf_descriptor_x转换为point_type
            std::array<double, 640> point;
            for (int i = 0; i < 640; i++)
            {
                point[i] = esf_descriptor_y[i];
            }
            qe.push(point);
            history_points.push_back(point);
            if (qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(point, 1, result, result_index);
            int loopFrameId = result_index[0];
            double dist = 0;
            // 计算两个向量的L1距离
            std::array<double, 640> history = history_points[loopFrameId];
            for (int i = 0; i < 640; i++)
            {
                dist += fabs(point[i] - history[i]);
            }
            std::cout << "No." << bin_number << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_M2DP(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/M2DP_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        KDTree<std::array<double, 192>, 192> kdtree;
        std::vector<std::array<double, 192> > history_points;
        std::vector<std::array<double, 192> > result;
        std::vector<size_t> result_index;
        std::queue<std::array<double, 192> > qe;  // 缓冲队列，不考虑前50帧
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);
            
            M2DP m2dp(current_cloud);
            Eigen::Matrix<double, 1, 192> desM2dp;
            desM2dp = m2dp.get_m2dp_result();
            std::array<double, 192> A_m2dp_point;
            for(int i = 0; i < 192; i++)
            {
                A_m2dp_point[i] = desM2dp(0, i);
            }
            qe.push(A_m2dp_point);
            history_points.push_back(A_m2dp_point);
            if(qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(A_m2dp_point, 1, result, result_index);
            int loopFrameId = result_index[0];
            // 计算两个向量的L2距离
            double dist = 0;
            std::array<double, 192> history = history_points[loopFrameId];
            for(int i = 0; i < 192; i++)
            {
                dist += pow(A_m2dp_point[i] - history[i], 2);
            }
            dist = sqrt(dist);

            std::cout << "No." << cloud_i << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_Scan_Context(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    //std::string data_root = "/home/jm/catkin_ws/data/kitti_bin/00/";
    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    //输出所有结果
    
    /*
    static std::once_flag flag;
    std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("../results/SC.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    */
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file;
    // loop_file.open("../results/SC.txt", std::ios::app);
    loop_file.open(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        SCManager SC;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC.detectLoopClosureID();
            std::cout << "No." << bin_number << ": " << result.first << " " << result.second << std::endl;
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_Iris_KNN(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/Iris_KITTI_KNN_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        int loop_event = 2;
        if(dataset_sequence == "00" || dataset_sequence == "05") {
            loop_event = 0;
        } else if(dataset_sequence == "08" || dataset_sequence == "01" || dataset_sequence == "02" || dataset_sequence == "03" || dataset_sequence == "04" || dataset_sequence == "05") {
            loop_event = 1;
        } else if(dataset_sequence == "02") {
            loop_event = 2;
        }

        LidarIris Iris(4, 18, 1.6, 0.75, 10, loop_event);
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;

            Iris.makeAndSaveLidarIrisAndKeys(laserCloudIn);
            // auto [id, shift] = Iris.UpdateFrame(loop_file_name);
            // auto [loopFrameId, min_dis] = Iris.UpdateFrame();
            std::pair<int, float> result = Iris.UpdateFrame();
            int loopFrameId = result.first;
            float min_dis = result.second;

            std::cout << "No." << bin_number << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return ;
}

void LCDManager::KITTI_Iris(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/Iris_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        int loop_event = 2;
        if(dataset_sequence == "00" || dataset_sequence == "05") {
            loop_event = 0;
        } else if(dataset_sequence == "08" || dataset_sequence == "01" || dataset_sequence == "02" || dataset_sequence == "03" || dataset_sequence == "04" || dataset_sequence == "05") {
            loop_event = 1;
        } else if(dataset_sequence == "02") {
            loop_event = 2;
        }
        LidarIris Iris(4, 18, 1.6, 0.75, loop_event);
        std::vector<LidarIris::FeatureDesc> dataset(file_list.size());
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;

            cv::Mat1b li1 = LidarIris::GetIris(laserCloudIn);
            LidarIris::FeatureDesc fd1 = Iris.GetFeature(li1);
            dataset[cloud_i] = fd1;

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            int loopFrameId = -1;
            float min_dis = 100000.0;
            for(size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
            {
                LidarIris::FeatureDesc fd2 = dataset[cloud_j];

                int bias;
                auto dis = Iris.Compare(fd1, fd2, &bias);
                if(dis < min_dis)
                {
                    min_dis = dis;
                    loopFrameId = cloud_j;
                }
            }
            if(loopFrameId == -1)
                min_dis = 0.0;

            std::cout << "No." << bin_number << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_SC_Intensity(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_Intensity_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        ISCManager SC_Intensity;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC_Intensity.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC_Intensity.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_ISC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/ISC_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        int ring_height = 20;
        int sector_width = 60;
        double max_dis= 50.0;
        ISCGenerationClass ISC;
        // ISC.init_param(60, 60, 40);
        ISC.init_param(ring_height, sector_width, max_dis, 0);
        std::cout << "注意：如果点云的强度是1-255之间的整数格式，请在iscGenerationClass.cpp中取消注释#define INTEGER_INTENSITY并重新编译" << std::endl;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            ISC.makeAndSaveISC(current_cloud);

            /*
            namespace fs = std::filesystem;

            // 生成二维ISC彩色图
            ISCDescriptor isc_color = ISC.getLastISCRGB();
            cv::resize(isc_color, isc_color, cv::Size(sector_width * 15, ring_height * 15));

            // 定义图片输出文件夹
            std::string color_outputPath = "/home/jlurobot/catkin_ws/src/JM_LCD/results/ISC_KITTI_color_" + dataset_sequence + "/";
            // 如果图片输出文件夹不存在，则创建该文件夹
            fs::path folderPath = color_outputPath;
            if (!fs::exists(folderPath)) {
                if(fs::create_directory(folderPath)) {
                    std::cout << "Folder created successfully." << std::endl;
                } else {
                    std::cerr << "Failed to create folder." << std::endl;
                }
            }

            // Save the ISC color image to a file
            std::string filename = "isc_color_image" + to_string(cloud_i) + ".png";
            cv::imwrite(color_outputPath + filename, isc_color);

            // Optional: If you also want to show the image using OpenCV window
            // cv::namedWindow("ISC Color Image", cv::WINDOW_NORMAL);
            // cv::imshow("ISC Color Image", isc_color);
            // cv::waitKey(0); // Wait for a key press to close the window

            */

            std::pair<int, float> result = ISC.detectLoopClosureIDAndDis();

            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_CSSC_Force(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/CSSC_KITTI_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path_now = file_list[cloud_i].file_name;
            int bin_number_now = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read_now = readKITTIPointCloudBin<PointType>(bin_path_now);
            // pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read_now);

            double min_dis = 2.00;
            int loopFrameId = -1;
            if(cloud_i >= 50) {  // 跳过前50帧
                for(size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
                {
                    string bin_path_pre = file_list[cloud_j].file_name;
                    int bin_number_pre = file_list[cloud_j].order_number;
                    pcl::PointCloud<PointType>::ConstPtr pc_to_read_pre = readKITTIPointCloudBin<PointType>(bin_path_pre);
                    // pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
                    pcl::PointCloud<PointType>::Ptr previous_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read_pre);

                    csscM CSSC;  // 创建了一个CSSC对象，用于计算LiDAR点云之间的相似性
                    // 调用CSSC对象的 calculateDistanceBtnPC 函数，传入两个点云数据 cloud1 和 cloud2，计算它们之间的相似性。
                    // 该函数返回一个 std::pair<float, float> 类型的结果，其中first表示计算得到的相似性值，second表示对应的粗略偏航角。

                    std::pair<double, int> result = CSSC.calculateDistanceBtnPC(previous_cloud, current_cloud, 64);
                    if(result.first < min_dis) {
                        min_dis = result.first;
                        loopFrameId = bin_number_pre;
                    }
                }
            }

            if(loopFrameId == -1)
                min_dis = 0.0;

            std::cout << "No." << bin_number_now << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << bin_number_now << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();
    return;
}

void LCDManager::KITTI_CSSC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/CSSC_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        csscM CSSC;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr cloudPtr = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            CSSC.makeAndSaveCSSCAndFingerprint(cloudPtr, 64);
            std::pair<int, float> result = CSSC.detectLoopClosureIDAndDis();
            std::cout << "No." << bin_number << ": " << result.first << " " << result.second << std::endl;
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_BoW3D(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/BoW3D_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {

        // 设置 LinK3D 的参数
        int nScans = 64;  // 激光雷达扫描线的数量
        float scanPeriod = 0.1;  // 激光雷达扫描周期
        float minimumRange = 0.1;  // 最小测距范围
        float distanceTh = 0.4;  // 距离阈值
        int matchTh = 6;  // 匹配阈值

        // 设置 BoW3D 的参数
        float thr = 3.5;  // 阈值
        int thf = 5;  // 阈值
        int num_add_retrieve_features = 5;  // 添加和检索特征的数量
        
        // 创建 LinK3D_Extractor 和 BoW3D 对象
        BoW3D::LinK3D_Extractor* pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
        BoW3D::BoW3D* pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            // 创建当前帧的Frame对象，并更新BoW3D
            BoW3D::Frame* pCurrentFrame = new BoW3D::Frame(pLinK3dExtractor, current_cloud);

            int loopFrameId = -1;  // 回环帧ID
            int matchKeypointNums = 0;  // 匹配到的特征关键点数量
            Eigen::Matrix3d loopRelR;  // 旋转
            Eigen::Vector3d loopRelt;  // 平移

            if(pCurrentFrame->mnId < 50)  // 前50帧不考虑回环，如考虑，将50换成2
            {
                // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
                pBoW3D->update(pCurrentFrame);
                loop_file << bin_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
                continue;
            }

            clock_t start, end;
            double time;
            start = clock();
            // 检索回环帧，并得到相对姿态信息
            // pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, matchKeypointNums, loopRelR, loopRelt);

            end = clock();
            time = ((double) (end - start)) / CLOCKS_PER_SEC;
            
            // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
            pBoW3D->update(pCurrentFrame);

            if(loopFrameId == -1)
            {
                cout << "-------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl; 
                cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl;
            }
            else
            {
                cout << "--------------------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" <<  pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;
                
                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;
                                
                cout << "Loop Relative t: " << endl;
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
            }

            loop_file << bin_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::KITTI_NDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/NDD_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        myNDD::NDDManager NDD;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            NDD.makeAndSaveNDDAndKeys(laserCloudIn);
            std::pair<int, float> result = NDD.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}


void LCDManager::KITTI_NDTMC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/NDTMC_KITTI_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        NDTMC ndtmc_manager(1.0);  // 创建NDTMC对象，传入分辨率参数
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;

            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);
            
            // for(int pt_idx  = 0; pt_idx < current_cloud->points.size(); pt_idx++)  // 源码中是这样处理的
            // {
            //     current_cloud->points[pt_idx].z += ndtmc_manager.LIDAR_HEIGHT;
            // }

            ndtmc_manager.makeAndSaveNDTScanContextAndKeys(current_cloud);

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            double min_dis = 1000.00;
            int loopFrameId = -1;
            for(size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
            {
                std::pair<double, int> result = ndtmc_manager.distanceBtnNDTScanContext(ndtmc_manager.all_desc[cloud_i], ndtmc_manager.all_desc[cloud_j], 1);
                if(result.first < min_dis) {
                    min_dis = result.first;
                    loopFrameId = cloud_j;
                }
            }

            if(loopFrameId == -1)
                min_dis = 0.0;

            std::cout << "No." << bin_number << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


// ============================= JORD =============================


void LCDManager::JORD_ESF(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/ESF_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        KDTree< std::array<double, 640>, 640> kdtree;
        std::vector< std::array<double, 640> > history_points;
        std::vector< std::array<double, 640> > result;
        std::vector<size_t> result_index;
        std::queue< std::array<double, 640> > qe;  // 缓冲队列，不考虑前50帧
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            // pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(pc_to_read);
            
            pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
            pcl::ESFEstimation<PointType, pcl::ESFSignature640> ESF;
            ESF.setInputCloud(pc_to_read);
            ESF.compute(*esf_descriptor);
            // 将esf_descriptor转换为y方向的vector
            std::vector<float> esf_descriptor_y;
            for (int i = 0; i < 640; i++)
            {
                esf_descriptor_y.push_back(esf_descriptor->points[0].histogram[i]);
            }
            // 将esf_descriptor_x转换为point_type
            std::array<double, 640> point;
            for (int i = 0; i < 640; i++)
            {
                point[i] = esf_descriptor_y[i];
            }
            qe.push(point);
            history_points.push_back(point);
            if (qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(point, 1, result, result_index);
            int loopFrameId = result_index[0];
            double dist = 0;
            // 计算两个向量的L1距离
            std::array<double, 640> history = history_points[loopFrameId];
            for (int i = 0; i < 640; i++)
            {
                dist += fabs(point[i] - history[i]);
            }
            std::cout << "No." << pcd_number << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}


void LCDManager::JORD_M2DP(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/M2DP_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        KDTree<std::array<double, 192>, 192> kdtree;
        std::vector<std::array<double, 192> > history_points;
        std::vector<std::array<double, 192> > result;
        std::vector<size_t> result_index;
        std::queue<std::array<double, 192> > qe;  // 缓冲队列，不考虑前50帧
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            // pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(pc_to_read);
            
            M2DP m2dp(pc_to_read);
            Eigen::Matrix<double, 1, 192> desM2dp;
            desM2dp = m2dp.get_m2dp_result();
            std::array<double, 192> A_m2dp_point;
            for(int i = 0; i < 192; i++)
            {
                A_m2dp_point[i] = desM2dp(0, i);
            }
            qe.push(A_m2dp_point);
            history_points.push_back(A_m2dp_point);
            if(qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(A_m2dp_point, 1, result, result_index);
            int loopFrameId = result_index[0];
            // 计算两个向量的L2距离
            double dist = 0;
            std::array<double, 192> history = history_points[loopFrameId];
            for(int i = 0; i < 192; i++)
            {
                dist += pow(A_m2dp_point[i] - history[i], 2);
            }
            dist = sqrt(dist);

            std::cout << "No." << cloud_i << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}


void LCDManager::JORD_Scan_Context(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    //std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/01/";
    //std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/02/";
    //std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/03/";
    //std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/04/";
    //std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/05/";
    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    //std::ofstream loop_file("/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_JORD_01.txt", std::ios::out);
    //std::ofstream loop_file("/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_JORD_02.txt", std::ios::out);
    //std::ofstream loop_file("/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_JORD_03.txt", std::ios::out);
    //std::ofstream loop_file("/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_JORD_04.txt", std::ios::out);
    //std::ofstream loop_file("/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_JORD_05.txt", std::ios::out);
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        SCManager SC;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            std::cout << "No." << pcd_number << ": " << result.first << " " << result.second << std::endl;
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::JORD_Iris_KNN(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/Iris_JORD_KNN_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        int loop_event = 2;
        if(dataset_sequence == "00" || dataset_sequence == "05") {
            loop_event = 0;
        } else if(dataset_sequence == "08" || dataset_sequence == "01" || dataset_sequence == "02" || dataset_sequence == "03" || dataset_sequence == "04" || dataset_sequence == "05") {
            loop_event = 1;
        } else if(dataset_sequence == "02") {
            loop_event = 2;
        }

        LidarIris Iris(4, 18, 1.6, 0.75, 10, loop_event);
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;

            Iris.makeAndSaveLidarIrisAndKeys(laserCloudIn);
            std::pair<int, float> result = Iris.UpdateFrame();
            int loopFrameId = result.first;
            float min_dis = result.second;

            std::cout << "No." << pcd_number << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return ;
}

void LCDManager::JORD_Iris(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }
    
    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/Iris_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        int loop_event = 2;
        if(dataset_sequence == "00" || dataset_sequence == "05") {
            loop_event = 0;
        } else if(dataset_sequence == "08" || dataset_sequence == "01" || dataset_sequence == "02" || dataset_sequence == "03" || dataset_sequence == "04" || dataset_sequence == "05") {
            loop_event = 1;
        } else if(dataset_sequence == "02") {
            loop_event = 2;
        }
        LidarIris Iris(4, 18, 1.6, 0.75, loop_event);
        std::vector<LidarIris::FeatureDesc> dataset(file_list.size());
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {

            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;

            cv::Mat1b li1 = LidarIris::GetIris(laserCloudIn);
            LidarIris::FeatureDesc fd1 = Iris.GetFeature(li1);
            dataset[cloud_i] = fd1;

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            int loopFrameId = -1;
            float min_dis = 100000.0;
            for(size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
            {
                LidarIris::FeatureDesc fd2 = dataset[cloud_j];

                int bias;
                auto dis = Iris.Compare(fd1, fd2, &bias);
                if(dis < min_dis)
                {
                    min_dis = dis;
                    loopFrameId = cloud_j;
                }
            }
            if(loopFrameId == -1)
                min_dis = 0.0;

            std::cout << "No." << pcd_number << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}


void LCDManager::JORD_SC_Intensity(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/SC_Intensity_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        ISCManager SC_Intensity;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC_Intensity.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC_Intensity.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();
    return ;
}

void LCDManager::JORD_ISC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/ISC_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        int ring_height = 20;
        int sector_width = 60;
        double max_dis= 50.0;
        ISCGenerationClass ISC;
        // ISC.init_param(60, 60, 40);
        ISC.init_param(ring_height, sector_width, max_dis, 1);
        std::cout << "注意：如果点云的强度是1-255之间的整数格式，请在iscGenerationClass.cpp中取消注释#define INTEGER_INTENSITY并重新编译" << std::endl;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }
            // pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            // pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            ISC.makeAndSaveISC(pc_to_read);

            /*

            namespace fs = std::filesystem;

            // 生成二维ISC彩色图
            ISCDescriptor isc_color = ISC.getLastISCRGB();
            cv::resize(isc_color, isc_color, cv::Size(sector_width * 15, ring_height * 15));

            // 定义图片输出文件夹
            std::string color_outputPath = "/home/jlurobot/catkin_ws/src/JM_LCD/results/ISC_JORD_color_" + dataset_sequence + "/";
            // 如果图片输出文件夹不存在，则创建该文件夹
            fs::path folderPath = color_outputPath;
            if (!fs::exists(folderPath)) {
                if(fs::create_directory(folderPath)) {
                    std::cout << "Folder created successfully." << std::endl;
                } else {
                    std::cerr << "Failed to create folder." << std::endl;
                }
            }

            // Save the ISC color image to a file
            std::string filename = "isc_color_image" + to_string(cloud_i) + ".png";
            cv::imwrite(color_outputPath + filename, isc_color);

            // Optional: If you also want to show the image using OpenCV window
            // cv::namedWindow("ISC Color Image", cv::WINDOW_NORMAL);
            // cv::imshow("ISC Color Image", isc_color);
            // cv::waitKey(0); // Wait for a key press to close the window

            */

            std::pair<int, float> result = ISC.detectLoopClosureIDAndDis();

            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}


void LCDManager::JORD_Seed(std::string dataset_sequence)
{

    return;
}


void LCDManager::JORD_CSSC_Force(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/CSSC_JORD_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path_now = file_list[cloud_i].file_name;
            int pcd_number_now = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read_now(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path_now, *pc_to_read_now) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudInNow = *pc_to_read_now;

            double min_dis = 2.00;
            int loopFrameId = -1; 
            if(cloud_i >= 50) {  // 跳过前50帧
                for(size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
                {
                    string pcd_path_pre = file_list[cloud_j].file_name;
                    int pcd_number_pre = file_list[cloud_j].order_number;
                    pcl::PointCloud<PointType>::Ptr pc_to_read_pre(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
                    
                    // 加载PCD文件
                    if(pcl::io::loadPCDFile<PointType>(pcd_path_pre, *pc_to_read_pre) == -1) {
                        PCL_ERROR("Couldn't read PCD file.\n");
                        // return -1;
                        return ;
                    }

                    // pcl::PointCloud<pcl::PointXYZ> laserCloudInPre = *pc_to_read_pre;

                    csscM CSSC;  // 创建了一个CSSC对象，用于计算LiDAR点云之间的相似性
                    // 调用CSSC对象的 calculateDistanceBtnPC 函数，传入两个点云数据 cloud1 和 cloud2，计算它们之间的相似性。
                    // 该函数返回一个 std::pair<float, float> 类型的结果，其中first表示计算得到的相似性值，second表示对应的粗略偏航角。

                    // std::pair<double, int> result = CSSC.calculateDistanceBtnPC(pc_to_read_pre, pc_to_read_now);
                    std::pair<double, int> result = CSSC.calculateDistanceBtnPC(pc_to_read_pre, pc_to_read_now, 16);
                    if(result.first < min_dis) {
                        min_dis = result.first;
                        loopFrameId = pcd_number_pre;
                    }
                }
            }

            if(loopFrameId == -1)
                min_dis = 0.0;

            std::cout << "No." << pcd_number_now << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << pcd_number_now << " " << loopFrameId << " " << min_dis << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}

void LCDManager::JORD_CSSC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/CSSC_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        csscM CSSC;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            CSSC.makeAndSaveCSSCAndFingerprint(pc_to_read, 16);
            std::pair<int, float> result = CSSC.detectLoopClosureIDAndDis();
            std::cout << "No." << pcd_number << ": " << result.first << " " << result.second << std::endl;
            //loop_file << pcd_number + 1 << " " << result.first << " " << result.second << std::endl;
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return ;
}


void LCDManager::JORD_BoW3D(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/BoW3D_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {

        // 设置 LinK3D 的参数
        int nScans = 16;  // 激光雷达扫描线的数量
        float scanPeriod = 0.1;  // 激光雷达扫描周期
        float minimumRange = 0.1;  // 最小测距范围
        float distanceTh = 0.4;  // 距离阈值
        int matchTh = 6;  // 匹配阈值

        // 设置 BoW3D 的参数
        float thr = 3.5;  // 阈值
        int thf = 5;  // 阈值
        int num_add_retrieve_features = 5;  // 添加和检索特征的数量
        
        // 创建 LinK3D_Extractor 和 BoW3D 对象
        BoW3D::LinK3D_Extractor* pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
        BoW3D::BoW3D* pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {

            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;

            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;

            // 创建当前帧的Frame对象，并更新BoW3D
            BoW3D::Frame* pCurrentFrame = new BoW3D::Frame(pLinK3dExtractor, pc_to_read);

            int loopFrameId = -1;  // 回环帧ID
            int matchKeypointNums = 0;  // 匹配到的特征关键点数量
            Eigen::Matrix3d loopRelR;  // 旋转
            Eigen::Vector3d loopRelt;  // 平移

            if(pCurrentFrame->mnId < 50)  // 前50帧不考虑回环，如考虑，将50换成2
            {
                // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
                pBoW3D->update(pCurrentFrame);
                loop_file << pcd_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
                continue;
            }

            clock_t start, end;
            double time;
            start = clock();
            // 检索回环帧，并得到相对姿态信息
            // pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, matchKeypointNums, loopRelR, loopRelt);

            end = clock();
            time = ((double) (end - start)) / CLOCKS_PER_SEC;
            
            // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
            pBoW3D->update(pCurrentFrame);

            if(loopFrameId == -1)
            {
                cout << "-------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl;
            }
            else
            {
                cout << "--------------------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" <<  pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;
                
                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;
                                
                cout << "Loop Relative t: " << endl;
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
            }

            loop_file << pcd_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}


void LCDManager::JORD_NDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/NDD_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        myNDD::NDDManager NDD;
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);  // 这里不能用 ConstPtr 
            
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            NDD.makeAndSaveNDDAndKeys(laserCloudIn);
            std::pair<int, float> result = NDD.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();

    return;
}


void LCDManager::JORD_STD(std::string dataset_sequence)
{
    
    return;
}


void LCDManager::JORD_Contour_Context(std::string dataset_sequence)
{
    
    return;
}


void LCDManager::JORD_NDTMC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/" + dataset_sequence + "/";
    
    vector<DataFile> file_list;
    DataLoader data_loader;
    if(data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return ;
    }

    std::string loop_file_name = "/home/jlurobot/catkin_ws/src/JM_LCD/results/NDTMC_JORD_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if(loop_file.is_open()) {
        NDTMC ndtmc_manager(1.0);  // 创建NDTMC对象，传入分辨率参数
        for(size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;

            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>);
            // 加载PCD文件
            if(pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1) {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return ;
            }
            // pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            // pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            ndtmc_manager.makeAndSaveNDTScanContextAndKeys(pc_to_read);

            if(cloud_i < 50)  // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            double min_dis = 1000.00;
            int loopFrameId = -1;
            for(size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
            {
                std::pair<double, int> result = ndtmc_manager.distanceBtnNDTScanContext(ndtmc_manager.all_desc[cloud_i], ndtmc_manager.all_desc[cloud_j], 1);
                if(result.first < min_dis) {
                    min_dis = result.first;
                    loopFrameId = cloud_j;
                }
            }

            if(loopFrameId == -1)
                min_dis = 0.0;

            std::cout << "No." << pcd_number << ", loopFrameId: " << loopFrameId << ", dist: " << min_dis << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)min_dis << std::endl;
        }
    } else {
        printf("Outputfile can't open.\n");
        // return -1;
        return ;
    }
    loop_file.close();
    return;
}


