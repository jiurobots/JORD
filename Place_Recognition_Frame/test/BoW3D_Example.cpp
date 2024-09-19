#include <ros/ros.h>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>

#include <sstream>
#include <iomanip>
#include "LinK3D_Extractor.h"
#include "BoW3D.h"


using namespace std;
using namespace BoW3D;

/*
    这段代码实现了一个BoW3D回环检测的程序。
    首先设置了LinK3D和BoW3D的参数。然后通过read_lidar_data函数从KITTI数据集中读取点云数据。
    在主循环中，读取点云数据，构建当前帧的点云对象，然后创建当前帧的Frame对象，并通过BoW3D更新当前帧。
    如果当前帧数小于2，则只更新BoW3D。如果当前帧数大于等于2，则通过BoW3D进行回环检测并获取回环帧的相对姿态信息。
    最后，打印回环检测的结果和相对姿态信息。程序会在循环中不断读取点云数据并进行回环检测，直到程序退出。
*/

// 首先设置 LinK3D 和 BoW3D 的参数

//Parameters of LinK3D
int nScans = 64; //Number of LiDAR scan lines  // 激光雷达扫描线的数量
float scanPeriod = 0.1;  // 激光雷达扫描周期
float minimumRange = 0.1;  // 最小测距范围
float distanceTh = 0.4;  // 距离阈值
int matchTh = 6;  // 匹配阈值

//Parameters of BoW3D
float thr = 3.5;  // 阈值
int thf = 5;  // 阈值
int num_add_retrieve_features = 5;  // 添加和检索特征的数量

//Here, the KITTI's point cloud with '.bin' format is read.
// 从 .bin 格式的KITTI点云文件中读取数据
vector<float> read_lidar_data(const std::string lidar_data_path)
{
    // 读取点云数据文件
    std::ifstream lidar_data_file;
    lidar_data_file.open(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    if(!lidar_data_file)
    {
        cout << "Read End..." << endl;
        exit(-1);
    }

    // 获取文件大小并设置文件指针到文件开头

    // 这里使用seekg函数将文件指针移动到文件的末尾。seekg函数用于在文件中定位文件指针的位置，
    // 它接受两个参数：第一个参数是相对于起始位置的偏移量，第二个参数是指定从哪个位置开始计算偏移量。
    // 在这里，第一个参数为0，表示相对于文件开头位置的偏移量为0，第二个参数std::ios::end表示从文件末尾开始计算偏移量。
    lidar_data_file.seekg(0, std::ios::end);
    // 使用tellg函数获取当前文件指针的位置，也就是文件的大小（以字节为单位）。
    // 由于每个浮点数占用4个字节，所以将文件大小除以sizeof(float)，得到文件中包含的浮点数元素个数，并将其存储在num_elements变量中。
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    // 最后，使用seekg函数将文件指针重置到文件的开头，以便后续读取LiDAR数据文件中的浮点数数据。
    lidar_data_file.seekg(0, std::ios::beg);

    // 创建一个vector来存储点云数据，并将数据读取到其中
    std::vector<float> lidar_data_buffer(num_elements);
    // 这是C++中的文件输入流（ifstream）的read函数。
    // 它接受两个参数：第一个参数是指向字符缓冲区的指针，用于存储从文件中读取的数据；第二个参数是要读取的字节数。
    // 这里使用reinterpret_cast进行类型转换，将指向lidar_data_buffer[0]的指针转换为char*类型，因为read函数要求提供一个char*类型的指针。
    // 这里是要读取的字节数，即从文件中读取的浮点数数据总共占用的字节数。
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}


int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "BoW3D");
    ros::NodeHandle nh;0

    /*Please replace the dataset folder path with the path in your computer. KITTI's 00, 02, 05, 06, 07, 08 have loops*/
    /*请使用计算机上的数据集文件夹路径替换此处路径。KITTI的00、02、05、06、07、08文件夹包含回环数据*/
    string dataset_folder;
    dataset_folder = "/home/cuiyunge/dataset/velodyne/"; //The last '/' should be added    // 文件夹路径末尾应该加上'/' 

    // 创建 LinK3D_Extractor 和 BoW3D 对象
    BoW3D::LinK3D_Extractor* pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
    BoW3D::BoW3D* pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

    size_t cloudInd = 0;

    // 设置LiDAR的频率为10Hz
    ros::Rate LiDAR_rate(10); //LiDAR frequency 10Hz
    while (ros::ok())
    {
        // 构建点云文件路径并读取点云数据
        std::stringstream lidar_data_path;  // 用于存储 LiDAR 数据文件的路径
        // dataset_folder是数据集文件夹的路径。
        // cloudInd是一个计数器，用于指示当前要读取的LiDAR数据文件。
        // std::setfill('0') 用于设置填充字符为 '0'，意味着当输出数字的位数不足时，使用 '0' 字符进行填充。
        // 通过std::setw(6)设置cloudInd的输出宽度为6位，不足6位的前面用0填充。
        // .bin是LiDAR数据文件的扩展名。最终，lidar_data_path将包含完整的文件路径。
        lidar_data_path << dataset_folder << std::setfill('0') << std::setw(6) << cloudInd << ".bin";  // 将LiDAR数据文件的路径拼接起来
        // 这里调用了read_lidar_data函数，将之前拼接好的LiDAR数据文件路径传递给该函数。
        // read_lidar_data函数将读取该文件，并将其中的浮点数数据存储在名为lidar_data的vector<float>中。
        vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        
        // 创建当前帧的点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(std::size_t i = 0; i < lidar_data.size(); i += 4)
        {            
            pcl::PointXYZ point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
    
            current_cloud->push_back(point);
        }

        // 创建当前帧的Frame对象，并更新BoW3D
        Frame* pCurrentFrame = new Frame(pLinK3dExtractor, current_cloud); 
            
        if(pCurrentFrame->mnId < 2)
        {
            // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
            pBoW3D->update(pCurrentFrame);  
        }
        else
        {                
            int loopFrameId = -1;  // 回环帧ID
            Eigen::Matrix3d loopRelR;  // 旋转
            Eigen::Vector3d loopRelt;  // 平移

            clock_t start, end;
            double time;       
            start = clock();
            // 检索回环帧，并得到相对姿态信息
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);

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
                cout << "Frame" <<  ->mnId << " Has Loop Frame" << loopFrameId << endl;
                
                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;
                                
                cout << "Loop Relative t: " << endl;                
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
            }
        }                       
        
        cloudInd ++;

        ros::spinOnce();
        LiDAR_rate.sleep();  // 每次循环后暂停一定的时间，以模拟特定的频率10Hz运行
        // LiDAR_rate.sleep() 会使程序暂停一段时间，以确保程序运行的频率不会超过10Hz，从而控制点云数据的处理速度。
        // 这样做是为了模拟LiDAR传感器的工作频率，并确保程序不会过于快速处理点云数据，以免导致资源浪费或不必要的计算。
    }

    return 0;
}

