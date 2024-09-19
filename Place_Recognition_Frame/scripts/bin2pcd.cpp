// 简单的C++程序，可以将Kitti数据集中的二进制格式的bin文件转换为PCD文件
/*
这个程序使用PCL库读取和保存点云数据。在程序中，首先检查输入参数，然后打开输入文件并读取点云数据。
点云数据存储在pcl::PointCloudpcl::PointXYZI类型的指针中。最后，程序使用pcl::PCDWriter类将点云数据保存到输出文件中。
这个程序可以通过命令行传递输入和输出文件的路径，例如：
./bin_to_pcd input_file.bin output_file.pcd
其中，./bin_to_pcd是程序的可执行文件名，input_file.bin是输入的bin文件的路径，output_file.pcd是输出的PCD文件的路径。
在程序中，输入文件的路径可以通过argv[1]来获取，输出文件的路径可以通过argv[2]来获取。
*/

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    // 检查输入参数
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " input_file.bin output_file.pcd" << std::endl;
        return -1;
    }

    // 打开输入文件
    std::ifstream input_file(argv[1], std::ios::binary);
    if (!input_file.is_open()) {
        std::cerr << "Failed to open input file: " << argv[1] << std::endl;
        return -1;
    }

    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI point;
    while (input_file.read((char *)&point, sizeof(pcl::PointXYZI))) {
        cloud->push_back(point);
    }
    input_file.close();

    // 保存点云数据到输出文件
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>(argv[2], *cloud, false);

    std::cout << "Converted " << cloud->size() << " points from " << argv[1] << " to " << argv[2] << std::endl;

    return 0;
}


/*
另一种方法：
打开终端（Terminal），进入bin文件所在的目录。
使用PCL库提供的pcl_convert_pcd_ascii_binary命令将bin文件转换为PCD文件。命令格式如下：
pcl_convert_pcd_ascii_binary input_file.bin output_file.pcd 0
input_file.bin是要转换的bin文件，output_file.pcd是输出的PCD文件，0表示转换为二进制格式的PCD文件，如果要转换为ASCII格式的PCD文件，可以将0改为1。

等待转换完成后，使用PCL库提供的pcl_viewer命令查看转换后的PCD文件。命令格式如下：
pcl_viewer output_file.pcd
其中，output_file.pcd是要查看的PCD文件。
*/