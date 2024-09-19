#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <algorithm>
#include <vector>

#include <boost/program_options.hpp>

#include "sensor_msgs/PointCloud2.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lcd_manager.h"

using namespace std;
namespace po = boost::program_options;



int main(int argc, char const *argv[])
{

    //ros::init(argc, argv, "pub_a_lidar_msg");
    //ros::NodeHandle nh;

    std::string dataset_name = "";
    std::string dataset_sequence = "";
    std::string descriptor_name = "";

    // 声明命令行选项和选项参数
    po::options_description desc("Allowed options");
    // 使用po::value<std::string>()->default_value("JORD")设置默认参数
    // 使用po::value<int>(&requiredParam)->required()设置必需的参数
    desc.add_options() // 注意缩写前不能有空格
        ("help,h", "Produce help message")
        ("dataset,d", po::value<std::string>(&dataset_name)->required(), "The name of the dataset (capital letters)")
        ("sequence,s", po::value<std::string>(&dataset_sequence)->required(), "The sequence number of the dataset")
        ("descriptor,n", po::value<std::string>(&descriptor_name)->required(), "The name of the descriptor (capital letters)");

    // 解析命令行参数
    po::variables_map vm;

    try {
        // 解析命令行参数
        po::store(po::parse_command_line(argc, argv, desc), vm);
        // 将解析的参数存储到变量中
        po::notify(vm);

        // if(vm.count("help")) {
        //     std::cout << desc << std::endl;
        //     return 0;
        // }

        if(vm.count("dataset")) {
            std::cout << "Dataset: " << vm["dataset"].as<std::string>() << std::endl;
        } else {
            std::cerr << "Missing required parameter." << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }

        if(vm.count("sequence")) {
            std::cout << "Sequence: " << vm["sequence"].as<std::string>() << std::endl;
        } else {
            std::cerr << "Missing required parameter." << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }

        if(vm.count("descriptor")) {
            std::cout << "Descriptor: " << vm["descriptor"].as<std::string>() << std::endl;
        } else {
            std::cerr << "Missing required parameter." << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }
    } catch (const po::error& e) {
        // 输出错误信息到标准错误输出流 std::cerr
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    // std::cout << dataset_name << "   " << dataset_sequence << "   " << descriptor_name << std::endl;
    
    
    LCDManager lcd_manager;

    enum DescType
    {
        JM_LCD,
        ESF,
        M2DP,
        SC,
        Iris,
        ISC,
        Seed,
        CSSC,
        FreSCo,
        BoW3D,
        NDD,
        STD,
        Cont2,
        NDTMC
    };

    unordered_map<string, DescType> Control_Param = {
        {"JM_LCD", JM_LCD},
        {"ESF", ESF},  // ok
        {"M2DP", M2DP},  // ok
        {"SC", SC},  // ok
        {"Iris", Iris},  // ok
        {"ISC", ISC},  // ok
        {"Seed", Seed},
        {"CSSC", CSSC},  // ok
        {"FreSCo", FreSCo},
        {"BoW3D", BoW3D},  //
        {"NDD", NDD},  // ok
        {"STD", STD},
        {"CC", Cont2},
        {"NDTMC", NDTMC}
    };

    if(dataset_name == "KITTI") {
        if(dataset_sequence != "00" && dataset_sequence != "02" && dataset_sequence != "05" && dataset_sequence != "08") {
            std::cout << "Invalid sequence number, please select in [\"00\", \"02\", \"05\", \"08\"]" << std::endl;
            return -1;
        }
        DescType caseValue = Control_Param[descriptor_name];
        switch (caseValue)
        {
        case JM_LCD:
            /* code */
            lcd_manager.KITTI_test(dataset_sequence);
            break;
        case ESF:
            lcd_manager.KITTI_ESF(dataset_sequence);
            break;
        case M2DP:
            lcd_manager.KITTI_M2DP(dataset_sequence);
            break;
        case SC:
            lcd_manager.KITTI_Scan_Context(dataset_sequence);
            break;
        case Iris:
            // lcd_manager.KITTI_Iris_KNN(dataset_sequence);
            lcd_manager.KITTI_Iris(dataset_sequence);
            break;
        case ISC:
            // lcd_manager.KITTI_SC_Intensity(dataset_sequence);
            lcd_manager.KITTI_ISC(dataset_sequence);
            break;
        case Seed:
            /* code */
            break;
        case CSSC:
            // lcd_manager.KITTI_CSSC_Force(dataset_sequence);
            lcd_manager.KITTI_CSSC(dataset_sequence);
            break;
        case FreSCo:
            /* code */
            break;
        case BoW3D:
            lcd_manager.KITTI_BoW3D(dataset_sequence);
            break;
        case NDD:
            lcd_manager.KITTI_NDD(dataset_sequence);
            break;
        case STD:
            /* code */
            break;
        case Cont2:
            /* code */
            break;
        case NDTMC:
            lcd_manager.KITTI_NDTMC(dataset_sequence);
            break;
        default:
            break;
        }
    } else if(dataset_name == "JORD") {
        if(dataset_sequence != "01" && dataset_sequence != "02" && dataset_sequence != "03" && dataset_sequence != "04" && dataset_sequence != "05") {
            std::cout << "Invalid sequence number, please select in [\"01\", \"02\", \"03\", \"04\", \"05\"]" << std::endl;
            return -1;
        }
        DescType caseValue = Control_Param[descriptor_name];
        switch (caseValue)
        {
        case JM_LCD:
            /* code */
            break;
        case ESF:
            lcd_manager.JORD_ESF(dataset_sequence);
            break;
        case M2DP:
            lcd_manager.JORD_M2DP(dataset_sequence);
            break;
        case SC:
            lcd_manager.JORD_Scan_Context(dataset_sequence);
            break;
        case Iris:
            // lcd_manager.JORD_Iris_KNN(dataset_sequence);
            lcd_manager.JORD_Iris(dataset_sequence);
            break;
        case ISC:
            // lcd_manager.JORD_SC_Intensity(dataset_sequence);
            lcd_manager.JORD_ISC(dataset_sequence);
            break;
        case Seed:
            /* code */
            break;
        case CSSC:
            // lcd_manager.JORD_CSSC_Force(dataset_sequence);
            lcd_manager.JORD_CSSC(dataset_sequence);
            break;
        case FreSCo:
            /* code */
            break;
        case BoW3D:
            lcd_manager.JORD_BoW3D(dataset_sequence);
            break;
        case NDD:
            lcd_manager.JORD_NDD(dataset_sequence);
            break;
        case STD:
            /* code */
            break;
        case Cont2:
            /* code */
            break;
        case NDTMC:
            /* code */
            lcd_manager.JORD_NDTMC(dataset_sequence);
            break;
        default:
            break;
        }
    }
    




    //kdtree_test();


    /*
    //ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/vis_a_pc", 10000);

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc_to_show = readKITTIPointCloudBin<pcl::PointXYZI>(data_path);

    if(!pc_to_show) {
        printf("No pc loaded.\n");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZI> laserCloudIn = *pc_to_show;

    TicToc t_tmp;

    int cloudSize = laserCloudIn.points.size();
    for(int i = 0; i < cloudSize; i++)
    {
        pcl::PointXYZI point;
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        // printf("x = %.3lf, y = %.3lf, z = %.3lf\n", point.x, point.y, point.z);
    }

    printf("read a point bin file cost %.3lf ms.\n", t_tmp.toc());

    */

    /*
    sensor_msgs::PointCloud2 pc_msg_to_pub;

    pcl::toROSMsg(*pc_to_show, pc_msg_to_pub);

    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        pc_msg_to_pub.header.stamp = ros::Time::now();
        pc_msg_to_pub.header.frame_id = "world";
        pub_cloud.publish(pc_msg_to_pub);
        printf("%lu\n", pc_msg_to_pub.data.size());
        rate.sleep();
    }
    */

    return 0;
}
