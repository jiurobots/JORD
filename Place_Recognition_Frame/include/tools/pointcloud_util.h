//
// Created by lewis on 8/24/22.
//

#ifndef CONT2_POINTCLOUD_UTIL_H
#define CONT2_POINTCLOUD_UTIL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 声明一个模板函数，用于读取KITTI数据集的二进制点云文件，并返回点云的共享指针
template<typename PointType>
typename pcl::PointCloud<PointType>::ConstPtr readKITTIPointCloudBin(const std::string &lidar_bin_path) {
	// 声明一个指向 PointCloud<PointType> 类型的共享指针 out_ptr，初始值为 nullptr
	typename pcl::PointCloud<PointType>::Ptr out_ptr = nullptr;

	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	// 分配一个 4MB 的缓冲区（只需要大约 130*4*4 KB 的空间）
	int num = 1000000;
	auto *data = (float *) malloc(num * sizeof(float));
	// pointers
	// 指针 px 指向 data 的首个元素
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	// 打开二进制文件流
	FILE *stream;
	stream = fopen(lidar_bin_path.c_str(), "rb");
	if (stream) {
		// 读取二进制数据到 data 缓冲区中，并返回实际读取的点数（每个点由 4 个 float 组成）
		num = fread(data, sizeof(float), num, stream) / 4;
		// 创建 PointCloud<PointType> 的共享指针 out_ptr，并预留空间
		out_ptr.reset(new pcl::PointCloud<PointType>());
		out_ptr->reserve(num);
		// 循环遍历读取的点数据，并将其添加到点云中
		for (int32_t i = 0; i < num; i++) {
			PointType pt;
			pt.x = *px;
			pt.y = *py;
			pt.z = *pz;

			// 判断 PointType 是否为 pcl::PointXYZI 类型
			if constexpr (std::is_same<PointType, pcl::PointXYZI>::value) {
				// PointType 是 pcl::PointXYZI 类型
				// 在这里可以进行针对 pcl::PointXYZI 类型的操作
				pt.intensity = *pr; // Add intensity
			} else {
				// PointType 不是 pcl::PointXYZI 类型
				// 在这里可以进行其他类型的操作
			}

			out_ptr->push_back(pt);

			// 更新指针位置，移动到下一个点的数据
			px += 4;
			py += 4;
			pz += 4;
			pr += 4;
		}
		// 关闭文件流
		fclose(stream);
	} else {
		// 如果文件无法打开，输出错误信息并退出程序
		printf("Lidar bin file %s does not exist.\n", lidar_bin_path.c_str());
		exit(-1);
	}
	// 释放分配的内存空间
	free(data);
	// 返回点云的共享指针
	return out_ptr;
}

#endif //CONT2_POINTCLOUD_UTIL_H
