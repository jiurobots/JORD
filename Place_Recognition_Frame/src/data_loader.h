#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

// #include <filesystem>

//#include <io.h>
#include <sys/io.h>
#include <dirent.h>

using namespace std;

struct DataFile
{
	std::string file_name;   /*!< 文件名 */
	int order_number;        /*!< 文件序号, 取文件名 */
};

class DataLoader
{

public:

    DataLoader();
    ~DataLoader();

    // KITTI
    int kitti_bin_loader(string data_path, string dataset_sequence, vector<DataFile> &file_list);

    // JORD
    int jord_pcd_loader(string data_path, vector<DataFile> &file_list);

};