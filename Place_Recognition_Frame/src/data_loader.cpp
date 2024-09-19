#include "data_loader.h"

DataLoader::DataLoader()
{

}

DataLoader::~DataLoader()
{

}


int DataLoader::kitti_bin_loader(string data_path, string dataset_sequence, vector<DataFile>& file_list)
{
    // std::string data_path = "/home/jm/catkin_ws/src/JM_LCD/sample_data/KITTI/00/velodyne/000094.bin";
    // std::string data_root = "/home/jm/catkin_ws/data/kitti_bin/00/";
    std::string data_root = "/mnt/1t/dataset/KITTI/kitti_bin/00/";  // 由于使用了data_path，故暂时无用
    
    // 可以定义后面的后缀为*.exe，*.txt等来查找特定后缀的文件，*.*是通配符，匹配所有类型,路径连接符最好是左斜杠/，可跨平台
    //std::string search_path = data_path + "/*.bin";

    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(data_path.c_str()))) // 打开一个目录
    {
        cout<<"Folder doesn't Exist!"<<endl;
        return -1;
    }
    while((ptr = readdir(pDir))!= NULL) // 循环读取目录数据
    {
        /*
        if(strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            printf("d_name : %s\n", ptr->d_name); //输出文件名
            //filenames.push_back(path + "/" + ptr->d_name);
        }
        */
        if(string(ptr->d_name).find(".bin") != string::npos)
        {
            DataFile DF;
            DF.file_name = data_path + string(ptr->d_name);
            int num = 0;
            for(char c : ptr->d_name)
            {
                if(c == '.')
                    break;
                num = num * 10 + (c - '0');
            }
            if(dataset_sequence == "08")
                num -= 1100;
            DF.order_number = num;
            file_list.push_back(DF);
        }
    }
    closedir(pDir);  // 关闭目录指针

    sort(file_list.begin(), file_list.end(), [&](DataFile a, DataFile b){
        return a.order_number < b.order_number;
    });

    return 1;
}


int DataLoader::jord_pcd_loader(string data_path, vector<DataFile>& file_list)
{

    std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/00/";  // 由于使用了data_path，故暂时无用
    
    // 可以定义后面的后缀为*.exe，*.txt等来查找特定后缀的文件，*.*是通配符，匹配所有类型,路径连接符最好是左斜杠/，可跨平台
    //std::string search_path = data_path + "/*.bin";

    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(data_path.c_str()))) // 打开一个目录
    {
        cout<<"Folder doesn't Exist!"<<endl;
        return -1;
    }
    while((ptr = readdir(pDir))!= NULL) // 循环读取目录数据
    {
        /*
        if(strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            printf("d_name : %s\n", ptr->d_name); //输出文件名
            //filenames.push_back(path + "/" + ptr->d_name);
        }
        */
        if(string(ptr->d_name).find(".pcd") != string::npos)
        {
            DataFile DF;
            DF.file_name = data_path + string(ptr->d_name);
            int num = 0;
            for(char c : ptr->d_name)
            {
                if(c == '.')
                    break;
                num = num * 10 + (c - '0');
            }
            DF.order_number = num;
            file_list.push_back(DF);
        }
    }
    closedir(pDir);  // 关闭目录指针

    sort(file_list.begin(), file_list.end(), [&](DataFile a, DataFile b){
        return a.order_number < b.order_number;
    });

    return 1;
}