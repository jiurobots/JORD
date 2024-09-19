#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>

struct pt {
    int index;
    double x;
    double y;
    double z;
};


void JORD_dataset(std::string pc_tum_gt_path, std::string lcd_gt_path)
{

    std::ifstream pc_tum_gt_file(pc_tum_gt_path, std::ios::in);
    std::ofstream loop_gt_file(lcd_gt_path, std::ios::out);
    // 文件是否能正常打开
    if(pc_tum_gt_file.is_open()) {
        int index = 0;
        std::string line;
        std::vector<pt> all_data;
        while(std::getline(pc_tum_gt_file, line)) {
            // printf("index = %d\n", index);
            // std::cout << line << std::endl;
            std::stringstream ss(line);
            std::vector<double> vt;
            std::string temp;
            while(std::getline(ss, temp, ' ')) {
                vt.push_back(stof(temp));
            }
            // double timestamp = vt[0];
            // double x = vt[1], y = vt[2], z = vt[3];
            // printf("%lf %lf %lf %lf\n",  timestamp, x, y, z);
            all_data.push_back({index, vt[1], vt[2], vt[3]});
            index++;
        }

        //printf("index = %d\n", index);
        //printf("all_data.size() = %d\n", all_data.size());

        for(int i = 0; i < index; i++) {
            loop_gt_file << i;
            for(int j = 0; j < index; j++) {
                double dis = (all_data[i].x - all_data[j].x) * (all_data[i].x - all_data[j].x) + (all_data[i].y - all_data[j].y) * (all_data[i].y - all_data[j].y);
                if(i != j && dis <= (double)16.0) {
                    loop_gt_file << " " << j;
                }
            }
            loop_gt_file << "\n";
        }

    }

    pc_tum_gt_file.close();
    loop_gt_file.close();

    return ;
}

int main(int argc, char const *argv[])
{
    //std::string pc_tum_gt_path = "/mnt/1t/dataset/JORD/jord_gt/jord_pc_tum_gt/jord_pc_tum_gt_051.txt";
    //std::string pc_tum_gt_path = "/mnt/1t/dataset/JORD/jord_gt/jord_pc_tum_gt/jord_pc_tum_gt_052.txt";
    //std::string pc_tum_gt_path = "/mnt/1t/dataset/JORD/jord_gt/jord_pc_tum_gt/jord_pc_tum_gt_061.txt";
    //std::string pc_tum_gt_path = "/mnt/1t/dataset/JORD/jord_gt/jord_pc_tum_gt/jord_pc_tum_gt_0621.txt";
    std::string pc_tum_gt_path = "/mnt/1t/dataset/JORD/jord_gt/jord_pc_tum_gt/jord_pc_tum_gt_0622.txt";

    // 定义真值文件路径
    std::string lcd_gt_path = "/home/jlurobot/catkin_ws/src/JM_LCD/results/lcd_gt.txt";

    try {
        JORD_dataset(pc_tum_gt_path, lcd_gt_path);
    } catch (const std:: exception & ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    printf("回环的真值文件已生成\n");

    return 0;
}
