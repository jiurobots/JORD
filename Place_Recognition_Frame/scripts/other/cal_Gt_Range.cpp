#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

struct pose {
	float x;
	float y;
};

int main()
{
	std::ifstream iFin("C:\\JLUdata\\yyy\\02.txt");
	std::ofstream out("C:\\JLUdata\\yyy\\gt_02.txt");
	std::vector<pose> vec;
	if (iFin)
	{
		std::string str;
		float x, y, z;
		pose temp;
		float r1, r2, r3, t1, r4, r5, r6, t2, r7, r8, r9, t3;
		while (std::getline(iFin, str))
		{
			//kitti
			std::stringstream ss(str);
			ss >> r1 >> r2 >> r3 >> t1 >> r4 >> r5 >> r6 >> t2 >> r7 >> r8 >> r9 >> t3;
			//jlu
			//std::stringstream word(str);
			//word >> x >> y >> z;
			//printf("x:%f; y:%f\n", x, y);
			temp.x = t1;
			temp.y = t3;
			vec.push_back(temp);
		}
		std::cout << vec.size() << std::endl;
		for (int i = 0; i < vec.size(); i++)
		{
			out << i;
			for (int j = 0; j < vec.size(); j++)
			{
				float dis = (vec[i].x - vec[j].x) * (vec[i].x - vec[j].x) + (vec[i].y - vec[j].y) * (vec[i].y - vec[j].y);
				if (dis <= 16.0f && i>j )
				{
					out << " " << j;
				}
			}
			out << "\n";
		}
	}
	return 0;
}