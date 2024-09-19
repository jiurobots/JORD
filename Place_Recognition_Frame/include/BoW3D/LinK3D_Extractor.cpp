#include "LinK3D_Extractor.h"


namespace BoW3D
{
    LinK3D_Extractor::LinK3D_Extractor(
            int nScans_, 
            float scanPeriod_, 
            float minimumRange_, 
            float distanceTh_,           
            int matchTh_):
            nScans(nScans_), 
            scanPeriod(scanPeriod_), 
            minimumRange(minimumRange_), 
            distanceTh(distanceTh_),          
            matchTh(matchTh_)
            {
                scanNumTh = ceil(nScans / 6);
                ptNumTh = ceil(1.5 * scanNumTh);
            }

    // 移除范围过近的无效点，对距离小于阈值的点云进行滤除（这样可以减少点云中噪声和冗余点，提高后续处理的效率和准确性）
    void LinK3D_Extractor::removeClosedPointCloud(
            const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
            pcl::PointCloud<pcl::PointXYZ> &cloud_out)
    {
        // 这里初始化返回点云。假如输入输出点云为不同的对象，则需要将输出点云的时间戳和容器大小与输入点云同步
        if (&cloud_in != &cloud_out)
        {
            // 将输入点云的头信息（header）赋值给输出点云。点云的头信息包含了一些元数据，例如时间戳、坐标系等。
            cloud_out.header = cloud_in.header;  // 统一header(时间戳)
            // 预先分配空间以避免添加的点的时候造成频繁的空间分配
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;  // 使用 size_t 类型而不使用 int 是为了和cloud_in.points.size()做比较时类型一致
        // 把点云距离小于给定阈值 minimumRange 的点去除掉
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            // 该点到原点的距离是小于指定的 minimumRange ，为过近点，视为无效点，滤去
            if (cloud_in.points[i].x * cloud_in.points[i].x 
                + cloud_in.points[i].y * cloud_in.points[i].y 
                + cloud_in.points[i].z * cloud_in.points[i].z 
                < minimumRange * minimumRange)
            {
                continue;
            }
                
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }

        // 如果有点被剔除时，那么size肯定会改变，则需要重新调整输出容器大小
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        // 这里是对每条扫描线上的点云进行直通滤波，因此设置点云的高度为1，宽度为数量，稠密点云
        // 过滤掉之后每一条线的激光雷达数量不一定，所以没有办法通过宽和高区分线，因此这里不做特殊处理
        cloud_out.height = 1;  // 数据行数，默认1为无组织的数据，说明是无结构的点云，即无序点云
        cloud_out.width = static_cast<uint32_t>(j);  // 可以理解成输出点云中有效点的数量
        // is_dense用来判断points中的数据是否是有限的（有限为true）或者说是判断点云中的点是否包含 Inf/NaN 这种值（包含为false）。
        cloud_out.is_dense = true;  // 指定点云中的所有数据都是有效的（注意之前已经调用pcl库的函数去除掉了点云中的NaN点，然后现在又去除了点云中距离小于阈值的点）。
    }

    // 提取边缘特征点
    void LinK3D_Extractor::extractEdgePoint(
            pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn,
            ScanEdgePoints &edgePoints)
    {
        // 存储每条线对应的起始和结束索引，每条雷达扫描线上的可以计算曲率的点云点的起始索引和结束索引分别用scanStartInd数组和scanEndInd数组记录
        vector<int> scanStartInd(nScans, 0);
        vector<int> scanEndInd(nScans, 0);

        // 创建临时点云对象并将输入点云复制给它
        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        laserCloudIn = *pLaserCloudIn;


        // 对点云进行预处理，去除掉不合要求的点
        // 1、去除点云中的NaN值并将处理后的点云保存在同一个点云对象中
        vector<int> indices;  // 这个变量保存了下面去除nan点的序号
        // 调用pcl库的函数去除掉点云中的NaN(Not a number)点（无效点）
        // 第一个参数是输入，第二个参数是输出，第三个参数是列表保存输出的点在输入里的位置
        // 输出里的第i个点，是输入里的第indices[i]个点，即cloud_out.points[i] = cloud_in.points[indices[i]]
        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        // 2、去除点云中距离小于阈值的点，这里调用作者写的函数removeClosedPointCloud
        removeClosedPointCloud(laserCloudIn, laserCloudIn);


        /*
        为了使点云有序，需要做到两件事：为每个点找到它所对应的扫描线（SCAN）；为每条扫描线上的点分配时间戳。
        要计算每个点的时间戳，首先我们需要确定这个点的角度范围。可以使用<cmath>中的atan2( )函数计算点云点的水平角度。下面要计算点云角度范围。
        这块在后面的激光框架基本都不需要了，因为后面雷达的驱动将每个点的线号、角度、时间戳都发出来了。所以说这一块可以跳过不看。
        */

        // 获取点云中点的数量
        int cloudSize = laserCloudIn.points.size();

        // startOri和endOri分别为起始点和终止点的方位角（与x轴的夹角），为了给后面计算相对起点的时间戳用的
        // atan2()函数是atan(y, x)函数的增强版，不仅可以求取arctan(y/x)还能够确定象限（atan2的输出为-pi到pi、atan输出为-pi/2到pi/2）
        // 通常激光雷达扫描方向是顺时针旋转，这里在取 atan 的基础上先取反（相当于转成了逆时针），arctan(-x) = -arctan(x)
        // 这样起始点理论上为 -pi，结束点为 pi，更符合直观
        // 理论上起始点和结束点的差值应该是 0，为了显示两者区别，将结束点的方向补偿2pi，表示结束点和起始点实际上逆时针经过一圈

        // 计算点云的起始角度和结束角度
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;  // atan2范围是[-Pi,PI]，这里加上2PI是为了保证起始到结束相差2PI符合实际
    
        // 处理几种个别情况，比如这里大于3PI，和小于PI，就需要调整到合理范围
        // 以保持结束点的朝向和起始点的方向差始终在 [pi, 3pi] 之间 （实际上是 2pi 附近）
        if (endOri - startOri > 3 * M_PI)
        {
            // case 1: 起始点在 -179°，结束点在 179°，补偿 2pi 后相差超过一圈，实际上是结束点相对于起始点还没转完整一圈
            // 因此将结束点的 2pi 补偿去掉为 179°，与起始点相差 358°，表示结束点和起始点差别是一圈少2°
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            // case 2: 起始点为 179°，结束点为 -179°，补偿后结束点为 181°，此时不能反映真实差别，需要对结束点再补偿上 2pi，表示经过了一圈多 2°
            endOri += 2 * M_PI;
        }
        
        // ------------------------ 为点云的点找到对应的扫描线，每条扫描线都有它固定的俯仰角，我们可以根据点云点的垂直角度为其寻找对应的扫描线 ------------------------

        /*
        LOAM利用了旋转式3D激光雷达分线扫描的特性，将原始激光雷达点云分解成了若干扫描线对应点云的集合。
        在这一过程中，LOAM利用了Velodyne VLP-16，HDL-32E，HDL-64三款激光雷达相邻线束之间的垂直夹角为定值的特性。
        对于扫描线垂直分布不均匀的激光雷达如Velodyne HDL-32C，则不能采用这种方法，需要自己根据扫描线分布，计算扫描点对应的扫描线。
        此外，LOAM还利用了旋转式3D激光雷达顺序扫描的特性，计算了每一个激光雷达点相对于该帧起始时间的相对测量时间，为后续去点云畸变做准备。
        */

        /*
            （假设64线Lidar的水平角度分辨率是0.2deg，则每个scan理论有360/0.2=1800个点，为方便叙述，我们称每个scan点的ID为fireID，即fireID [0~1799]，相应的scanID [0~63] ）
            接下来通过Lidar坐标系下点的仰角以及水平夹角计算点云的scanID和fireID，需要注意的是这里的计算方式其实并不适用kitti数据集的点云，因为kitti的点云已经被运动补偿过。
        */

        bool halfPassed = false;  // 过半记录标志
        int count = cloudSize;  // 记录总点数
        pcl::PointXYZI point;  // 临时变量
        // 将点云按扫描线分别存储在一个子点云中
        vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(nScans);
        
        for (int i = 0; i < cloudSize; i++)  // 遍历点云中的每个点
        {
            // 小技巧，对临时变量 Point 只初始化一次减少空间分配
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            
            // 计算激光点的俯仰角（单位是角度），通过计算垂直视场角确定激光点在哪个扫描线上（nScans线激光雷达）
            // 求仰角atan输出为-pi/2到pi/2，实际看scanID应该每条线之间差距是2度
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;  // 线束的ID

            // 计算是第几个线束 scan
            // 根据激光雷达类型与参数确定当前点所属的扫描线编号
            if (nScans == 16)
            {
                // 如果是16线激光雷达，结算出的angle应该在-15~15之间，+-15°的垂直视场，垂直角度分辨率2°，则-15°时的scanID = 0
                // velodyne 激光雷达的竖直 FoV 是[-15, 15]，分辨率是 2°（垂直一共30°  每两条线之间的夹角是2°）
                // 这里通过这样的计算可以对该激光点分配一个 [0, 15] 的扫描线ID
                scanID = int((angle + 15) / 2 + 0.5);
                // 如果点的距离值不准有可能会计算出不在范围内的 ID 此时不考虑这个点
                if (scanID > (nScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (nScans == 32)  // 32线激光雷达
            {
                // 思路和 16 线的情况一致
                // 垂直视场角+10.67~-30.67°，垂直角度分辨率1.33°，-30.67°时的scanID = 0
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (nScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (nScans == 64)  // 64线激光雷达
            {   
                // 和 16 线的情况一致
                // 垂直视场角+2~-24.8°，垂直角度分辨率0.4254°，+2°时的scanID = 0
                if (angle >= -8.83)  // scanID的范围是 [0,32]
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else  // scanID的范围是 [33,]
                    scanID = nScans / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies 
                // 不考虑扫描线 id 在50以上的点
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
            }

            /*
            // 每个sweep不一定从水平0°开始，没有复位过程，雷达只在电机旋转到接近0°时记下当前对应激光点的精确坐标；
            // 同样，结束的位置，也不一定是0°，都是在0°附近，这为只使用激光点坐标计算水平角度带来一定的复杂性；
            // 若起始角度是0°，结束角度是10°+2*pi，若只通过坐标来计算水平角度，如果得到的角度是5°，那么这个激光点是开始的时候（5°）扫描的，还是结束的时候（2*pi+5°）时扫描的？
            // 所以只使用XYZ位置有时候无法得到其精确的扫描时间，还需要结合时序信息，因为一个sweep中返回的点是按照时间的顺序排列的。
            // 这里变量halfPassed就是来解决这个问题的
            */

            // 计算激光点水平方向角，通过取反操作可以将雷达扫描方向为逆时针（-pi 到 pi）
            float ori = -atan2(point.y, point.x);  // 计算水平角
            // 把计算的水平角 放到 开始角度startOri和结束角度endOri 合理 的区间之内
            // 根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿，如果此时扫描没有过半，则halfPassed为false
            if (!halfPassed)  // 主要有 -pi 到 pi 的区间, 分成两个半圆算的
            { 
                // 对一些 Corner case 处理，确保-PI / 2 < ori - startOri < 3 / 2 * PI， 如果ori-startOri小于-0.5pi或大于1.5pi，则调整ori的角度
                if (ori < startOri - M_PI / 2)
                {
                    // case 1：起始点在 179 °，逆时针转过几度后，当前点是 -179°，需要加上 2pi 作为补偿
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    // case 2: 理论上在逆时针转的时候不会出现这种情况，在顺时针的情况下，起始点在 -179°，
                    // 顺时针转过两度后到 179°，此时差距过大，需要减去 2pi
                    ori -= 2 * M_PI;
                }

                // 扫描点过半则设定halfPassed为true，如果超过180度，就说明过了一半了
                if (ori - startOri > M_PI)
                {
                    // 角度校正后如果和起始点相差超过 pi，表示已经过半圈
                    halfPassed = true;
                }
            }
            else
            {
                // 确保-PI * 3 / 2 < ori - endOri < PI / 2
                // 经过半圈后，部分情况（扫描线从 179°到 -179°时）需要加 2pi
                ori += 2 * M_PI;  // 先补偿2PI
                if (ori < endOri - M_PI * 3 / 2)
                {
                    // case 1: 在逆时针下理论上不会出现
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    // case 2: 起始点在 -179°，逆时针经过半圈后当前点在 1°，
                    // 此时差值是对的，因此将2pi补偿减去
                    ori -= 2 * M_PI;
                }
            }

            // 将点的水平角度作为点云中点的强度信息，并将点添加到相应的扫描线点云中
            point.intensity = ori;
            // 根据每条scan的ID送入各自数组，表示这一条扫描线上的点
            laserCloudScans[scanID].points.push_back(point);  // 按线分类保存        
        }
        // 获取扫描线点云的数量
        size_t scanSize = laserCloudScans.size();
        // 调整输出边缘点云容器的大小为扫描线点云的数量
        edgePoints.resize(scanSize);
        // 重新设置点云的数量
        cloudSize = count;
        
        // 遍历扫描线点云，提取边缘点
        // 计算每一个点的曲率，这里的laserCloudScans是有序的点云，故可以直接这样计算
        // 十点求曲率，自然是在一条线上的十个点
        for(int i = 0; i < nScans; i++)
        {
            int laserCloudScansSize = laserCloudScans[i].size();
            if(laserCloudScansSize >= 15)
            {
                for(int j = 5; j < laserCloudScansSize - 5; j++)
                {
                    // 计算当前点和周围十个点（左右各 5 个）在 x, y, z 方向上的差值： 10*p_i - sum(p_{i-5:i-1,i+1:i+5})
                    // 注意这里对每一条扫描线的边缘的计算是有问题的，因为会引入相邻扫描线的点，但见上面操作，每一条扫描线我们不考虑边缘的五个点，所以为了方便可以这么操作
                    // 这里的计算曲率就是对应的论文中的公式，在代码里就是分别求一个轴维度的曲率。以x轴为例，就是前五个点x的值+后五个点x的值-10*当前点x的值
                    float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x
                                + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x
                                + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x
                                + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x
                                + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x
                                + laserCloudScans[i].points[j + 5].x;
                    float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y
                                + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y
                                + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y
                                + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y
                                + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y
                                + laserCloudScans[i].points[j + 5].y;
                    float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z
                                + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z
                                + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z
                                + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z
                                + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z
                                + laserCloudScans[i].points[j + 5].z;
                    // 确定每一个点对应的离散曲率
                    float curv = diffX * diffX + diffY * diffY + diffZ * diffZ;
                    
                    // 判断是否是边缘点，并初始化特征点的相关参数
                    if(curv > 10 && curv < 20000)
                    {
                        /*
                        * relTime 是一个0~1之间的小数，代表占用一帧扫描时间的比例，乘以扫描时间得到真实扫描时刻。
                        * scanPeriod扫描时间默认为0.1s
                        * 角度的计算是为了计算相对起始时刻的时间，为点云畸变补偿使用
                        */
                        float ori = laserCloudScans[i].points[j].intensity;
                        // 估计当前当前点和起始点的时间差
                        //看看旋转多少了，记录比例relTime
                        float relTime = (ori - startOri) / (endOri - startOri);  // 计算当前点 在起始和结束之间的比率

                        // 创建临时点对象，并保存边缘点的坐标、扫描线位置、曲率和水平角度
                        PointXYZSCA tmpPt;
                        tmpPt.x = laserCloudScans[i].points[j].x;
                        tmpPt.y = laserCloudScans[i].points[j].y;
                        tmpPt.z = laserCloudScans[i].points[j].z;
                        // 小技巧：用 scan_position 的整数部分和小数部分来存储该点所属的扫描线以及相对时间：
                        // [线id].[相对时间*扫描周期]
                        // 整数部分是scan线束的索引(id)，小数部分是相对起始时刻的时间
                        tmpPt.scan_position = i + scanPeriod * relTime;
                        tmpPt.curvature = curv;
                        tmpPt.angle = ori;
                        // 根据每条scan的ID送入各自扫描线数组中，表示这一条扫描线上的点
                        edgePoints[i].emplace_back(tmpPt);  // 按线分类保存
                    }
                }
            }
        }
    }

    /*
        使用角度信息引导，将潜在点聚集在小群体中，而不是聚集在整个区域中。其动机是具有大致相同水平角度的点更有可能位于同一簇上。
        具体来说，我们首先将以 LiDAR 坐标系原点为中心的水平面平均划分为 N_sect 扇区，然后只对每个扇区中的点进行聚类。
    */
    //Roughly divide the areas to save time for clustering.
    // 将提取的边缘点根据其水平角度大致划分到不同的扇区中，以节省聚类时间
    void LinK3D_Extractor::divideArea(ScanEdgePoints &scanCloud, ScanEdgePoints &sectorAreaCloud)
    {
        // 水平面被划分为以激光雷达坐标为中心的120个扇区区域。
        sectorAreaCloud.resize(120); //The horizontal plane is divided into 120 sector area centered on LiDAR coordinate.
        int numScansPt = scanCloud.size();  // 获取边缘点容器中的扫描线数量
        if(numScansPt == 0)  // 如果没有边缘点，则直接返回
        {
            return;
        }
        // 遍历每个扫描线
        for(int i = 0; i < numScansPt; i++) 
        {
            int numAScanPt = scanCloud[i].size();  // 获取当前扫描线中边缘点的数量
            // 遍历当前扫描线中的边缘点
            for(int j = 0; j < numAScanPt; j++)
            {
                int areaID = 0;  // 初始化扇区 ID 为0，然后根据边缘点的水平角度来确定其所属的扇区
                float angle = scanCloud[i][j].angle;
                // 将角度映射到0~119之间，共120个扇区
                if(angle > 0 && angle < 2 * M_PI)
                {
                    areaID = std::floor((angle / (2 * M_PI)) * 120);
                }   
                else if(angle > 2 * M_PI)
                {
                    areaID = std::floor(((angle - 2 * M_PI) / (2 * M_PI)) * 120);
                }
                else if(angle < 0)
                {
                    areaID = std::floor(((angle + 2 * M_PI) / (2 * M_PI)) * 120);
                }
                // 将边缘点添加到相应的扇区中
                sectorAreaCloud[areaID].push_back(scanCloud[i][j]);
            }
        }
    }

    // 求聚类簇到中心点的平均距离
    float LinK3D_Extractor::computeClusterMean(vector<PointXYZSCA> &cluster)
    {        
        float distSum = 0;
        int numPt = cluster.size();

        for(int i = 0; i < numPt; i++)
        {
            distSum += distXY(cluster[i]);
        }

        return (distSum/numPt);
    }

    // 求聚类簇X、Y的平均值
    void LinK3D_Extractor::computeXYMean(vector<PointXYZSCA> &cluster, std::pair<float, float> &xyMeans)
    {         
        int numPt = cluster.size();
        float xSum = 0;
        float ySum = 0;

        for(int i = 0; i < numPt; i++)
        {
            xSum += cluster[i].x;
            ySum += cluster[i].y;
        }

        float xMean = xSum/numPt;
        float yMean = ySum/numPt;
        xyMeans = std::make_pair(xMean, yMean);
    }

    // 将扇区中的边缘点进行聚类操作，将满足一定条件的边缘点聚合成一个簇
    void LinK3D_Extractor::getCluster(const ScanEdgePoints &sectorAreaCloud, ScanEdgePoints &clusters)
    {    
        ScanEdgePoints tmpclusters;  // 临时存储聚类结果的容器
        PointXYZSCA curvPt;  // 创建一个空的 PointXYZSCA 类型变量 curvPt
        vector<PointXYZSCA> dummy(1, curvPt);  // 创建一个只包含一个 curvPt 的 vector 容器，用于后续操作

        int numArea = sectorAreaCloud.size();  // 获取扇区的数量

        //Cluster for each sector area.
        for(int i = 0; i < numArea; i++)  // 遍历每个扇区
        {
            // 如果扇区中的边缘点数量小于 6，则跳过该扇区，不进行聚类
            if(sectorAreaCloud[i].size() < 6)
                continue;

            int numPt = sectorAreaCloud[i].size();  // 获取当前扇区中的边缘点数量
            ScanEdgePoints curAreaCluster(1, dummy);  // 初始化当前扇区聚类的第一个簇
            curAreaCluster[0][0] = sectorAreaCloud[i][0];  // 相当于先把该区域的第1个点放入

            for(int j = 1; j < numPt; j++)  // 遍历当前扇区中剩下的每个边缘点
            {
                int numCluster = curAreaCluster.size();  // 获取当前扇区簇的数量

                for(int k = 0; k < numCluster; k++)  // 遍历当前扇区的每个簇
                {
                    float mean = computeClusterMean(curAreaCluster[k]);  // 求聚类簇中的点到中心点的平均距离
                    std::pair<float, float> xyMean;
                    computeXYMean(curAreaCluster[k], xyMean);  // 求聚类簇中的点X、Y的平均值
                    
                    PointXYZSCA tmpPt = sectorAreaCloud[i][j];  // 用一个临时变量来获取当前边缘点
                    
                    // 如果当前边缘点满足一定条件（当前边缘点与簇中平均值的差值小于距离阈值 distanceTh），则将其加入当前簇中
                    if(abs(distXY(tmpPt) - mean) < distanceTh 
                        && abs(xyMean.first - tmpPt.x) < distanceTh 
                        && abs(xyMean.second - tmpPt.y) < distanceTh)
                    {
                        curAreaCluster[k].emplace_back(tmpPt);
                        break;
                    }
                    else if(abs(distXY(tmpPt) - mean) >= distanceTh && k == numCluster-1)  // 该点不属于之前的任何一个簇
                    {
                        curAreaCluster.emplace_back(dummy);  // 新增一个簇
                        curAreaCluster[numCluster][0] = tmpPt;  // 将当前边缘点加入其中
                    }
                    else
                    { 
                        continue; 
                    }                    
                }
            }

            // 对于当前扇区，将满足条件的簇加入到临时聚类结果容器 tmpclusters 中
            int numCluster = curAreaCluster.size();
            for(int j = 0; j < numCluster; j++)
            {
                int numPt = curAreaCluster[j].size();
                // 簇中点的数量太少，丢弃该簇
                if(numPt < ptNumTh)
                {
                    continue;
                }
                tmpclusters.emplace_back(curAreaCluster[j]);
            }
        }

        int numCluster = tmpclusters.size();  // 簇的数量
        
        vector<bool> toBeMerge(numCluster, false);  // 是否合并
        multimap<int, int> mToBeMergeInd;
        set<int> sNeedMergeInd;

        //Merge the neighbor clusters.
        // 合并相邻聚类
        for(int i = 0; i < numCluster; i++)
        {
            if(toBeMerge[i]) {
                continue;
            }
            float means1 = computeClusterMean(tmpclusters[i]);  // 求聚类簇中的点到中心点的平均距离
            std::pair<float, float> xyMeans1;
            computeXYMean(tmpclusters[i], xyMeans1);  // 求聚类簇中的点X、Y的平均值

            for(int j = 1; j < numCluster; j++)  // ? 回头需测试一下 j = i + 1
            {
                if(toBeMerge[j]) {
                    continue;
                }

                float means2 = computeClusterMean(tmpclusters[j]);
                std::pair<float, float> xyMeans2;
                computeXYMean(tmpclusters[j], xyMeans2);

                // 如果当前两个簇满足一定条件（两个簇中平均值的差值小于 2*distanceTh），则认为这两个簇可以合并
                if(abs(means1 - means2) < 2*distanceTh 
                    && abs(xyMeans1.first - xyMeans2.first) < 2*distanceTh 
                    && abs(xyMeans1.second - xyMeans2.second) < 2*distanceTh)
                {
                    mToBeMergeInd.insert(std::make_pair(i, j));
                    sNeedMergeInd.insert(i);
                    toBeMerge[i] = true;
                    toBeMerge[j] = true;
                }
            }
        }

        // 如果没有需要合并的簇，则直接将 tmpclusters 中的簇加入到最终的聚类结果容器 clusters 中
        if(sNeedMergeInd.empty())  // 不需要合并
        {
            for(int i = 0; i < numCluster; i++)
            {
                clusters.emplace_back(tmpclusters[i]);
            }
        }
        else  // 如果存在需要合并的簇，则根据合并关系将相应的簇进行合并，并将最终的簇结果加入到 clusters 中
        {  // 需要合并（感觉可以优化）
            for(int i = 0; i < numCluster; i++)
            {
                if(toBeMerge[i] == false)
                {
                    clusters.emplace_back(tmpclusters[i]);
                }
            }
            
            for(auto setIt = sNeedMergeInd.begin(); setIt != sNeedMergeInd.end(); ++setIt)
            {
                int needMergeInd = *setIt;
                auto entries = mToBeMergeInd.count(needMergeInd);
                // 调用 multimap.find() 返回的是一个迭代器范围，可以通过遍历该范围来处理所有具有给定键的元素。
                // std::multimap 的迭代器 std::multimap::iterator 是一个指向 键-值 对的指针。
                // 这个键-值对由 .first 和 .second 成员组成，它们分别表示迭代器指向的元素的键和值。
                auto iter = mToBeMergeInd.find(needMergeInd);
                vector<int> vInd;

                while(entries)
                {
                    // 因为 std::multimap 允许多个具有相同键的元素，所以在遍历 multimap 时，iterator.second 将指向一系列具有相同键的值，你可以通过迭代 iterator.second 来处理这些值。
                    int ind = iter->second;
                    vInd.emplace_back(ind);
                    ++iter;
                    --entries;
                }

                clusters.emplace_back(tmpclusters[needMergeInd]);  // 将当前需要合并的簇添加到最终聚类结果中
                // 注意这里先放入 clusters 中，之后合并的时候，不会修改之前聚类的结果
                size_t numCluster = clusters.size();

                for(size_t j = 0; j < vInd.size(); j++)
                {
                    for(size_t ptNum = 0; ptNum < tmpclusters[vInd[j]].size(); ptNum++)
                    {
                        clusters[numCluster - 1].emplace_back(tmpclusters[vInd[j]][ptNum]);  // 将被合并的簇的边缘点添加到最终聚类结果中
                    }
                }
            }
        }       
    }

    // 计算从一个点 ptFrom 到另一个点 ptTo 的方向向量，并将结果存储在 Eigen::Vector2f 类型的引用 direction 中
    // Eigen::Vector2f 是一个二维向量（列向量，维度为 2 行 1 列），包含两个浮点数（单精度浮点数）元素，分别代表向量在 x 和 y 方向上的分量
    void LinK3D_Extractor::computeDirection(pcl::PointXYZI ptFrom, pcl::PointXYZI ptTo, Eigen::Vector2f &direction)
    {
        // 计算终点 ptTo 在 x 方向上的坐标减去起点 ptFrom 在 x 方向上的坐标，得到 x 方向上的分量。
        direction(0, 0) = ptTo.x - ptFrom.x;  // 0行0列存x
        // 计算终点 ptTo 在 y 方向上的坐标减去起点 ptFrom 在 y 方向上的坐标，得到 y 方向上的分量。
        direction(1, 0) = ptTo.y - ptFrom.y;  // 1行0列存y
    }

    // 提取有效的边缘聚集关键点，即聚合关键点
    // 从输入的聚类 clusters 中提取出关键点，并将有效的聚类存储到 validCluster 中，然后返回提取的关键点（有效簇的质心） keyPoints
    vector<pcl::PointXYZI> LinK3D_Extractor::getMeanKeyPoint(const ScanEdgePoints &clusters, ScanEdgePoints &validCluster)
    {        
        int count = 0;  // 初始化计数器，表示有效关键点的数量
        int numCluster = clusters.size();  // 获取输入聚类的总数
        // 初始化用于存储关键点的容器 keyPoints 和 tmpKeyPoints，以及存储有效聚类的容器 tmpEdgePoints。
        vector<pcl::PointXYZI> keyPoints;
        vector<pcl::PointXYZI> tmpKeyPoints;
        ScanEdgePoints tmpEdgePoints;
        map<float, int> distanceOrder;  // 用于去重关键点。
        // 键为关键点到原点的距离，值为关键点在 tmpKeyPoints 中的索引，这样可以按照距离从小到大对关键点进行排序。

        for(int i = 0; i < numCluster; i++)  // 遍历每一个聚类
        {
            int ptCnt = clusters[i].size();  // 获取当前聚类中点的数量     
            if(ptCnt < ptNumTh)  // 簇中点的数量太少，丢弃该簇
            {
                continue;
            }

            //  初始化一些变量
            vector<PointXYZSCA> tmpCluster;  // tmpCluster 用于存储当前聚类中的点
            set<int> scans;  // scans 用于记录该聚类中点所属的不同扫描线位置
            float x = 0, y = 0, z = 0, intensity = 0;  // x、y、z 和 intensity 用于计算当前聚类的质心
            for(int ptNum = 0; ptNum < ptCnt; ptNum++)
            {
                PointXYZSCA pt = clusters[i][ptNum];
                int scan = int(pt.scan_position);
                scans.insert(scan);

                x += pt.x;
                y += pt.y;
                z += pt.z;
                intensity += pt.scan_position;  // 整数部分是scan线束的索引(id)，小数部分是相对起始时刻的时间
            }

            // 有效的边缘关键点通常垂直分布在簇中，如果不同扫描线位置的个数小于 scanNumTh ，说明该聚类太扁平，故过滤掉扁平的簇
            if(scans.size() < (size_t)scanNumTh)
            {
                continue;
            }

            // 计算当前聚类的质心
            pcl::PointXYZI pt;  // 簇的质心
            pt.x = x/ptCnt;
            pt.y = y/ptCnt;
            pt.z = z/ptCnt;
            pt.intensity = intensity/ptCnt;

            // 簇中质心到原点的距离
            float distance = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

            // 使用 distanceOrder 进行去重：检查 distance 是否已经出现过，如果出现过，说明该质心已经是一个重复的关键点，跳过该聚类，继续下一个。
            // 该方法万一方向不同，距离凑巧相等？？？
            auto iter = distanceOrder.find(distance);
            if(iter != distanceOrder.end())
            {
                continue;
            }
            // 将 distance 加入 distanceOrder 中，并更新计数器 count
            distanceOrder[distance] = count;
            count++;
            
            tmpKeyPoints.emplace_back(pt);
            tmpEdgePoints.emplace_back(clusters[i]);
        }

        // 遍历 distanceOrder 中的关键点，并根据距离从小到大的顺序，将关键点和对应的有效聚类添加到 keyPoints 和 validCluster 中。
        for(auto iter = distanceOrder.begin(); iter != distanceOrder.end(); iter++)
        {
            int index = (*iter).second;
            pcl::PointXYZI tmpPt = tmpKeyPoints[index];
            
            keyPoints.emplace_back(tmpPt);
            validCluster.emplace_back(tmpEdgePoints[index]);
        }
        // 返回提取到的关键点
        return keyPoints;
    }

    // 将输入的浮点数 in 四舍五入到小数点后一位
    float LinK3D_Extractor::fRound(float in)
    {
        float f;
        // std::round()可以四舍五入到最近的整数，下面的操作可以四舍五入到最近的1位小数
        int temp = std::round(in * 10);
        f = temp/10.0;
        
        return f;
    }

    // 计算关键点的描述符
    void LinK3D_Extractor::getDescriptors(const vector<pcl::PointXYZI> &keyPoints, 
                                          cv::Mat &descriptors)
    {
        // 首先，检查传入的关键点向量 keyPoints 是否为空，如果为空则直接返回
        if(keyPoints.empty())
        {
            return;
        }

        int ptSize = keyPoints.size();  // 获取关键点的数量

        // 初始化一个大小为 ptSize*180 的单通道浮点型 cv::Mat 对象 descriptors，用于存储描述符
        descriptors = cv::Mat::zeros(ptSize, 180, CV_32FC1);

        // 创建两个二维向量表 distanceTab 和 directionTab，分别用于存储两点之间的距离和方向向量
        vector<vector<float>> distanceTab;  // 距离表
        vector<float> oneRowDis(ptSize, 0);
        distanceTab.resize(ptSize, oneRowDis);

        vector<vector<Eigen::Vector2f>> directionTab;  // 方向表
        Eigen::Vector2f direct(0, 0);
        vector<Eigen::Vector2f> oneRowDirect(ptSize, direct);
        directionTab.resize(ptSize, oneRowDirect);

        //Build distance and direction tables for fast descriptor generation.
        // 预处理：构建距离表和方向表，以便快速生成描述符。
        // 对于每一个关键点 keyPoints[i]，遍历其与其他关键点 keyPoints[j] 的距离，并计算两点之间的方向向量，将结果存储在 distanceTab 和 directionTab 中
        for(size_t i = 0; i < keyPoints.size(); i++)
        {
            for(size_t j = i + 1; j < keyPoints.size(); j++)
            {
                float dist = distPt2Pt(keyPoints[i], keyPoints[j]);  // 求出两关键点的距离
                distanceTab[i][j] = fRound(dist);  // 四舍五入到一位小数
                distanceTab[j][i] = distanceTab[i][j];

                Eigen::Vector2f tmpDirection;
                                
                tmpDirection(0, 0) = keyPoints[j].x - keyPoints[i].x;
                tmpDirection(1, 0) = keyPoints[j].y - keyPoints[i].y;

                directionTab[i][j] = tmpDirection;
                directionTab[j][i] = -tmpDirection;
            }
        }

        for(size_t i = 0; i < keyPoints.size(); i++)  // 遍历每一个关键点
        {
            vector<float> tempRow(distanceTab[i]);
            std::sort(tempRow.begin(), tempRow.end());  // 将 distanceTab[i] 中的距离进行排序
            int Index[3];  // 三个最近关键点的索引
           
            // 获取与当前关键点 keyPoints[i] 距离最近的三个关键点。
            //Get the closest three keypoints of current keypoint.
            for(int k = 0; k < 3; k++)
            {
                vector<float>::iterator it1 = find(distanceTab[i].begin(), distanceTab[i].end(), tempRow[k + 1]);  // ? 距离相同怎么办？ 有点问题
                if(it1 == distanceTab[i].end()) {
                    continue;  // 怎么可能搜不到？
                } else {
                    // 并根据排序结果确定这三个关键点的索引 Index
                    Index[k] = std::distance(distanceTab[i].begin(), it1);  // 返回两个迭代器之间的距离，也可以理解为计算两个元素 first 和 last 之间的元素数。
                }
            }

            //Generate the descriptor for each closest keypoint. 
            //The final descriptor is based on the priority of the three closest keypoint.
            // 为每个最近的关键点生成描述符。最后的描述符是基于三个最接近的关键点的优先级。
            for(int indNum = 0; indNum < 3; indNum++)
            {
                int index = Index[indNum];
                Eigen::Vector2f mainDirection;
                mainDirection = directionTab[i][index];  // 主方向
                
                // 将描述符分为180个区域
                vector<vector<float>> areaDis(180);  // areaDis 存储 180 个区域中最近关键点的距离
                areaDis[0].emplace_back(distanceTab[i][index]);
                
                // 遍历其他关键点
                for(size_t j = 0; j < keyPoints.size(); j++)
                {
                    if(j == i || (int)j == index)  // 跟中心点和最近关键点重复，直接跳过
                    {
                        continue;
                    }
                    
                    // 计算其与主方向向量的夹角
                    Eigen::Vector2f otherDirection = directionTab[i][j];
                
                    Eigen::Matrix2f matrixDirect;  // 定义了一个名为 matrixDirect 的 2x2 浮点型矩阵，类型为 Eigen::Matrix2f
                    matrixDirect << mainDirection(0, 0), mainDirection(1, 0), otherDirection(0, 0), otherDirection(1, 0);  // 用 << 运算符来初始化这个矩阵的值
                    float deter = matrixDirect.determinant();  // 行列式计算
                    // 行列式的计算结果是一个浮点数，代表了矩阵 matrixDirect 的线性变换的缩放因子。
                    // 行列式的值对于线性变换的特征和性质有很重要的意义，这里用来判断两个方向向量的旋转方向。

                    int areaNum = 0;  // 用于记录当前计算得到的角度所在的区域编号
                    // 计算两个向量 mainDirection 和 otherDirection 之间的夹角的余弦值：
                    // mainDirection.dot(otherDirection)：表示两个向量的点积（内积），即对应元素相乘再相加。
                    // mainDirection.norm() 和 otherDirection.norm()：表示向量的2-范数，即向量的模（magnitude）或长度。
                    // 余弦值的计算公式为：cosAng = dotProduct / (lengthMainDirection * lengthOtherDirection)。
                    // 通过这个计算，我们可以得到两个向量之间的夹角的余弦值。
                    // 通过余弦值，我们可以进一步得到角度的信息，例如用 acos() 函数来计算实际的夹角。
                    // 这个夹角可以用来判断两个向量的方向关系，以及它们之间的旋转角度。
                    double cosAng = (double)mainDirection.dot(otherDirection) / (double)(mainDirection.norm() * otherDirection.norm());
                    
                    // 如果余弦值的绝对值减 1 的结果大于 0，说明夹角为 0 度或 180 度，这种情况忽略不计，跳过当前迭代，继续下一个迭代
                    if(abs(cosAng) - 1 > 0)
                    {
                        continue;
                    }

                    // 使用余弦值计算实际的夹角角度 angle，并将其转换为角度制，acos() 函数返回值为弧度制的角度         
                    float angle = acos(cosAng) * 180 / M_PI;

                    // 接着，根据夹角的大小和两个向量之间的方向关系，来确定 areaNum 所在的区域编号。
                    // 这里使用了 deter 变量来判断两个向量的方向关系。
                    // 如果夹角为正向，则 deter > 0，计算夹角所在的区域编号；
                    // 如果夹角为负向，则 deter < 0，需要对角度进行一些修正后计算夹角所在的区域编号。
                    // 最终，得到的 areaNum 表示 angle 所在的区域索引。

                    // 如果角度不在 [0, 180] 范围内，忽略不计，跳过当前迭代，继续下一个迭代
                    if(angle < 0 || angle > 180)
                    {
                        continue;
                    }

                    // 如果向量 mainDirection 和 otherDirection 之间的夹角为正向，计算所在的区域编号，其中 areaNum 表示区域的索引
                    if(deter > 0)
                    {
                        // 将 (angle - 1) / 2 的结果向上取整，得到一个整数值 areaNum，用来表示 angle 所在的区域编号
                        areaNum = ceil((angle - 1) / 2);
                    }
                    else  // 如果向量 mainDirection 和 otherDirection 之间的夹角为负向，计算所在的区域编号，其中 areaNum 表示区域的索引
                    {
                        if(angle - 2 < 0)
                        { 
                            areaNum = 0;
                        }
                        else
                        {
                            angle = 360 - angle;  // 对角度进行修正
                            areaNum = ceil((angle - 1) / 2); 
                        }
                    }

                    if(areaNum != 0)
                    {
                        areaDis[areaNum].emplace_back(distanceTab[i][j]);  // 将距离存储在相应的区域中
                    }
                }
                
                // descriptors 是一个 cv::Mat 类型的矩阵，其数据类型为 float，并且其每一行存储一个关键点的特征描述符。
                // descriptors.ptr<float>(i) 是一个用于访问 cv::Mat 类型数据中特定行的指针。
                // ptr<float>(i) 方法返回了矩阵 descriptors 的第 i 行的指针，这样就可以通过该指针访问第 i 行的数据。
                // 注意：由于 descriptor 是指向 cv::Mat 数据的指针，所以对 descriptor 进行修改会影响到原始的 descriptors 矩阵中的数据。
                float *descriptor = descriptors.ptr<float>(i);                           
                // 对于每个区域，将存储的距离排序后，取第一个非零距离作为该区域的描述符值
                for(int areaNum = 0; areaNum < 180; areaNum++) 
                {
                    if(areaDis[areaNum].size() == 0)
                    {
                        continue;
                    }
                    else
                    {
                        std::sort(areaDis[areaNum].begin(), areaDis[areaNum].end());

                        if(descriptor[areaNum] == 0)
                        {
                            descriptor[areaNum] = areaDis[areaNum][0]; 
                        }
                    }
                }
            }
        }
    }
    // 两个关键点描述符的匹配
    // 其中 curAggregationKeyPt 为当前关键点，toBeMatchedKeyPt 为待匹配关键点，
    // curDescriptors 为当前描述符，toBeMatchedDescriptors 为待匹配的描述符，vMatchedIndex为匹配的结果
    void LinK3D_Extractor::match(
            vector<pcl::PointXYZI> &curAggregationKeyPt, 
            vector<pcl::PointXYZI> &toBeMatchedKeyPt,
            cv::Mat &curDescriptors,
            cv::Mat &toBeMatchedDescriptors,
            vector<pair<int, int>> &vMatchedIndex)
    {        
        int curKeypointNum = curAggregationKeyPt.size();  // 当前关键点的数量
        int toBeMatchedKeyPtNum = toBeMatchedKeyPt.size();  // 待匹配关键点的数量
        
        multimap<int, int> matchedIndexScore;  // 存储匹配索引和匹配得分的多重映射
        multimap<int, int> mMatchedIndex;  // 存储待匹配索引和匹配索引的多重映射
        set<int> sIndex;  // 存储待匹配索引的集合
       
        for(int i = 0; i < curKeypointNum; i++)  // 遍历每一个关键点
        {
            std::pair<int, int> highestIndexScore(0, 0);  // 存储当前关键点与最佳匹配关键点的索引和匹配得分
            float* pDes1 = curDescriptors.ptr<float>(i);  // 获取指向第 i 行开头的浮点指针（即当前关键点的描述符指针）
            
            for(int j = 0; j < toBeMatchedKeyPtNum; j++)
            {
                int sameDimScore = 0;  // 记录维度相同的得分
                float* pDes2 = toBeMatchedDescriptors.ptr<float>(j);  // 获取指向第 j 行开头的浮点指针（即待匹配关键点的描述符指针）
                
                for(int bitNum = 0; bitNum < 180; bitNum++)  // 遍历描述符维度
                {
                    // 在两个描述符中计算相应非零维度之间的差值的绝对值。如果该值低于 0.2 ，则相似性分数增加 1 。
                    if(pDes1[bitNum] != 0 && pDes2[bitNum] != 0 && abs(pDes1[bitNum] - pDes2[bitNum]) <= 0.2) {
                        sameDimScore += 1;
                    }
                    
                    if(bitNum > 90 && sameDimScore < 3) {  // 小trick， 剪枝操作（若匹配得分不足3，提前终止）
                        break;                        
                    }                    
                }
                // 如果待匹配得分高于当前最佳匹配得分，则更新最佳匹配索引和得分
                if(sameDimScore > highestIndexScore.second)
                {
                    highestIndexScore.first = j;
                    highestIndexScore.second = sameDimScore;
                }
            }
            
            // 为之后删除重复的匹配项做准备
            //Used for removing the repeated matches.
            matchedIndexScore.insert(std::make_pair(i, highestIndexScore.second)); //Record i and its corresponding score.  // 记录i及其相应的分数。
            mMatchedIndex.insert(std::make_pair(highestIndexScore.first, i)); //Record the corresponding match between j and i.  // 记录j和i之间的对应匹配。
            sIndex.insert(highestIndexScore.first); //Record the index that may be repeated matches.  // 记录可能重复匹配的索引。
        }

        // 删除重复的匹配项。
        //Remove the repeated matches.
        for(set<int>::iterator setIt = sIndex.begin(); setIt != sIndex.end(); ++setIt)
        {
            int indexJ = *setIt;
            auto entries = mMatchedIndex.count(indexJ);
            if(entries == 1)  // 只找到了1个对应的匹配项（最佳情况）
            {
                auto iterI = mMatchedIndex.find(indexJ);
                auto iterScore = matchedIndexScore.find(iterI->second);
                if(iterScore->second >= matchTh)  // 相似性分数大于匹配阈值，认为匹配有效
                {                    
                    vMatchedIndex.emplace_back(std::make_pair(iterI->second, indexJ));
                }           
            }
            else  // 有重复的匹配项（一个j对多个i）
            { 
                auto iter1 = mMatchedIndex.find(indexJ);
                int highestScore = 0;
                int highestScoreIndex = -1;

                while(entries)  // 在多重映射中寻找匹配得分最高的索引
                {
                    int indexI = iter1->second;
                    auto iterScore = matchedIndexScore.find(indexI);
                    if(iterScore->second > highestScore) {  // 取相似性分数最大的一个作为匹配项
                        highestScore = iterScore->second;
                        highestScoreIndex = indexI;
                    }                
                    ++iter1;
                    --entries;
                }

                if(highestScore >= matchTh)  // 相似性分数大于匹配阈值，认为匹配有效
                {
                    vMatchedIndex.emplace_back(std::make_pair(highestScoreIndex, indexJ));
                }
            }
        }
    }

    //Remove the edge keypoints with low curvature for further edge keypoints matching.
    // 移除曲率较低的边缘关键点，以便进一步匹配边缘关键点。
    // 该函数将 cluters 中每个簇中的点按照线束分开，每个簇的每条线束中只取曲率最大的点，并将筛选后的点存储在名为 filtered 的向量中。
    void LinK3D_Extractor::filterLowCurv(ScanEdgePoints &clusters, ScanEdgePoints &filtered)
    {
        int numCluster = clusters.size();  // 获取簇的数量
        filtered.resize(numCluster);  // 将 filtered 向量的大小调整为与聚类数量相匹配
        for(int i = 0; i < numCluster; i++)  // 遍历每一个聚类
        {
            int numPt = clusters[i].size();  // 获取当前聚类中点的数量
            ScanEdgePoints tmpCluster;  // 创建一个临时聚类集合 tmpCluster 来存储筛选后的点
            vector<int> vScanID;  // 创建一个向量 vScanID 来记录扫描线ID

            for(int j = 0; j < numPt; j++)  // 把每个簇中的点按照线束分开，每个线束单独成簇
            {
                PointXYZSCA pt = clusters[i][j];
                int scan = int(pt.scan_position);  // 获取该点的扫描线ID
                auto it = std::find(vScanID.begin(), vScanID.end(), scan);  // 检查 vScanID 中是否已存在该扫描线ID

                if(it == vScanID.end())
                {  // 如果 vScanID 中不存在该扫描线ID，则将其加入，并在 tmpCluster 中创建一个新的子聚类
                    vScanID.emplace_back(scan);
                    vector<PointXYZSCA> vPt(1, pt);
                    tmpCluster.emplace_back(vPt);
                }
                else  // 如果 vScanID 中已存在该扫描线ID，则将该点加入到 tmpCluster 中对应的子聚类。
                {
                    int filteredInd = std::distance(vScanID.begin(), it);
                    tmpCluster[filteredInd].emplace_back(pt);
                }
            }

            // 对 tmpCluster 中的每个子聚类进行处理
            for(size_t scanID = 0; scanID < tmpCluster.size(); scanID++)
            {
                if(tmpCluster[scanID].size() == 1)
                {  // 如果子聚类仅包含一个点，直接将该点添加到 filtered 聚类中
                    filtered[i].emplace_back(tmpCluster[scanID][0]);
                }
                else  // 否则，在子聚类中找到曲率最高的点，并将该点添加到 filtered 聚类中
                {
                    float maxCurv = 0;
                    PointXYZSCA maxCurvPt;
                    for(size_t j = 0; j < tmpCluster[scanID].size(); j++)
                    {
                        if(tmpCluster[scanID][j].curvature > maxCurv)
                        {
                            maxCurv = tmpCluster[scanID][j].curvature;
                            maxCurvPt = tmpCluster[scanID][j];
                        }
                    }
                    filtered[i].emplace_back(maxCurvPt);
                }
            }  
        }
    }

    //Get the edge keypoint matches based on the matching results of aggregation keypoints.
    // 根据聚合关键点的匹配结果，得到边缘关键点匹配。
    // 匹配两个描述符后，将在簇中搜索边缘关键点的匹配项。对于匹配到的聚集关键点，选取每条扫描线中具有最高平滑度的对应边缘关键点。对位于同一扫描线上的边缘关键点进行匹配。
    void LinK3D_Extractor::findEdgeKeypointMatch(
            ScanEdgePoints &filtered1, 
            ScanEdgePoints &filtered2, 
            vector<std::pair<int, int>> &vMatched, 
            vector<std::pair<PointXYZSCA, PointXYZSCA>> &matchPoints)
    {
        int numMatched = vMatched.size();  // 匹配到的聚合关键点数量
        for(int i = 0; i < numMatched; i++)  // 遍历每对匹配的关键点对
        {
            pair<int, int> matchedInd = vMatched[i];  // 获取当前匹配的关键点对的索引
            
            int numPt1 = filtered1[matchedInd.first].size();  // 第一个过滤后的边缘点云簇的点数量
            int numPt2 = filtered2[matchedInd.second].size();  // 第二个过滤后的边缘点云簇的点数量

            map<int, int> mScanID_Index1;  // 存储第一个边缘点云簇中每个扫描线ID对应的点的索引
            map<int, int> mScanID_Index2;  // 存储第二个边缘点云簇中每个扫描线ID对应的点的索引

            // 将第一个边缘点云簇中的点按扫描线ID进行分组，并存储扫描线ID与点的索引的映射
            for(int i = 0; i < numPt1; i++)
            {
                int scanID1 = int(filtered1[matchedInd.first][i].scan_position);
                pair<int, int> scanID_Ind(scanID1, i);
                mScanID_Index1.insert(scanID_Ind);
            }
            // 将第二个边缘点云簇中的点按扫描线ID进行分组，并存储扫描线ID与点的索引的映射
            for(int i = 0; i < numPt2; i++)
            {
                int scanID2 = int(filtered2[matchedInd.second][i].scan_position);
                pair<int, int> scanID_Ind(scanID2, i);
                mScanID_Index2.insert(scanID_Ind);
            }

            // 在两个边缘点云簇的扫描线ID映射中找到匹配的扫描线ID，并将匹配的点对加入matchPoints向量中
            for(auto it1 = mScanID_Index1.begin(); it1 != mScanID_Index1.end(); it1++)
            {
                int scanID1 = (*it1).first;
                auto it2 = mScanID_Index2.find(scanID1);
                if(it2 == mScanID_Index2.end()) {  // 如果找不到匹配的扫描线ID，则跳过当前点
                    continue;
                }
                else
                {
                    vector<PointXYZSCA> tmpMatchPt;  // 无用变量
                    PointXYZSCA pt1 = filtered1[matchedInd.first][(*it1).second];  // 第一个边缘点
                    PointXYZSCA pt2 = filtered2[matchedInd.second][(*it2).second];  // 第二个边缘点
                    
                    pair<PointXYZSCA, PointXYZSCA> matchPt(pt1, pt2);  // 将匹配的点对组成一个pair
                    matchPoints.emplace_back(matchPt);  // 将匹配的点对加入matchPoints向量
                }
            }
        }
    }

    // 重载函数运算符 operator()。将传入的点云 pLaserCloudIn 通过 LinK3D_Extractor 进行一系列的处理，
    // 得到一些聚类的关键点 keyPoints、点云的描述符 descriptors 和边缘关键点 validCluster。
    void LinK3D_Extractor::operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, vector<pcl::PointXYZI> &keyPoints, cv::Mat &descriptors, ScanEdgePoints &validCluster)
    {
        // typedef vector<vector<PointXYZSCA>> ScanEdgePoints;
        ScanEdgePoints edgePoints;
        // 从输入的点云数据中提取边缘点，并将边缘点保存在 ScanEdgePoints 类型的容器 edgePoints 中
        extractEdgePoint(pLaserCloudIn, edgePoints);

        // 将提取的边缘点根据其水平角度划分到不同的扇区中，然后将每个扇区中的边缘点存储在 ScanEdgePoints 类型的容器 sectorAreaCloud 中
        ScanEdgePoints sectorAreaCloud;
        divideArea(edgePoints, sectorAreaCloud); 

        // 将扇区中的边缘点进行聚类操作，将满足一定条件的边缘点聚合成一个簇，并将这些簇存储在 ScanEdgePoints 类型的容器 clusters 中
        ScanEdgePoints clusters;
        getCluster(sectorAreaCloud, clusters); 

        // 从输入的聚类 clusters 中提取出关键点，并将有效的聚类存储到 validCluster 中，然后返回提取的关键点（有效簇的质心） keyPoints
        vector<int> index;
        keyPoints = getMeanKeyPoint(clusters, validCluster);

        // 计算关键点 keyPoints 的描述符，并将生成的描述符存储到 descriptors 中
        getDescriptors(keyPoints, descriptors); 
    }

}
