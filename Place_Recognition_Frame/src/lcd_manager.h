#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <filesystem>

#include "Iris/LidarIris.h"
#include "M2DP/m2dp.h"
#include "SC/Scancontext.h"
#include "SC_I/IScancontext.h"
#include "ISC/iscGenerationClass.h"
#include "CSSC/csscM.h"
#include "BoW3D/Frame.h"
#include "BoW3D/LinK3D_Extractor.h"
#include "BoW3D/BoW3D.h"
#include "NDD/NDD.h"
#include "NDTMC/NDTMC.h"

#include "data_loader.h"

#include "tools/pointcloud_util.h"
#include "tools/tictoc.h"
#include "tools/nanoflann.hpp"
#include "tools/KDTreeVectorOfVectorsAdaptor.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/esf.h>

class LCDManager
{
public:

    void KITTI_test(std::string dataset_sequence);
    
    // KITTI
    void KITTI_ESF(std::string dataset_sequence);

    void KITTI_M2DP(std::string dataset_sequence);

    void KITTI_Scan_Context(std::string dataset_sequence);

    void KITTI_Iris_KNN(std::string dataset_sequence);
    void KITTI_Iris(std::string dataset_sequence);

    void KITTI_SC_Intensity(std::string dataset_sequence);
    void KITTI_ISC(std::string dataset_sequence);

    void KITTI_CSSC_Force(std::string dataset_sequence);
    void KITTI_CSSC(std::string dataset_sequence);

    void KITTI_BoW3D(std::string dataset_sequence);

    void KITTI_NDD(std::string dataset_sequence);

    void KITTI_NDTMC(std::string dataset_sequence);


    // JORD
    void JORD_ESF(std::string dataset_sequence);

    void JORD_M2DP(std::string dataset_sequence);

    void JORD_Scan_Context(std::string dataset_sequence);

    void JORD_Iris_KNN(std::string dataset_sequence);
    void JORD_Iris(std::string dataset_sequence);

    void JORD_SC_Intensity(std::string dataset_sequence);
    void JORD_ISC(std::string dataset_sequence);

    void JORD_Seed(std::string dataset_sequence);

    void JORD_CSSC_Force(std::string dataset_sequence);
    void JORD_CSSC(std::string dataset_sequence);

    void JORD_BoW3D(std::string dataset_sequence);

    void JORD_NDD(std::string dataset_sequence);

    void JORD_STD(std::string dataset_sequence);

    void JORD_Contour_Context(std::string dataset_sequence);

    void JORD_NDTMC(std::string dataset_sequence);

};