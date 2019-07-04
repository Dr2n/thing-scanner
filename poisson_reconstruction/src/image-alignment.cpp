#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "load_data.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_reconstructed(){

    //virtual sensor: read rgb.png and depth.png from dataset

    //for every incoming image
    //1. pose estimation via direct image alignment/feature matching
    //2. object segmentation(optional)
    //3. generate new point cloud
    //4. combine point cloud

    //no new frame: return combined point cloud
}

