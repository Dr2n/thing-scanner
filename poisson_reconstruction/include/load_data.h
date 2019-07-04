#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ POINT  ;
typedef pcl::PointCloud<POINT> PointCloud ;

// load ply file
// param: - file_path: data path
// return: - pointer to cloud
PointCloud::Ptr load_ply(std::string& file_path);


// load pcd file
// param: - file_path: data path
// return: - pointer to cloud
PointCloud::Ptr load_ply(std::string& file_path);
