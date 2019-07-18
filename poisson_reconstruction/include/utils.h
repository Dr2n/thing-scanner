#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB rgbPoint  ;
typedef pcl::PointXYZ    xyzPoint  ;
typedef pcl::PointCloud<rgbPoint> rgbPointCloud ;
typedef pcl::PointCloud<xyzPoint> xyzPointCloud ;


//function for poisson reconstruction
pcl::PolygonMesh poissonReconstuction(xyzPointCloud::Ptr cloud);

//texturing function
pcl::PolygonMesh meshTexturing(pcl::PolygonMesh& mesh, rgbPointCloud::Ptr sourceRGBPointCloud);

//load ply file
rgbPointCloud::Ptr loadPlyFile_RGBPoint(std::string& file_path);

//load pcd file
rgbPointCloud::Ptr loadPcdFile_RGBPoint(std::string& file_path);

//
Eigen::Matrix4f loadPoseInfo(std::string& file_path);



