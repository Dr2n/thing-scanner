#include "load_data.h"


PointCloud::Ptr load_ply(std::string& file_path)
{
    //new point cloud
    PointCloud::Ptr cloud(new PointCloud);

    //load file
    pcl::PLYReader reader;
    reader.read(file_path,*cloud);

    return cloud;
}

PointCloud::Ptr load_pcd(std::string& file_path)
{
    //new point cloud
    PointCloud::Ptr cloud(new PointCloud);

    //load file
    pcl::PCDReader reader;
    reader.read(file_path,*cloud);

    return cloud;
}

