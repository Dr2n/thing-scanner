#include "utils.h"


//load single ply file
rgbPointCloud::Ptr loadPlyFile_RGBPoint(std::string& file_path)
{

    rgbPointCloud::Ptr cloud(new rgbPointCloud);

    //load ply file
    pcl::PLYReader reader;
    reader.read(file_path,*cloud);

    return cloud;

}

//load single pcd file
rgbPointCloud::Ptr loadPcdFile_RGBPoint(std::string& file_path)
{

    rgbPointCloud::Ptr cloud(new rgbPointCloud);

    //load file
    pcl::PCDReader reader;
    reader.read(file_path,*cloud);

    return cloud;

}







