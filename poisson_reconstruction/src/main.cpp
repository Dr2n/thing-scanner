#include "utils.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

//given a point cloud, do poisson reconstruction

int main(int argc, char **argv){

    //***************************************   arguments   ********************************************//

    std::string data_path, outfile_path;

    if(argc < 2)
    {
        std::cout<<"too few arguments!"<<std::endl;
        std::cout<<"first param: data path"<<std::endl;
        std::cout<<"second param: output file path"<<std::endl;
    }

    data_path = argv[1];
    outfile_path = argv[2];


    //load optimized rgb textured point cloud
    rgbPointCloud::Ptr rgb_joint_cloud(new rgbPointCloud);
    rgb_joint_cloud = loadPlyFile_RGBPoint(data_path);

    //*************************************   pcl cloud viewer   *******************************************//
    //point cloud viewer
    pcl::visualization::CloudViewer viewer("cloud viewer");

   //*****************************   poisson reconstruction   ************************************//

   //convert rgbPointCloud to xyzPointCloud
   //poisson reconstruction only works with xyzPointCloud
   xyzPointCloud::Ptr joint_cloud(new xyzPointCloud);
   pcl::copyPointCloud(*rgb_joint_cloud,*joint_cloud);

   //call poisson reconstruction function
   pcl::PolygonMesh mesh;
   mesh = poissonReconstuction(joint_cloud);
   std::cout<<"reconstructed!"<<std::endl;

   //**********************************  texturing  **********************************************//

   //call texturing function
   //texture mesh with rgb color
   mesh = meshTexturing(mesh,rgb_joint_cloud);


   //*********************************   save file   *********************************************//
   pcl::io::savePLYFile(outfile_path, mesh);
   std::cout<<"file saved!"<<std::endl;

    return 0;
}
