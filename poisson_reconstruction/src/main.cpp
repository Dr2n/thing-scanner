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


    //************************************** work with data set   *******************************************//

//    std::string cloud_path = "../data/detergent/clouds/NP1_" ;
//    std::string pose_path = "../data/detergent/poses/NP1_";

//    //new point cloud
//    rgbPointCloud::Ptr rgb_cloud1(new rgbPointCloud);
//    rgbPointCloud::Ptr rgb_cloud2(new rgbPointCloud);

//    for(int i=0;i<=119;i++)
//    {
//        std::string obj1,obj2;
//        std::string pose1_path, pose2_path;

//        Eigen::Matrix4f p1,p2;
//        if(i==0)
//        {
//            obj1 = cloud_path + std::to_string(0) + ".pcd";
//            rgb_cloud1 = loadPcdFile_RGBPoint(obj1);

//            continue;
//        }

//        //current rgb point cloud
//        obj2 = cloud_path + std::to_string(3*i) + ".pcd";
//        rgb_cloud2 = loadPcdFile_RGBPoint(obj2);

////        pcl::IterativeClosestPoint<rgbPoint,rgbPoint> icp;
////        icp.setInputSource(rgb_cloud2);
////        icp.setInputTarget(rgb_cloud1);
////        icp.setMaximumIterations(50);
////        icp.setMaxCorrespondenceDistance(0.05);

////        //perform alignment
////        icp.align(*rgb_joint_cloud);

////        //obtain transformation between source pd and target pd
////        Eigen::Matrix4f transformation = icp.getFinalTransformation();

////        //combine rgb point cloud
////        rgbPointCloud::Ptr rgb_cloud2_transformed(new rgbPointCloud);
////        pcl::transformPointCloud(*rgb_cloud2, *rgb_cloud2_transformed, transformation);

////        *rgb_cloud1 = (*rgb_cloud1 + *rgb_cloud2_transformed);

//        //point cloud viewer

//        viewer.wasStopped(500);
//        viewer.showCloud(rgb_cloud2);

//        std::cout<<"current frame is: "<<i<<std::endl;

//   }

//    //*****************************   poisson reconstruction   ************************************//

//    //convert rgbPointCloud to xyzPointCloud
//    //poisson reconstruction only works with xyzPointCloud
//    xyzPointCloud::Ptr joint_cloud(new xyzPointCloud);
//    pcl::copyPointCloud(*rgb_joint_cloud,*joint_cloud);

//    //call poisson reconstruction function
//    pcl::PolygonMesh mesh;
//    mesh = poissonReconstuction(joint_cloud);
//    std::cout<<"reconstructed!"<<std::endl;

//    //**********************************  texturing  **********************************************//

//    //call texturing function
//    //texture mesh with rgb color
//    mesh = meshTexturing(mesh,rgb_joint_cloud);


//    //*********************************   save file   *********************************************//
//    pcl::io::savePLYFile(outfile_path, mesh);
//    std::cout<<"file saved!"<<std::endl;

    return 0;
}
