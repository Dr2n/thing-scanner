#include "load_data.h"

#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

//given a point cloud, do poisson reconstruction

int main(){

    std::cout<<" this is a file for poisson reconstruction! "<<std::endl;

    std::string data_path = "../data/test_mesh_1.ply";

    //new point cloud
    PointCloud::Ptr cloud(new PointCloud);

    //load ply file
    cloud = load_ply(data_path);

    //load file
    //pcl::PCDReader reader;
    //reader.read(file_path,*cloud);

    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);

    //estimate local surface properties
    pcl::NormalEstimationOMP<POINT, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);

    normal_estimator.setInputCloud(cloud);

    pcl::search::KdTree<POINT>::Ptr tree(new pcl::search::KdTree<POINT>);
    tree->setInputCloud(cloud);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(20);

    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cloud, center);
    normal_estimator.setViewPoint(center[0],center[1],center[2]);

    normal_estimator.compute(*normal_cloud);

    for(size_t i=0;i<normal_cloud->size();i++)
    {
        normal_cloud->points[i].normal_x *= -1;
        normal_cloud->points[i].normal_y *= -1;
        normal_cloud->points[i].normal_z *= -1;

    }
    std::cout<<"normal computed!"<<std::endl;

    //concatenate normal and points
    pcl::PointCloud<pcl::PointNormal>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normal_cloud, *combined_cloud);

    //poisson reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(combined_cloud);

    //output polynom
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    std::cout<<"reconstructed!"<<std::endl;

    pcl::io::savePLYFile("reconstructed_mesh.ply", mesh);
    std::cout<<"file saved!"<<std::endl;

    return 0;
}
