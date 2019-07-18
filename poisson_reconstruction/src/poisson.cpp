#pragma once
#include "utils.h"

pcl::PolygonMesh poissonReconstuction(xyzPointCloud::Ptr cloud)
{
    //normal point cloud
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);

    //estimator for surface normal
    pcl::NormalEstimationOMP<xyzPoint, pcl::Normal> normal_estimator;

    //kd tree used to find nearest neighbors for normal computation
    pcl::search::KdTree<xyzPoint>::Ptr tree(new pcl::search::KdTree<xyzPoint>);
    tree->setInputCloud(cloud);

    normal_estimator.setInputCloud(cloud);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(20);

    //calculate point cloud center
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cloud, center);
    normal_estimator.setViewPoint(center[0],center[1],center[2]);

    //compute normal of point cloud
    normal_estimator.compute(*normal_cloud);

    //inverse the normal
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
    poisson.setConfidence(false); //default value
    poisson.setDegree(2); //default value
    poisson.setDepth(8); //octree resolution, default value
    poisson.setInputCloud(combined_cloud);

    //kd tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr pn_tree(new pcl::search::KdTree<pcl::PointNormal>);
    pn_tree->setInputCloud(combined_cloud);

    poisson.setSearchMethod(pn_tree);

    //output polynom
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    return mesh;

}

pcl::PolygonMesh meshTexturing(pcl::PolygonMesh& mesh, rgbPointCloud::Ptr sourceRGBPointCloud)
{
    rgbPointCloud mesh_cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);


    pcl::KdTreeFLANN<rgbPoint> kdTreeForTexturing;
    kdTreeForTexturing.setInputCloud(sourceRGBPointCloud);

    int K = 5;
    std::vector<int> k_indices(5);
    std::vector<float> k_sqr_dists(K);

    for(int i=0;i<mesh_cloud.size();i++)
    {
        int curr_r(0), curr_b(0),curr_g(0);

        int num_neighbors = kdTreeForTexturing.nearestKSearch(mesh_cloud[i],K,k_indices,k_sqr_dists);
        if(!num_neighbors) continue;

        for(int j=0;j<num_neighbors;j++)
        {
            curr_r += sourceRGBPointCloud->points[k_indices[j]].r;
            curr_g += sourceRGBPointCloud->points[k_indices[j]].g;
            curr_b += sourceRGBPointCloud->points[k_indices[j]].b;

        }

        mesh_cloud.points[i].r = int(curr_r/k_sqr_dists.size());
        mesh_cloud.points[i].g = int(curr_g/k_sqr_dists.size());
        mesh_cloud.points[i].b = int(curr_b/k_sqr_dists.size());
    }

    pcl::toPCLPointCloud2(mesh_cloud,mesh.cloud);

    return mesh;
}
