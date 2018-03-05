//
// Created by alberto on 05/03/18.
//

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "utils.h"

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
using Data  = std::vector<PCD<Cloud::PointType>, Eigen::aligned_allocator<PCD<Cloud::PointType>>>;

void correspondenceEstimation(Cloud::ConstPtr source, Cloud::ConstPtr target, Eigen::Matrix4f &transformation);

Cloud::Ptr icpRegistration(Cloud::ConstPtr source, Cloud::ConstPtr target);

Cloud::Ptr method1(const Data &data);

Cloud::Ptr method2(const Data &data);

void visualize(Cloud::ConstPtr cloud1, Cloud::ConstPtr cloud2);


int main(int argc, char** argv) {

    auto data = loadData<Cloud::PointType>(argc, argv, true);

    auto cloud_m1 = method1(data);

    auto cloud_m2 = method2(data);

    visualize(cloud_m1, cloud_m2);

    return 0;
}


Cloud::Ptr method1(const Data &data) {
    Cloud::Ptr result(new Cloud);
    pcl::copyPointCloud(*data[0].cloud, *result);

    pcl::visualization::PCLVisualizer viewer;

    for (size_t i = 1; i < data.size(); ++i) {

        viewer.removeAllPointClouds();

        const auto &cloud = data[i].cloud;

        std::cout << "Unregistered clouds" << std::endl;

        pcl::visualization::PointCloudColorHandlerRGBField<Cloud::PointType> rgb1(result);
        viewer.addPointCloud<Cloud::PointType> (result, rgb1, "cloud1");
        pcl::visualization::PointCloudColorHandlerRGBField<Cloud::PointType> rgb2(cloud);
        viewer.addPointCloud<Cloud::PointType> (cloud, rgb2, "cloud2");

        viewer.spin();

        std::cout << "Correspondence estimation" << std::endl;

        //register cloud with result
        Eigen::Matrix4f transformation;
        correspondenceEstimation(cloud, result, transformation);

        Cloud::Ptr tmp(new Cloud);
        pcl::transformPointCloud(*cloud, *tmp, transformation); //transform current pair into the global transform

        viewer.removePointCloud("cloud2");
        pcl::visualization::PointCloudColorHandlerRGBField<Cloud::PointType> rgb3(tmp);
        viewer.addPointCloud<Cloud::PointType> (tmp, rgb3, "cloud3");

        viewer.spin();

        std::cout << "ICP registration" << std::endl;

        auto registered = icpRegistration(tmp, result);

        viewer.removePointCloud("cloud3");
        pcl::visualization::PointCloudColorHandlerRGBField<Cloud::PointType> rgb4(registered);
        viewer.addPointCloud<Cloud::PointType> (registered, rgb4, "cloud4");

        viewer.spin();

        *result += *registered;
    }

    return result;
}

Cloud::Ptr method2(const Data &data) {
    Cloud::Ptr result(new Cloud);
    pcl::copyPointCloud(*data[0].cloud, *result);

    std::vector<Cloud::Ptr> registeredClouds;
    for (auto &cloud : registeredClouds)
        cloud = Cloud::Ptr(new Cloud);

    for (size_t i = 1; i < data.size(); ++i) {
        const auto &cloud = data[i].cloud;

        //register cloud with result
        Eigen::Matrix4f transformation;
        correspondenceEstimation(cloud, result, transformation);

        Cloud::Ptr tmp(new Cloud);
        pcl::transformPointCloud(*cloud, *tmp, transformation); //transform current pair into the global transform

        auto registered = icpRegistration(tmp, result);

        registeredClouds.push_back(registered);
    }

    for (const auto &registered : registeredClouds) {
        *result += *registered;
    }

    return result;
}


void correspondenceEstimation(Cloud::ConstPtr source, Cloud::ConstPtr target, Eigen::Matrix4f &transformation) {
    pcl::registration::CorrespondenceEstimation<Cloud::PointType, Cloud::PointType> est;
    pcl::CorrespondencesPtr corr(new pcl::Correspondences);

    est.setInputSource(source);
    est.setInputTarget(target);
    est.determineReciprocalCorrespondences(*corr);

    pcl::registration::CorrespondenceRejectorSampleConsensus<Cloud::PointType> rejector;
    rejector.setInputSource(source);
    rejector.setInputTarget(target);
    rejector.setInlierThreshold(0.1);
    rejector.setMaximumIterations(10);
    rejector.setInputCorrespondences(corr);

    pcl::Correspondences inliers;
    rejector.getCorrespondences(inliers);

    transformation = rejector.getBestTransformation();
}

Cloud::Ptr icpRegistration(Cloud::ConstPtr source, Cloud::ConstPtr target) {
    pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);

    icp.setInputSource(source);
    icp.setInputTarget(target);

    Cloud::Ptr alignedCloud(new Cloud);
    icp.align(*alignedCloud);
    icp.
}

void visualize(Cloud::ConstPtr cloud1, Cloud::ConstPtr cloud2) {

    pcl::visualization::PCLVisualizer viewer;

    // Draw output point cloud:
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (0.1);
    viewer.addText ("Clouds", 10, 10);

    int vp1, vp2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, vp2);

    pcl::visualization::PointCloudColorHandlerRGBField<Cloud::PointType> rgb1(cloud1);
    viewer.addPointCloud<Cloud::PointType> (cloud1, rgb1, "cloud1");

    pcl::visualization::PointCloudColorHandlerRGBField<Cloud::PointType> rgb2(cloud2);
    viewer.addPointCloud<Cloud::PointType> (cloud2, rgb2, "cloud2");

    // Loop for visualization (so that the visualizers are continuously updated):
    std::cout << "Visualization... "<< std::endl;
    while (!viewer.wasStopped ())
    {
        viewer.spin ();
    }
}

