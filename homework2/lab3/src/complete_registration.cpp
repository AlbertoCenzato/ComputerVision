//
// Created by alberto on 05/03/18.
//
#include <thread>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation

#include "visualizer.h"
#include "utils.h"

using std::vector;
using pcl::visualization::PCLVisualizer;

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudNormal = pcl::PointCloud<pcl::PointNormal>;
using pcl::visualization::PointCloudColorHandlerGenericField;

template<typename PointT>
using Data  = std::vector<lab3::PCD<PointT>, Eigen::aligned_allocator<lab3::PCD<PointT>>>;


void correspondenceEstimation(Cloud::ConstPtr source, Cloud::ConstPtr target, Eigen::Matrix4f &transformation);

Cloud::Ptr method1(const vector<Cloud::Ptr> &data, lab3::Visualizer &visualizer);

Cloud::Ptr method2(const vector<Cloud::Ptr> &data, lab3::Visualizer &visualizer);

pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(Cloud::ConstPtr cloud);


int main(int argc, char** argv) {

    auto data = lab3::loadData<Cloud::PointType>(argc, argv, true);

    // Check user input
    if (data.empty()) {
        PCL_ERROR ("Syntax is: %s <cloud1.pcd> <cloud2.pcd> ... [*]", argv[0]);
        PCL_ERROR ("[*] - multiple files can be added.");
        return (-1);
    }
    PCL_INFO ("Loaded %d datasets.", int(data.size()));
    std::cout << std::endl;

    std::vector<Cloud::Ptr> clouds;
    for (const auto &c : data)
        clouds.push_back(c.cloud);

    lab3::Visualizer visualizer("Comparison between two registration methods");

    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "---------- Registering method 1 -----------" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    auto cloud_m1 = method1(clouds, visualizer);

    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "---------- Registering method 2 -----------" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    auto cloud_m2 = method2(clouds, visualizer);

    //visualize(cloud_m1, cloud_m2);

    return 0;
}


Cloud::Ptr method1(const vector<Cloud::Ptr> &data, lab3::Visualizer &visualizer) {

    Cloud::Ptr result(new Cloud);
    pcl::copyPointCloud(*data[0], *result);

    for (size_t i = 1; i < data.size(); ++i) {

        std::cout << "Correspondence estimation, cloud " << i << " vs clouds 0 + ... + " << i-1 << std::endl;

        const auto &cloud = data[i];

        visualizer.showCloudsTopLeft<Cloud::PointType>({cloud, result});

        Eigen::Matrix4f transform;
        correspondenceEstimation(cloud, result, transform);

        Cloud::Ptr tmp(new Cloud);
        pcl::transformPointCloud(*cloud, *tmp, transform); //transform current pair into the global transform

        visualizer.showCloudsTopLeft<Cloud::PointType>({tmp, result});

        std::cout << "ICP registration, cloud " << i << " vs clouds 0 + ... + " << i-1 << std::endl;

        Cloud::Ptr registered(new Cloud);
        lab3::pairAlign<lab3::PointRepresentationCurv, Cloud::PointType>(tmp, result, registered, transform, visualizer);

        *result += *registered;
    }

    std::cout << "Final registration result" << std::endl;

    visualizer.showCloudsTopLeft<Cloud::PointType>({result});

    return result;
}

Cloud::Ptr method2(const vector<Cloud::Ptr>& data, lab3::Visualizer &visualizer) {
    Cloud::Ptr result(new Cloud);
    pcl::copyPointCloud(*data[0], *result);

    std::vector<Cloud::Ptr> registeredClouds;
    for (auto &cloud : registeredClouds)
        cloud = Cloud::Ptr(new Cloud);

    for (size_t i = 1; i < data.size(); ++i) {
        const auto &cloud = data[i];

        std::cout << "Correspondence estimation, cloud 0 vs cloud " << i << std::endl;

        visualizer.showCloudsBottomLeft<Cloud::PointType>({result, cloud});

        //register cloud with result
        Eigen::Matrix4f transform;
        correspondenceEstimation(cloud, result, transform);

        Cloud::Ptr tmp(new Cloud);
        pcl::transformPointCloud(*cloud, *tmp, transform); //transform current pair into the global transform

        visualizer.showCloudsBottomLeft<Cloud::PointType>({result, tmp});

        std::cout << "ICP registration, cloud 0 vs cloud " << i << std::endl;

        Cloud::Ptr registered(new Cloud);
        lab3::pairAlign<lab3::PointRepresentationCurv, Cloud::PointType>(tmp, result, registered, transform, visualizer, false);

        registeredClouds.push_back(registered);
    }

    for (const auto &registered : registeredClouds) {
        *result += *registered;
    }

    std::cout << "Final registration result" << std::endl;

    visualizer.showCloudsBottomLeft<Cloud::PointType>({result});

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


CloudNormal::Ptr computeNormals(Cloud::ConstPtr cloud) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<Cloud::PointType, CloudNormal::PointType> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<Cloud::PointType>::Ptr tree(new pcl::search::KdTree<Cloud::PointType>());
    ne.setSearchMethod(tree);

    // Output datasets
    CloudNormal::Ptr cloud_normals(new CloudNormal);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    std::cout << "Computing normals...please wait..." << std::flush;
    auto numOfThreads = std::thread::hardware_concurrency();
    ne.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP
    ne.compute (*cloud_normals);
    std::cout << "done." << std::endl;

    return cloud_normals;
}