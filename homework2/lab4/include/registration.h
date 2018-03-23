//
// Created by alberto on 22/03/18.
//

#ifndef CV_HW2_LAB4_FPFH_H
#define CV_HW2_LAB4_FPFH_H

#include <thread>

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/vfh.h>

namespace lab4 {

    template<typename PointT>
    pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(const typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        pcl::NormalEstimationOMP<PointT, pcl::PointNormal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);

        // Output datasets
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(0.03);

        // Compute the features
        std::cout << "Computing normals...please wait... " << std::flush;
        auto numOfThreads = std::thread::hardware_concurrency();
        ne.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP
        ne.compute(*cloudNormals);
        std::cout << "done." << std::endl;

        return cloudNormals;
    }


    pcl::PointCloud<pcl::PointWithScale>::Ptr computeSIFT(pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals)
    {
        const float min_scale = 0.01f;
        const int n_octaves = 3;
        const int n_scales_per_octave = 4;
        const float min_contrast = 0.001f;

        pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
        pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        sift.setSearchMethod(tree);
        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(cloudNormals);
        sift.compute(*result);

        return result;
    }

    template<typename PointT>
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(const typename pcl::PointCloud<PointT>::ConstPtr cloud,
                                                           pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals)
    {
        // Create the FPFH estimation class, and pass the input dataset+normals to it
        pcl::FPFHEstimationOMP<PointT, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(cloudNormals);

        // Output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        fpfh.setRadiusSearch(0.05);

        // Compute the features
        std::cout << "Computing FPFH features...please wait... " << std::flush;
        auto numOfThreads = std::thread::hardware_concurrency();
        fpfh.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP
        fpfh.compute (*fpfhs);
        std::cout << "done." << std::endl;

        return fpfhs;
    }

    template<typename PointT>
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr computePFHRGB(const typename pcl::PointCloud<PointT>::ConstPtr cloud,
                                                             pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals)
    {
        pcl::PFHRGBEstimation<PointT, pcl::PointNormal, pcl::PFHRGBSignature250> pfhrgb;
        pfhrgb.setInputCloud(cloud);
        pfhrgb.setInputNormals(cloudNormals);

        // Output datasets
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgbs(new pcl::PointCloud<pcl::PFHRGBSignature250>);

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        pfhrgb.setRadiusSearch(0.06);

        // Compute the features
        std::cout << "Computing PFHRGB features...please wait... " << std::flush;
        //auto numOfThreads = std::thread::hardware_concurrency();
        //pfhrgb.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP
        pfhrgb.compute (*pfhrgbs);
        std::cout << "done." << std::endl;

        return pfhrgbs;
    }

    template<typename PointT>
    pcl::PointCloud<pcl::VFHSignature308>::Ptr computeVFH(const typename pcl::PointCloud<PointT>::ConstPtr cloud,
                                                          const pcl::PointCloud<pcl::PointNormal>::ConstPtr normals)
    {
        // Create the VFH estimation class, and pass the input dataset+normals to it
        pcl::VFHEstimation<PointT, pcl::PointNormal, pcl::VFHSignature308> vfh;
        vfh.setInputCloud(cloud);
        vfh.setInputNormals(normals);
        // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        vfh.setSearchMethod(tree);
        vfh.setRadiusSearch(0.05);

        // Output datasets
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

        // Compute the features
        vfh.compute (*vfhs);
        return vfhs;
    }


    template<typename PointT>
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extractFPFHDescriptors(const typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        auto cloud_normals = computeNormals<PointT>(cloud);

        auto keyPoints = computeSIFT(cloud_normals);

        return computeFPFH<PointT>(cloud, cloud_normals);
    }

    template<typename PointT>
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr extractPFHRGBDescriptors(const typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        auto cloud_normals = computeNormals<PointT>(cloud);

        auto keyPoints = computeSIFT(cloud_normals);

        return computePFHRGB<PointT>(cloud, cloud_normals);
    }

    template<typename PointT>
    pcl::PointCloud<pcl::VFHSignature308>::Ptr extractVFHDescriptors(const typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        auto cloud_normals = computeNormals<PointT>(cloud);

        auto keyPoints = computeSIFT(cloud_normals);

        return computeVFH<PointT>(cloud, cloud_normals);
    }

} // namespace lab4

#endif //CV_HW2_LAB4_FPFH_H
