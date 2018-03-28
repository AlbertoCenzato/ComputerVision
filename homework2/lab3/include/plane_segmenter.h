//
// Created by alberto on 21/03/18.
//

#ifndef CV_HW2_LAB4_PLANE_SEGMENTER_H
#define CV_HW2_LAB4_PLANE_SEGMENTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace lab4 {

    template<typename PointT>
    class PlaneSegmenter {
    public:
        using PointType = PointT;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PointCloudTPtr = typename PointCloudT::Ptr;
        using PointCloudTConstPtr = typename PointCloudT::ConstPtr;


        PlaneSegmenter() : coefficients(new pcl::ModelCoefficients()), planeCloud(new PointCloudT),
                           segmentedCloud(new PointCloudT) {
            // Optional
            seg.setOptimizeCoefficients(true);
            // Mandatory
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            Eigen::Vector3f axis(0.f, 1.f, 0.f);
            seg.setAxis(axis);
            seg.setEpsAngle(30.0f * (M_PI / 180.0f)); // necessary otherwise setAxis is ignored
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(500);
            seg.setDistanceThreshold(0.01);
        }

        void setInputCloud(PointCloudTConstPtr cloud) {
            inputCloud = cloud;
            coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
            planeCloud = PointCloudTPtr(new PointCloudT);
            segmentedCloud = PointCloudTPtr(new PointCloudT);
        }


        void estimatePlane(bool filter = true) {
            PointCloudTConstPtr cloud = filter ? filterCloud(inputCloud) : inputCloud;

            // Segment the largest planar component from the remaining cloud
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                error = true;
                return;
            } else
                error = false;

            // Extract the inliers
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*planeCloud);
            std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height
                      << " data points." << std::endl;

            // Create the filtering object
            extract.setNegative(true);        // to make filter method to return "outliers" instead of "inliers"
            extract.filter(*segmentedCloud);
        }

        void estimatePlane(PointCloudTConstPtr cloud, bool filter = true) {
            setInputCloud(cloud);
            estimatePlane(filter);
        }


        PointCloudTPtr getPlaneCloud() const {
            return planeCloud;
        }

        Eigen::VectorXf getPlaneCoefficients() const {
            return Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(coefficients->values.data(),
                                                                 coefficients->values.size());
        }

        PointCloudTPtr getSegmentedCloud() const {
            return segmentedCloud;
        }

        bool hasErrors() const {
            return error;
        }

        void setAxisAndTolerance(const Eigen::Vector3f &axis, double angleTolerance = 30.0) {
            seg.setAxis(axis);
            seg.setEpsAngle(angleTolerance * (M_PI / 180.0f));
        }

        void setDistanceThreshold(double threshold) {
            seg.setDistanceThreshold(threshold);
        }


    private:
        bool error = false;

        PointCloudTConstPtr inputCloud;
        pcl::SACSegmentation<PointT> seg;

        pcl::ModelCoefficients::Ptr coefficients;

        PointCloudTPtr planeCloud, segmentedCloud;


        PointCloudTPtr filterCloud(PointCloudTConstPtr cloud) const {
            PointCloudTPtr cloudFiltered(new PointCloudT);

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f, 0.01f, 0.01f);
            sor.filter(*cloudFiltered);

            std::cout << "PointCloud after filtering: " << cloudFiltered->width * cloudFiltered->height
                      << " data points." << std::endl;

            return cloudFiltered;
        }


    };

} // namespace lab4

#endif //CV_HW2_LAB4_PLANE_SEGMENTER_H