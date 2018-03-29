//
// Created by alberto on 21/03/18.
//

#ifndef CV_HW2_LAB4_MULTI_CLOUD_REGISTER_H
#define CV_HW2_LAB4_MULTI_CLOUD_REGISTER_H

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <thread>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_rejection_features.h>

#include "registration.h"

namespace lab4 {

    template<typename PointT>
    class MultiCloudRegister {
    public:
        using PointType = PointT;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PointCloudTPtr = typename PointCloudT::Ptr;
        using PointCloudTConstPtr = typename PointCloudT::ConstPtr;

        MultiCloudRegister() : registeredCloud(new PointCloudT), keypoints(new pcl::PointCloud<pcl::FPFHSignature33>) { }

        void clearRegisteredCloud()
        {
            registeredCloud = PointCloudTPtr(new PointCloudT);
        }

        PointCloudTPtr registerClouds()
        {
            if (registeredCloud->empty())
                pcl::copyPointCloud(*cloudsToRegister[0], *registeredCloud);

            std::vector<PointCloudTPtr> registeredClouds;
            for (auto &cloud : registeredClouds)
                cloud = PointCloudTPtr(new PointCloudT);

            for (size_t i = 1; i < cloudsToRegister.size(); ++i) {
                const auto &cloud = cloudsToRegister[i];

                Eigen::Matrix4f transform;
                correspondenceEstimation(cloud, registeredCloud, transform);

                PointCloudTPtr tmp(new PointCloudT);
                pcl::transformPointCloud(*cloud, *tmp, transform); //transform current pair into the global transform

                pcl::visualization::PCLVisualizer viewer("After tranformation");
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbSrc(tmp);
                viewer.addPointCloud<PointT>(tmp, rgbSrc, "source_cloud");

                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbTgt(registeredCloud);
                viewer.addPointCloud<PointT>(registeredCloud, rgbTgt, "target_cloud");
                viewer.spin();

                //-------------------------------

                std::cout << "ICP registration, cloud 0 vs cloud " << i << std::endl;

                PointCloudTPtr registered(new PointCloudT);
                pairAlign(tmp, registeredCloud, registered, transform/*,visualizer, false*/);

                viewer.removeAllPointClouds();
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbSrc1(registered);
                viewer.addPointCloud<PointT>(registered, rgbSrc1, "source_cloud");

                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbTgt1(registeredCloud);
                viewer.addPointCloud<PointT>(registeredCloud, rgbTgt1, "target_cloud");
                viewer.spin();

                //registeredClouds.push_back(registered);
                *registeredCloud += *registered;
            }

/*
            for (const auto &registered : registeredClouds) {
                *registeredCloud += *registered;
            }
            */

            cloudsToRegister.clear();

            return registeredCloud;
        }

        PointCloudTPtr registerClouds(const std::vector<PointCloudTConstPtr> &clouds)
        {
            addCloudsToRegister(clouds);
            return registerClouds();
        }

        void addCloudsToRegister(const std::vector<PointCloudTConstPtr> &clouds)
        {
            for (const auto &cloud : clouds)
                cloudsToRegister.push_back(cloud);
        }

        PointCloudTPtr getRegisteredCloud() const
        {
            return registeredCloud;
        }

    private:
        PointCloudTPtr registeredCloud;
        std::vector<PointCloudTConstPtr> cloudsToRegister;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr keypoints;

        void correspondenceEstimation(PointCloudTConstPtr source, PointCloudTConstPtr target,
                                      Eigen::Matrix4f &transformation)
        {
            auto sourceKeypoints = extractFPFHDescriptors<PointT>(source);
            if (keypoints->empty())
                keypoints = extractFPFHDescriptors<PointT>(target);

            pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
            pcl::CorrespondencesPtr corr(new pcl::Correspondences);

            est.setInputSource(sourceKeypoints);
            est.setInputTarget(keypoints);
            est.determineReciprocalCorrespondences(*corr);


            pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;

            rejector.setInputSource(source);
            rejector.setInputTarget(target);
            rejector.setInlierThreshold(0.03); //0.03);
            rejector.setMaximumIterations(30);
            rejector.setInputCorrespondences(corr);

            pcl::Correspondences inliers;
            rejector.getCorrespondences(inliers);

            pcl::visualization::PCLVisualizer viewer("Correspondeces");
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbSrc(source);
            viewer.addPointCloud<PointT>(source, rgbSrc, "source_cloud");

            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbTgt(target);
            viewer.addPointCloud<PointT>(target, rgbTgt, "target_cloud");

            viewer.addCorrespondences<PointT>(source, target, inliers);

            viewer.spin();

            transformation = rejector.getBestTransformation();

            *keypoints += *sourceKeypoints;
        }


        // Define a new point representation for < x, y, z, curvature >
        class PointRepresentationCurv : public pcl::PointRepresentation<pcl::PointNormal> {
            using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
        public:
            PointRepresentationCurv() {
                // Define the number of dimensions
                nr_dimensions_ = 4;
            }

            // Override the copyToFloatArray method to define our feature vector
            virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const {
                // < x, y, z, curvature >
                out[0] = p.x;
                out[1] = p.y;
                out[2] = p.z;
                out[3] = p.curvature;
            }
        };


        /**
         * \brief Align a pair of PointCloud datasets and return the result
         * \param cloudSrc the source PointCloud
         * \param cloudTgt the target PointCloud
         * \param output the resultant aligned source PointCloud
         * \param finalTransform the resultant transform between source and target
         */
        void pairAlign(PointCloudTConstPtr cloudSrc, PointCloudTConstPtr cloudTgt, PointCloudTPtr output,
                       Eigen::Matrix4f &finalTransform) const
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);
            computeNormalsAndCurv(cloudSrc, cloudTgt, points_with_normals_src, points_with_normals_tgt);

            // Instantiate our custom point representation (defined above) ...
            PointRepresentationCurv point_representation;
            // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
            std::vector<float> alpha(point_representation.getNumberOfDimensions(), 1.0f);
            point_representation.setRescaleValues(alpha.data());

            // Align
            pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
            reg.setTransformationEpsilon(1e-6);
            // Set the maximum distance between two correspondences (src<->tgt) to 10cm
            // Note: adjust this based on the size of your datasets
            reg.setMaxCorrespondenceDistance(0.1);
            // Set the point representation
            reg.setPointRepresentation(boost::make_shared<const PointRepresentationCurv>(point_representation));

            reg.setInputSource(points_with_normals_src);
            reg.setInputTarget(points_with_normals_tgt);
            reg.setMaximumIterations(2);

            // Run the same optimization in a loop and visualize the results
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f prev, targetToSource;

            pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;

            for (int i = 0; i < 30; ++i) {
                std::cout << "Iteration Nr. " << i << std::endl;

                // save cloud for visualization purpose
                points_with_normals_src = reg_result;

                // Estimate
                reg.setInputSource(points_with_normals_src);
                reg.align(*reg_result);

                //accumulate transformation between each Iteration
                Ti = reg.getFinalTransformation() * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
                    reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

                prev = reg.getLastIncrementalTransformation();
            }

            pcl::transformPointCloud(*cloudSrc, *output, Ti); // Transform target back in source frame

            finalTransform = Ti;
        }


        void computeNormalsAndCurv(PointCloudTConstPtr cloud_src, PointCloudTConstPtr cloud_tgt,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt) const
        {
            // Compute surface normals and curvature
            pcl::NormalEstimation<PointT, pcl::PointNormal> norm_est;
            typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
            norm_est.setSearchMethod(tree);
            norm_est.setKSearch(30);

            norm_est.setInputCloud(cloud_src);
            norm_est.compute(*points_with_normals_src);
            pcl::copyPointCloud(*cloud_src, *points_with_normals_src);

            norm_est.setInputCloud(cloud_tgt);
            norm_est.compute(*points_with_normals_tgt);
            pcl::copyPointCloud(*cloud_tgt, *points_with_normals_tgt);
        }





    };



} // namespace lab4
#endif //CV_HW2_LAB4_MULTI_CLOUD_REGISTER_H
