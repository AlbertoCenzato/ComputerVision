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

namespace lab4 {

    template<typename PointT>
    class MultiCloudRegister {
    public:
        using PointType = PointT;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PointCloudTPtr = typename PointCloudT::Ptr;
        using PointCloudTConstPtr = typename PointCloudT::ConstPtr;

        MultiCloudRegister() : registeredCloud(new PointCloudT) { }

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

                //std::cout << "Correspondence estimation, cloud 0 vs cloud " << i << std::endl;

                //visualizer.showCloudsBottomLeft<Cloud::PointType>({result, cloud});

                //register cloud with result
                Eigen::Matrix4f transform;
                correspondenceEstimation(cloud, registeredCloud, transform);

                PointCloudTPtr tmp(new PointCloudT);
                pcl::transformPointCloud(*cloud, *tmp, transform); //transform current pair into the global transform

                //visualizer.showCloudsBottomLeft<Cloud::PointType>({result, tmp});

                std::cout << "ICP registration, cloud 0 vs cloud " << i << std::endl;

                PointCloudTPtr registered(new PointCloudT);
                pairAlign(tmp, registeredCloud, registered, transform/*,visualizer, false*/);

                registeredClouds.push_back(registered);
            }

            for (const auto &registered : registeredClouds) {
                *registeredCloud += *registered;
            }

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

        void correspondenceEstimation(PointCloudTConstPtr source, PointCloudTConstPtr target,
                                      Eigen::Matrix4f &transformation) const {
            pcl::registration::CorrespondenceEstimation<PointT, PointT> est;
            pcl::CorrespondencesPtr corr(new pcl::Correspondences);

            est.setInputSource(source);
            est.setInputTarget(target);
            est.determineReciprocalCorrespondences(*corr);

            pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
            rejector.setInputSource(source);
            rejector.setInputTarget(target);
            rejector.setInlierThreshold(0.5);
            rejector.setMaximumIterations(30);
            rejector.setInputCorrespondences(corr);

            pcl::Correspondences inliers;
            rejector.getCorrespondences(inliers);

            transformation = rejector.getBestTransformation();
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
         * \param cloud_src the source PointCloud
         * \param cloud_tgt the target PointCloud
         * \param output the resultant aligned source PointCloud
         * \param final_transform the resultant transform between source and target
         */
        void pairAlign(PointCloudTConstPtr cloud_src, PointCloudTConstPtr cloud_tgt, PointCloudTPtr output,
                       Eigen::Matrix4f &final_transform) const
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);
            computeNormalsAndCurv(cloud_src, cloud_tgt, points_with_normals_src, points_with_normals_tgt);

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

                /*
                // visualize current state
                if (visualizeTop)
                    vis.showCloudsTopRight<pcl::PointNormal>({points_with_normals_tgt, points_with_normals_src});
                else
                    vis.showCloudsBottomRight<pcl::PointNormal>({points_with_normals_tgt, points_with_normals_src});
                 */
            }

            //targetToSource = Ti.inverse(); // Get the transformation from target to source

            pcl::transformPointCloud(*cloud_src, *output, Ti/*targetToSource*/); // Transform target back in source frame

            /*
            if (visualizeTop)
                vis.showIterTop<PointT>(output, cloud_tgt);
            else
                vis.showIterBottom<PointT>(output, cloud_tgt);
            */

            //*output += *cloud_src;  //add the source to the transformed target

            final_transform = Ti;//targetToSource;
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
