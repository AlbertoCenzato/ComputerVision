//
// Created by alberto on 05/03/18.
//

#ifndef CV_HW2_LAB3_UTILS_H
#define CV_HW2_LAB3_UTILS_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

namespace lab3 {

//convenient structure to handle our pointclouds
    template<typename PointT>
    struct PCD {
        typename pcl::PointCloud<PointT>::Ptr cloud;
        std::string f_name;

        PCD() : cloud(new pcl::PointCloud<PointT>) {}
    };

    template<typename PointT>
    struct PCDComparator {
        bool operator()(const PCD<PointT> &p1, const PCD<PointT> &p2) {
            return (p1.f_name < p2.f_name);
        }

    };


////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
    template<typename PointT>
    std::vector<PCD<PointT>, Eigen::aligned_allocator<PCD<PointT>>> loadData(int argc, char **argv, bool downsample) {
        std::vector<PCD<PointT>, Eigen::aligned_allocator<PCD<PointT>>> models;

        std::string extension(".pcd");
        // Suppose the first argument is the actual test model
        for (int i = 1; i < argc; i++) {
            std::string fname = std::string(argv[i]);
            // Needs to be at least 5: .plot
            if (fname.size() <= extension.size())
                continue;

            std::transform(fname.begin(), fname.end(), fname.begin(), (int (*)(int)) tolower);

            //check that the argument is a pcd file
            if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0) {
                // Load the cloud and saves it into the global list of models
                PCD<PointT> m;
                m.f_name = argv[i];
                pcl::io::loadPCDFile(argv[i], *m.cloud);
                //remove NAN points from the cloud
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

                // Downsample for consistency and speed
                // \note enable this for large datasets
                if (downsample) {
                    pcl::VoxelGrid<PointT> grid;
                    grid.setLeafSize(0.01, 0.01, 0.01);
                    grid.setInputCloud(m.cloud);
                    grid.filter(*m.cloud);
                }

                models.push_back(m);
            }
        }

        return models;  // using vector<> move semantic to efficently return models
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

// Define a new point representation for < x, y, z, curvature >
    class PointRepresentationXYZ : public pcl::PointRepresentation<pcl::PointNormal> {
        using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
    public:
        PointRepresentationXYZ() {
            // Define the number of dimensions
            nr_dimensions_ = 3;
        }

        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const {
            // < x, y, z >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
        }
    };


////////////////////////////////////////////////////////////////////////////////
    template<typename PointT>
    void computeNormalsAndCurv(typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
                               typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
                               pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src,
                               pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt)
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

    /**
      * \brief Align a pair of PointCloud datasets and return the result
      * \param cloud_src the source PointCloud
      * \param cloud_tgt the target PointCloud
      * \param output the resultant aligned source PointCloud
      * \param final_transform the resultant transform between source and target
      */
    template<typename PointRepres, typename PointT>
    void pairAlign(typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
                   typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
                   typename pcl::PointCloud<PointT>::Ptr output, Eigen::Matrix4f &final_transform,
                   Visualizer &vis, bool visualizeTop = true)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);
        computeNormalsAndCurv<PointT>(cloud_src, cloud_tgt, points_with_normals_src, points_with_normals_tgt);

        // Instantiate our custom point representation (defined above) ...
        PointRepres point_representation;
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
        reg.setPointRepresentation(boost::make_shared<const PointRepres>(point_representation));

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

            // visualize current state
            if (visualizeTop)
                vis.showCloudsTopRight<pcl::PointNormal>({points_with_normals_tgt, points_with_normals_src});
            else
                vis.showCloudsBottomRight<pcl::PointNormal>({points_with_normals_tgt, points_with_normals_src});
        }

        //targetToSource = Ti.inverse(); // Get the transformation from target to source

        pcl::transformPointCloud(*cloud_src, *output, Ti/*targetToSource*/); // Transform target back in source frame

        if (visualizeTop)
            vis.showIterTop<PointT>(output, cloud_tgt);
        else
            vis.showIterBottom<PointT>(output, cloud_tgt);

        //*output += *cloud_src;  //add the source to the transformed target

        final_transform = Ti;//targetToSource;
    }


} // namespace lab3
    
#endif //CV_HW2_LAB3_UTILS_H
