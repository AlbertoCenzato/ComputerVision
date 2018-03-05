/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "visualizer.h"
#include "utils.h"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
using pcl::PointXYZ;
using pcl::PointNormal;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudWithNormals = pcl::PointCloud<pcl::PointNormal>;

// Define a new point representation for < x, y, z, curvature >
class PointRepresentationCurv : public pcl::PointRepresentation <PointNormal>
{
    using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
public:
    PointRepresentationCurv ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormal &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

// Define a new point representation for < x, y, z, curvature >
class PointRepresentationXYZ : public pcl::PointRepresentation <PointNormal>
{
    using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
public:
    PointRepresentationXYZ ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 3;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormal &p, float * out) const
    {
        // < x, y, z >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
    }
};




////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
template<typename PointRepres>
void pairAlign (const Cloud::Ptr cloud_src, const Cloud::Ptr cloud_tgt, Cloud::Ptr output,
                Eigen::Matrix4f &final_transform, Visualizer &vis, bool downsample = false)
{
    // Compute surface normals and curvature
    CloudWithNormals::Ptr points_with_normals_src (new CloudWithNormals);
    CloudWithNormals::Ptr points_with_normals_tgt (new CloudWithNormals);

    pcl::NormalEstimation<PointXYZ, PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (cloud_src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*cloud_src, *points_with_normals_src);

    norm_est.setInputCloud (cloud_tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*cloud_tgt, *points_with_normals_tgt);

    // Instantiate our custom point representation (defined above) ...
    PointRepres point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    std::vector<float> alpha(point_representation.getNumberOfDimensions(), 1.0f);
    point_representation.setRescaleValues(alpha.data());

    // Align
    pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const PointRepres> (point_representation));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    CloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO ("Iteration Nr. %d.\n", i); std::cout << std::flush;

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
        if (point_representation.getNumberOfDimensions() == 4)
            vis.showCloudsTopRight<PointNormal>(points_with_normals_tgt, points_with_normals_src);
        else
            vis.showCloudsBottomRight<PointNormal>(points_with_normals_tgt, points_with_normals_src);
    }

    targetToSource = Ti.inverse(); // Get the transformation from target to source

    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource); // Transform target back in source frame

    if (point_representation.getNumberOfDimensions() == 4)
        vis.showIterTop<Cloud::PointType>(output, cloud_src);
    else
        vis.showIterBottom<Cloud::PointType>(output, cloud_src);

    *output += *cloud_src;  //add the source to the transformed target

    final_transform = targetToSource;
}



/* ---[ */
int main (int argc, char** argv) {

    auto data = loadData<Cloud::PointType>(argc, argv, true); // Load data

    // Check user input
    if (data.empty()) {
        PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
        PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
        return (-1);
    }
    PCL_INFO ("Loaded %d datasets.", int(data.size()));
    std::cout << std::flush;

    // Create a PCLVisualizer object
    Visualizer visualizer(argc, argv, "Pairwise Incremental Registration example");

    Cloud::Ptr resultWithCurvature(new Cloud);
    Cloud::Ptr resultXYZ(new Cloud);
    Cloud::Ptr source, target;

    Eigen::Matrix4f GlobalTransformCurv = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f GlobalTransformXYZ  = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pairTransform;

    for (size_t i = 1; i < data.size(); ++i) {
        source = data[i-1].cloud;
        target = data[i].cloud;

        // Add visualization data
        visualizer.showCloudsBottomLeft<Cloud::PointType>(source, target);
        visualizer.showCloudsTopLeft   <Cloud::PointType>(source, target);

        // transform point clouds using curvature
        Cloud::Ptr temp(new Cloud);
        PCL_INFO ("Aligning %s (%d) with %s (%d) using curvature.\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        std::cout << std::flush;

        pairAlign<PointRepresentationCurv>(source, target, temp, pairTransform, visualizer, true);

        pcl::transformPointCloud(*temp, *resultWithCurvature, GlobalTransformCurv); //transform current pair into the global transform

        GlobalTransformCurv = pairTransform * GlobalTransformCurv; //update the global transform

        // transform point cloud using only XYZ
        temp->clear();
        PCL_INFO ("Aligning %s (%d) with %s (%d) without curvature.\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        std::cout << std::flush;

        pairAlign<PointRepresentationXYZ>(source, target, temp, pairTransform, visualizer, true);

        pcl::transformPointCloud(*temp, *resultXYZ, GlobalTransformXYZ); //transform current pair into the global transform

        GlobalTransformXYZ = pairTransform * GlobalTransformXYZ; //update the global transform


        //save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
        ss << i << ".pcd";
        pcl::io::savePCDFile(ss.str (), *resultWithCurvature, true);

        ss.clear();
        ss << i << ".pcd";
        pcl::io::savePCDFile(ss.str (), *resultXYZ, true);
    }
}
/* ]--- */
