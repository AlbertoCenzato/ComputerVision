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
 * adaptation Raphael Favier
 * adaptation Alberto Cenzato */

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

/* ---[ */
int main (int argc, char** argv) {

    auto data = lab3::loadData<Cloud::PointType>(argc, argv, true); // Load data

    // Check user input
    if (data.empty()) {
        PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
        PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
        return (-1);
    }
    PCL_INFO ("Loaded %d datasets.", int(data.size()));
    std::cout << std::endl;

    // Create a PCLVisualizer object
    lab3::Visualizer visualizer("Pairwise Incremental Registration example");

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
        visualizer.showCloudsBottomLeft<Cloud::PointType>({source, target});
        visualizer.showCloudsTopLeft   <Cloud::PointType>({source, target});

        // transform point clouds using curvature
        Cloud::Ptr temp(new Cloud);
        PCL_INFO ("Aligning %s (%d) with %s (%d) using curvature.\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        std::cout << std::flush;

        lab3::pairAlign<lab3::PointRepresentationCurv, Cloud::PointType>(source, target, temp, pairTransform, visualizer, false);

        pcl::transformPointCloud(*temp, *resultWithCurvature, GlobalTransformCurv); //transform current pair into the global transform

        GlobalTransformCurv = pairTransform * GlobalTransformCurv; //update the global transform

        // transform point cloud using only XYZ
        temp->clear();
        PCL_INFO ("Aligning %s (%d) with %s (%d) without curvature.\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        std::cout << std::flush;

        lab3::pairAlign<lab3::PointRepresentationXYZ, Cloud::PointType>(source, target, temp, pairTransform, visualizer, true);

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
