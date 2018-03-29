/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a PCD file.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>

#include "plane_segmenter.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/// PCL viewer ///
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

enum { COLS = 640, ROWS = 480 };

int print_help()
{
    cout << "*******************************************************" << std::endl;
    cout << "Ground based people detection app options:" << std::endl;
    cout << "   --help    <show_this_help>" << std::endl;
    cout << "   --svm     <path_to_svm_file>" << std::endl;
    cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
    cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
    cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
    cout << "   --sample  <sampling_factor (default = 1)>" << std::endl;
    cout << "*******************************************************" << std::endl;
    return 0;
}


int main (int argc, char** argv)
{
    if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

    /// Dataset Parameters:
    std::string filename = "../dataset_lab4/people/five_people.pcd";
    std::string svm_filename = "../dataset_lab4/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
    float min_confidence = -1.5;
    float min_height = 1.3;
    float max_height = 2.3;
    float voxel_size = 0.02;
    float sampling_factor = 1;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5,
                             0.0, 525, 239.5,
                             0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

    // Read if some parameters are passed from command line:
    pcl::console::parse_argument(argc, argv, "--file", filename);
    pcl::console::parse_argument(argc, argv, "--svm", svm_filename);
    pcl::console::parse_argument(argc, argv, "--conf", min_confidence);
    pcl::console::parse_argument(argc, argv, "--min_h", min_height);
    pcl::console::parse_argument(argc, argv, "--max_h", max_height);
    pcl::console::parse_argument(argc, argv, "--sample", sampling_factor);

    // Read Kinect data:
    PointCloudT::Ptr cloud = PointCloudT::Ptr (new PointCloudT);
    if (pcl::io::loadPCDFile(filename, *cloud) < 0)
    {
        cerr << "Failed to read test file `five_people.pcd`." << endl;
        return (-1);
    }

    // Create classifier for people detection:
    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

    // People detection app initialization:
    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
    people_detector.setVoxelSize(voxel_size);                        // set the voxel size
    people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    people_detector.setClassifier(person_classifier);                // set person classifier
    people_detector.setPersonClusterLimits(min_height, max_height, 0.1f, 1.6f);         // set person classifier
    people_detector.setSamplingFactor(sampling_factor);              // set a downsampling factor to the point cloud (for increasing speed)
//  people_detector.setSensorPortraitOrientation(true);              // set sensor orientation to vertical

    // Display pointcloud:
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    lab4::PlaneSegmenter<PointT> seg;
    seg.setAxisAndTolerance({0.f,1.f,0.f});
    seg.setDistanceThreshold(0.1);
    seg.estimatePlane(cloud);
    auto groundCoeffs = seg.getPlaneCoefficients();
    auto segmentedCloud = seg.getSegmentedCloud();
    auto planeCloud = seg.getPlaneCloud();

    // Draw output point cloud:
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(0.1);
    viewer.addText("Cloud 1", 10, 10);
    viewer.addPointCloud<PointT>(segmentedCloud, "cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(planeCloud, 255, 0, 0);
    viewer.addPointCloud<PointT>(planeCloud, red, "plane");

    // Loop for visualization (so that the visualizers are continuously updated):
    std::cout << "Visualization...press Q to continue. " << std::endl;
    while (!viewer.wasStopped()) {
        viewer.spin();
    }

    // Initialize new viewer:
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Perform people detection on the new cloud:
    std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
    people_detector.setInputCloud(cloud);
    people_detector.setGround(groundCoeffs);                    // set floor coefficients
    people_detector.compute(clusters);                           // perform people detection
    std::cout << "A" << std::endl;
    groundCoeffs = people_detector.getGround();                 // get updated floor coefficients

    // Draw cloud and people bounding boxes in the viewer:
    viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
    unsigned int k = 0;
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
        {
            // draw theoretical person bounding box in the PCL viewer:
            it->drawTBoundingBox(viewer, k);
            k++;
        }
    }
    std::cout << k << " people found" << std::endl;
    viewer.spin();

    return 0;
}