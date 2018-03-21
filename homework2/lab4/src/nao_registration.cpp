/*
 * main_show_clouds.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

#include "plane_segmenter.h"
#include "simple_viewer.h"

using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

CloudRGB::Ptr removeGroundPlane(CloudRGB::ConstPtr cloud);

CloudRGB::Ptr clusterOrHardThreshold(CloudRGB::Ptr cloud);

CloudRGB::Ptr registerClouds(std::vector<CloudRGB::Ptr> &clouds);


int main (int argc, char** argv) {

    lab4::SimpleViewer viewer("Nao");
	std::vector<CloudRGB> naoClouds;


	for (unsigned int i = 1; i <= 6; i++) {
		CloudRGB::Ptr cloud (new CloudRGB);

		// Load point cloud from .pcd file:
		std::stringstream ss;
		ss << "../dataset_lab4/nao/" << i << ".pcd";
		if (pcl::io::loadPCDFile<CloudRGB::PointType> (ss.str(), *cloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read the pcd file \n");
			return (-1);
		}

		viewer.visualize(cloud);

		auto cloudNoGround = removeGroundPlane(cloud);

        viewer.visualize(cloudNoGround);

		//auto naoCloud = clusterOrHardThreshold(cloudNoGround);

		//viewer.visualize(naoCloud);

		//naoClouds.push_back(naoCloud);
	}

	//auto finalCloud = registerClouds(naoClouds);

	// viewer.visualize(finalCloud);

	return 0;
}


CloudRGB::Ptr removeGroundPlane(CloudRGB::ConstPtr cloud)
{
	lab4::PlaneSegmenter<CloudRGB::PointType> seg;
	seg.setAxisAndTolerance({0.f,1.f,0.f});
    seg.setDistanceThreshold(0.05);
	seg.estimatePlane(cloud);
	return seg.getSegmentedCloud();
}