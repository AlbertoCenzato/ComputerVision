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

using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>


CloudRGB::Ptr removeGroundPlane(CloudRGB::Ptr cloud);

CloudRGB::Ptr clusterOrHardThreshold(CloudRGB::Ptr cloud);

CloudRGB::Ptr registerClouds(std::vector<CloudRGB::Ptr> &clouds);


int main (int argc, char** argv) {
	// Variables declaration:
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

		// Visualization:
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");

		// Draw output point cloud:
		viewer.setBackgroundColor (0, 0, 0);
		viewer.addCoordinateSystem (0.1);
		viewer.addText ("Cloud 1", 10, 10);
		pcl::visualization::PointCloudColorHandlerRGBField<CloudRGB::PointType> rgb(cloud);
		viewer.addPointCloud<CloudRGB::PointType>(cloud, rgb, "cloud");

		// Loop for visualization (so that the visualizers are continuously updated):
		std::cout << "Visualization... "<< std::endl;

		viewer.spin ();

		auto cloudNoGround = removeGroundPlane(cloud);

		// visualize

		auto naoCloud = clusterOrHardThreshold(cloudNoGround);

		// visualize

		naoClouds.push_back(naoCloud);
	}

	auto finalCloud = registerClouds(naoClouds);

	// visualize

	return 0;
}
