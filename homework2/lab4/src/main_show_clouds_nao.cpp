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

int
main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (unsigned int i = 1; i <= 6; i++)
	{

		// Load point cloud from .pcd file:
		std::stringstream ss;
		ss << "../data/nao/" << i << ".pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (ss.str(), *cloud) == -1) //* load the file
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
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");

		// Loop for visualization (so that the visualizers are continuously updated):
		std::cout << "Visualization... "<< std::endl;
	
		viewer.spin ();
	}
	
	return 0;
}
