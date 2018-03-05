/*
 * demo_simple_operations.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int main (int argc, char** argv)
{
	// Variables declaration:
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Load point cloud from .pcd file:
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset_lab1/table_scene_lms400.pcd",*cloud) == -1) {
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}

	std::cout << "Loaded " << cloud->width * cloud->height
			  << " data points from the pcd file. " << std::endl;
	std::cerr << "Cloud before filtering: " << cloud->width * cloud->height
			  << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create two new clouds from the original one:
	// we want to put in cloud1 points with y < 0
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (const auto &point : cloud->points) {
		if (point.x < 0) {
            pcl::PointXYZRGB p(0,0,255);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
		    cloud_out->points.push_back(p);
		}
	}

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// Draw output point cloud:
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(0.1, "cloud");
	viewer.addText("Cloud 1", 10, 10);
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_out);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_out, rgb, "cloud");

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	
	viewer.spin ();
	
	return 0;
}
