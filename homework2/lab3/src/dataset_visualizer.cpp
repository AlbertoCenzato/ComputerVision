/*
 * dataset_visualizer.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

typedef pcl::PointXYZRGB PointXYZ;

int
main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<PointXYZ>::Ptr cloud1 (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZ>::Ptr cloud2 (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZ>::Ptr cloud3 (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZ>::Ptr cloud4 (new pcl::PointCloud<PointXYZ>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<PointXYZ> ("../dataset_lab3/minimouse1.pcd", *cloud1) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<PointXYZ> ("../dataset_lab3/minimouse2.pcd", *cloud2) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<PointXYZ> ("../dataset_lab3/minimouse3.pcd", *cloud3) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<PointXYZ> ("../dataset_lab3/minimouse4.pcd", *cloud4) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	std::cout << "Loaded 4 files" << std::endl;

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// Draw output point cloud:
	viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (0.1);
	viewer.addText ("Clouds", 10, 10);
	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZ> rgb1(cloud1);
	viewer.addPointCloud<PointXYZ> (cloud1, rgb1, "cloud1");

	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZ> rgb2(cloud2);
	viewer.addPointCloud<PointXYZ> (cloud2, rgb2, "cloud2");

	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZ> rgb3(cloud3);
	viewer.addPointCloud<PointXYZ> (cloud3, rgb3, "cloud3");

	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZ> rgb4(cloud4);
	viewer.addPointCloud<PointXYZ> (cloud4, rgb4, "cloud4");

	// Visualization in different viewports:
	pcl::visualization::PCLVisualizer viewer2("PCL Viewer");
	int v1(0), v2(0), v3(0), v4(0);
	viewer2.createViewPort (0.0, 0.0, 0.5, 0.5, v1);
	viewer2.createViewPort (0.5, 0.0, 1.0, 0.5, v2);
	viewer2.createViewPort (0.0, 0.5, 0.5, 1.0, v3);
	viewer2.createViewPort (0.5, 0.5, 1.0, 1.0, v4);

	viewer2.setBackgroundColor (0, 0, 0);
	viewer2.addCoordinateSystem (0.1);
	viewer2.addPointCloud<PointXYZ> (cloud1, rgb1, "cloud1", v1);
	viewer2.addPointCloud<PointXYZ> (cloud2, rgb2, "cloud2", v2);
	viewer2.addPointCloud<PointXYZ> (cloud3, rgb3, "cloud3", v3);
	viewer2.addPointCloud<PointXYZ> (cloud4, rgb4, "cloud4", v4);

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spin ();
	}
	return 0;
}
