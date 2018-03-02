/*
 * demo_compute_normal.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <thread>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>	    // for computing normals
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud

int main (int argc, char** argv) {
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../dataset_lab2/minimouse1_segmented.pcd", *cloud) == -1) {
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}

	std::cout << "Loaded " << cloud->width * cloud->height
			  << " data points from the pcd file. "	<< std::endl;

	// Create the normal estimation class, and pass the input dataset to it
	//  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2(new pcl::PointCloud<pcl::Normal>);

  	std::cout << "Computing normals...please wait..." << std::flush;
	auto numOfThreads = std::thread::hardware_concurrency();
	ne.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP

    // Compute the normals
    ne.setRadiusSearch(0.03);           // Use all neighbors in a sphere of radius 3cm
	ne.compute(*cloud_normals_1);

    ne.setRadiusSearch(0.002);
    ne.compute(*cloud_normals_2);
	std::cout << "done." << std::endl;

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

	// Visualize normals
	int normalsVisualizationStep = 100; // to visualize a normal every normalsVisualizationStep
	float normalsScale = 0.02;

	pcl::visualization::PCLVisualizer viewer("Point clouds");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addCoordinateSystem(0.1);
	viewer.initCameraParameters();

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud);

	// Visualization in different viewports:
	int v1(0), v2(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 1, v2);

	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb1, "cloud1", v1);
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals_1, normalsVisualizationStep, normalsScale, "normals1", v1);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb2, "cloud2", v2);
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals_2, normalsVisualizationStep, normalsScale, "normals2", v2);

	std::cout << "Visualization...: "<< std::endl;
	while (!viewer.wasStopped()) {
		viewer.spin();
	}

	return 0;
}
