/*
 * demo_compute_FPFH.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <thread>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer
#include <pcl/visualization/histogram_visualizer.h>	// for histogram visualization
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud
#include <pcl/keypoints/sift_keypoint.h>


struct callbackArgs{
	// structure used to pass arguments to the callback function
	pcl::visualization::PCLHistogramVisualizer *histViewer;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;
};

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args) {
	// callback function used to visualize feature histograms
	auto data = static_cast<struct callbackArgs *>(args);
    if (event.getPointIndex() == -1)
      return;
    std::stringstream windowName;
    windowName << "FPFH for point " << event.getPointIndex();
    data->histViewer->addFeatureHistogram (*(data->fpfhs), "fpfh", event.getPointIndex(), windowName.str(), 640, 200);
}


pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    std::cout << "Computing normals...please wait..." << std::flush;
    auto numOfThreads = std::thread::hardware_concurrency();
    ne.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP
    ne.compute (*cloud_normals);
    std::cout << "done." << std::endl;

    return cloud_normals;
}


pcl::PointCloud<pcl::PointWithScale>::Ptr computeSIFT(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals) {
    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;

    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(*result);

    return result;
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals) {
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch(0.05);

    // Compute the features
    std::cout << "Computing FPFH features...please wait..." << std::flush;
    auto numOfThreads = std::thread::hardware_concurrency();
    fpfh.setNumberOfThreads(numOfThreads); 	// set number of threads when using OpenMP
    fpfh.compute (*fpfhs);
    std::cout << "done." << std::endl;

    return fpfhs;
}

int main (int argc, char** argv) {
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> ());

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../dataset_lab2/minimouse1_segmented.pcd", *cloud) == -1) {
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}

	std::cout << "Loaded " << cloud->width * cloud->height
			  << " data points from the pcd file. " << std::endl;

	// Remove NaN from the point cloud:
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    auto cloud_normals = computeNormals(cloud);

    auto keyPoints = computeSIFT(cloud_normals);

    auto fpfhs = computeFPFH(cloud, cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

	// Visualize FPFH:
	int normalsVisualizationStep = 100; // to visualize a normal every normalsVisualizationStep
	float normalsScale = 0.02;			// normals dimension

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PCLHistogramVisualizer histViewer;
	viewer.setBackgroundColor (0.0, 0.0, 0.5);
	viewer.addCoordinateSystem (0.1);
	viewer.initCameraParameters ();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "input_cloud");
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud, cloud_normals, normalsVisualizationStep, normalsScale, "normals");

	// Create structure containing the parameters for the callback function
	struct callbackArgs histCallbackArgs;
	histCallbackArgs.histViewer = &histViewer;
	histCallbackArgs.fpfhs = fpfhs;

	// Add point picking callback to viewer (for visualizing feature histograms):
	viewer.registerPointPickingCallback(pp_callback, static_cast<void*>(&histCallbackArgs));

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	while (!viewer.wasStopped()) {
		viewer.spin ();
		histViewer.spin();
	}

	return 0;
}
