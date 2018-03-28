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


pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

    // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i < cloud_normals->points.size(); ++i) {
        cloud_normals->points[i].x = cloud->points[i].x;
        cloud_normals->points[i].y = cloud->points[i].y;
        cloud_normals->points[i].z = cloud->points[i].z;
    }

    std::cout << "done." << std::endl;

    return cloud_normals;
}


pcl::PointCloud<pcl::PointWithScale>::Ptr computeSIFT(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,
                                                      pcl::IndicesPtr &indices)
{
    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;

    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_sift(new pcl::search::KdTree<pcl::PointNormal>());
    sift.setSearchMethod(tree_sift);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(*result);

    indices->clear();
    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointNormal>());
    tree->setInputCloud(cloud_normals);
    for (const auto &point : result->points) {
        std::vector<int> index;
        std::vector<float> dist;
        pcl::PointNormal p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        tree->nearestKSearch(p, 1, index, dist);
        indices->push_back(index[0]);
    }

    return result;
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,
                                                       pcl::IndicesPtr &indices)
{
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_normals);
    fpfh.setIndices(indices);
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

    auto keypointIndices = boost::make_shared<std::vector<int>>();
    auto keyPoints = computeSIFT(cloud_normals, keypointIndices);

    auto fpfhs = computeFPFH(cloud, cloud_normals, keypointIndices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto index : *keypointIndices)
        cloud_out ->push_back(cloud->points[index]);


	pcl::visualization::PCLVisualizer viewerFPFH("FPFH");
	pcl::visualization::PCLHistogramVisualizer histViewer;
	viewerFPFH.setBackgroundColor (0.0, 0.0, 0.5);
	viewerFPFH.addCoordinateSystem (0.1);
	viewerFPFH.initCameraParameters ();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_out);
	viewerFPFH.addPointCloud<pcl::PointXYZRGB> (cloud_out, rgb, "input_cloud");

	// Create structure containing the parameters for the callback function
	struct callbackArgs histCallbackArgs;
	histCallbackArgs.histViewer = &histViewer;
	histCallbackArgs.fpfhs = fpfhs;

	// Add point picking callback to viewerFPFH (for visualizing feature histograms):
	viewerFPFH.registerPointPickingCallback(pp_callback, static_cast<void*>(&histCallbackArgs));

    int normalsVisualizationStep = 100; // to visualize a normal every normalsVisualizationStep
    float normalsScale = 0.02;			// normals dimension
    pcl::visualization::PCLVisualizer viewerNormals("Normals");
    viewerNormals.setBackgroundColor (0.0, 0.0, 0.5);
    viewerNormals.addCoordinateSystem (0.1);
    viewerNormals.initCameraParameters ();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud);
    viewerNormals.addPointCloud<pcl::PointXYZRGB> (cloud, rgb2, "input_cloud");
    viewerNormals.addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud, cloud_normals, normalsVisualizationStep, normalsScale, "normals");

    // Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	while (!viewerFPFH.wasStopped() && !viewerNormals.wasStopped()) {
        viewerNormals.spin();
		viewerFPFH.spin ();
		histViewer.spin();
	}

	return 0;
}
