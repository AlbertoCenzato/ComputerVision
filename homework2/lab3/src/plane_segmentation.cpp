#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

//convenient typedefs
typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), plane_cloud (new pcl::PointCloud<PointT>),
		  remaining_cloud (new pcl::PointCloud<PointT>), cloud(new pcl::PointCloud<PointT>);

  // Read the cloud data
  if (pcl::io::loadPCDFile<PointT> ("../dataset/table_scene_lms400.pcd", *cloud) == -1) //* load the file
  {
	  PCL_ERROR ("Couldn't read the pcd file \n");
	  return (-1);
  }

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // Extract planes while 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);
    std::cerr << "PointCloud representing the planar component: " << plane_cloud->width * plane_cloud->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);		// to make filter method to return "outliers" instead of "inliers"
    extract.filter (*remaining_cloud);
    cloud_filtered.swap (remaining_cloud);
    i++;

    // Visualization:
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    // Draw output point cloud:
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (0.1);
    viewer.addText ("Cloud 1", 10, 10);
    viewer.addPointCloud<PointT> (remaining_cloud, "cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (plane_cloud, 255, 0, 0);
    viewer.addPointCloud<PointT> (plane_cloud, red, "plane");

    // Loop for visualization (so that the visualizers are continuously updated):
    std::cout << "Visualization...press Q to continue. "<< std::endl;
    while (!viewer.wasStopped ())
    {
    	viewer.spin ();
    }
  }

  return (0);
}
