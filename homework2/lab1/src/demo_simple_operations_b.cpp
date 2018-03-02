/*
 * demo_simple_operations.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer
#include <pcl/filters/voxel_grid.h>


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr downsampleCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float lx, float ly, float lz) {
    typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(lx, ly, lz);
    vg.filter(*cloud_out);

    return cloud_out;
}


int main (int argc, char** argv) {
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Load point cloud from .pcd file:
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset_lab1/table_scene_lms400.pcd",*cloud) == -1)	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}

	std::cout << "Loaded " << cloud->width * cloud->height
			  << " data points from the pcd file. " << std::endl;
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
			  << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudSubsets(4);
    for (auto &cloudSubset : cloudSubsets)
        cloudSubset = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto &point : cloud->points) {
        if (point.x < 0 && point.y < 0) {
            pcl::PointXYZRGB p(255,0,0);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            cloudSubsets[0]->push_back(p);
        }
        else if (point.x < 0 && point.y > 0) {
            pcl::PointXYZRGB p(0,255,0);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            cloudSubsets[1]->push_back(p);
        }
        else if (point.x > 0 && point.y < 0) {
            pcl::PointXYZRGB p(0,0,255);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            cloudSubsets[2]->push_back(p);
        }
        else {
            pcl::PointXYZRGB p(255,255,255);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            cloudSubsets[3]->push_back(p);
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> filteredClouds(4);

  	filteredClouds[0] = downsampleCloud<pcl::PointXYZRGB>(cloudSubsets[0], 0.005f, 0.005f, 0.005f);
    filteredClouds[1] = downsampleCloud<pcl::PointXYZRGB>(cloudSubsets[1], 0.01f, 0.01f, 0.01f);
    filteredClouds[2] = downsampleCloud<pcl::PointXYZRGB>(cloudSubsets[2], 0.05f, 0.05f, 0.05f);
    filteredClouds[3] = downsampleCloud<pcl::PointXYZRGB>(cloudSubsets[3], 0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto &filteredCloud : filteredClouds) {
        for (const auto &point : filteredCloud->points) {
            cloud_out->push_back(point);
        }
    }

	// Visualization
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


