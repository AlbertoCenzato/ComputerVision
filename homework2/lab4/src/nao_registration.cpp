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
#include <pcl/segmentation/extract_clusters.h>

#include "plane_segmenter.h"
#include "simple_viewer.h"
#include "multi_cloud_register.h"

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

CloudT::Ptr removeGroundPlane(CloudT::ConstPtr cloud);

CloudT::Ptr extractForegroundObject(CloudT::Ptr cloud);

CloudT::Ptr registerClouds(std::vector<CloudT::ConstPtr> &clouds);


int main (int argc, char** argv) {

    lab4::SimpleViewer viewer("Nao");
	std::vector<CloudT::ConstPtr> naoClouds;


	for (unsigned int i = 1; i <= 6; i++) {
		CloudT::Ptr cloud (new CloudT);

		// Load point cloud from .pcd file:
		std::stringstream ss;
		ss << "../dataset_lab4/nao/" << i << ".pcd";
		if (pcl::io::loadPCDFile<CloudT::PointType> (ss.str(), *cloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read the pcd file \n");
			return (-1);
		}

		viewer.visualize(cloud);

		auto cloudNoGround = removeGroundPlane(cloud);

        viewer.visualize(cloudNoGround);

		auto naoCloud = extractForegroundObject(cloudNoGround);

		viewer.visualize(naoCloud);

		naoClouds.push_back(naoCloud);
	}

	auto finalCloud = registerClouds(naoClouds);

	viewer.visualize(finalCloud);

	return 0;
}


CloudT::Ptr removeGroundPlane(CloudT::ConstPtr cloud)
{
	lab4::PlaneSegmenter<PointT> seg;
	seg.setAxisAndTolerance({0.f,1.f,0.f});
    seg.setDistanceThreshold(0.05);
	seg.estimatePlane(cloud);
	return seg.getSegmentedCloud();
}

CloudT::Ptr extractForegroundObject(CloudT::Ptr cloud)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    auto minDistance = std::numeric_limits<float>::max();
    int bestCluster = -1;

    std::vector<CloudT::Ptr> clusters(cluster_indices.size());
    for (auto i = 0; i < cluster_indices.size(); ++i) {
        const auto &indices = cluster_indices[i].indices;
        clusters[i] = CloudT::Ptr(new CloudT);
        auto &cluster = clusters[i];
        float meanDistance = 0;
        for (auto index : indices) {
            const auto &point = cloud->points[index];
            meanDistance += point.z;
            cluster->points.push_back(point);
        }
        meanDistance /= cluster->points.size();
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        if (meanDistance < minDistance) {
            minDistance = meanDistance;
            bestCluster = i;
        }
    }

    if (bestCluster == -1)
        return CloudT::Ptr(new CloudT);

    return clusters[bestCluster];
}


CloudT::Ptr registerClouds(std::vector<CloudT::ConstPtr> &clouds)
{
    lab4::MultiCloudRegister<PointT> reg;
    return reg.registerClouds(clouds);
}