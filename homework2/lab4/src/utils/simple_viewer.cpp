//
// Created by alberto on 21/03/18.
//

#include "simple_viewer.h"

namespace lab4 {

    SimpleViewer::SimpleViewer(std::string name) : viewer(name) {
        viewer.setBackgroundColor (0, 0, 0);
        viewer.addCoordinateSystem (0.1);
    }

    void SimpleViewer::visualize(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
        viewer.removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
        viewer.spin();
    }

} // namespace lab4