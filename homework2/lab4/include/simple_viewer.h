//
// Created by alberto on 21/03/18.
//

#ifndef CV_HW2_LAB4_SIMPLE_VIEWER_H
#define CV_HW2_LAB4_SIMPLE_VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>

namespace lab4 {

    class SimpleViewer {

        pcl::visualization::PCLVisualizer viewer;

    public:

        SimpleViewer(std::string name);

        void visualize(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

    };

} // namespace lab4
#endif //CV_HW2_LAB4_SIMPLE_VIEWER_H
