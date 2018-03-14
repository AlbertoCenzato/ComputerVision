//
// Created by alberto on 02/03/18.
//

#ifndef CV_HW2_LAB3_VISUALIZER_H
#define CV_HW2_LAB3_VISUALIZER_H

#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

namespace lab3 {


    class Visualizer {

    public:
        Visualizer(const std::string &name) : vis(name) {
            vis.createViewPort(0.0, 0.5, 0.5, 1.0, vpTopLeft);
            vis.createViewPort(0.5, 0.0, 1.0, 0.5, vpBottomRight);
            vis.createViewPort(0.0, 0.0, 0.5, 0.5, vpBottomLeft);
            vis.createViewPort(0.5, 0.5, 1.0, 1.0, vpTopRight);

            colors = {Color(255, 0, 0), // red
                      Color(0, 255, 0), // green
                      Color(0, 0, 255), // blue
                      Color(255, 255, 255)};// white
        }

        /**
         * \brief Display source and target on the second viewport of the visualizer
         */
        template<typename PointT>
        void showCloudsTopRight(std::initializer_list<typename pcl::PointCloud<PointT>::ConstPtr> clouds) {
            showCloudsRight<PointT>(vpTopRight, clouds);
        }

        /**
         * \brief Display source and target on the second viewport of the visualizer
         */
        template<typename PointT>
        void showCloudsBottomRight(std::initializer_list<typename pcl::PointCloud<PointT>::ConstPtr> clouds) {
            showCloudsRight<PointT>(vpBottomRight, clouds);
        }

        /**
         * \brief Display source and target on the first viewport of the visualizer
         */
        template<typename PointT>
        void showCloudsTopLeft(std::initializer_list<typename pcl::PointCloud<PointT>::ConstPtr> clouds) {
            showCloudsLeft<PointT>(vpTopLeft, clouds);
        }

        /**
         * \brief Display source and target on the first viewport of the visualizer
         */
        template<typename PointT>
        void showCloudsBottomLeft(std::initializer_list<typename pcl::PointCloud<PointT>::ConstPtr> clouds) {
            showCloudsLeft<PointT>(vpBottomLeft, clouds);
        }

        template<typename PointT>
        void showIterTop(typename pcl::PointCloud<PointT>::ConstPtr cloudTarget,
                         typename pcl::PointCloud<PointT>::ConstPtr cloudSource) {
            showIter<PointT>(cloudTarget, cloudSource, vpTopRight);
        }

        template<typename PointT>
        void showIterBottom(typename pcl::PointCloud<PointT>::ConstPtr cloudTarget,
                            typename pcl::PointCloud<PointT>::ConstPtr cloudSource) {
            showIter<PointT>(cloudTarget, cloudSource, vpBottomRight);
        }

        void removeAllPointCloudsTopRight() {
            vis.removeAllPointClouds(vpTopRight);
        }

        void removeAllPointCloudsBottomRight() {
            vis.removeAllPointClouds(vpBottomRight);
        }

        void removeAllPointCloudsTopLeft() {
            vis.removeAllPointClouds(vpTopLeft);
        }

        void removeAllPointCloudsBottomLeft() {
            vis.removeAllPointClouds(vpBottomLeft);
        }

    private:

        struct Color {
            uint8_t r;
            uint8_t g;
            uint8_t b;

            Color() : r(0), g(0), b(0) {}

            Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
        };

        pcl::visualization::PCLVisualizer vis;
        int vpTopLeft, vpTopRight, vpBottomLeft, vpBottomRight;
        std::array<Color, 4> colors;


        template<typename PointT>
        void showCloudsLeft(int vp, const std::initializer_list<typename pcl::PointCloud<PointT>::ConstPtr> &clouds) {
            vis.removeAllPointClouds(vp);

            int colorIndex = 0;
            for (const auto &cloud : clouds) {
                std::stringstream cloudID;
                cloudID << "viewport" << vp << "_" << "cloud" << colorIndex;
                auto color = colors[(colorIndex++) % 4];
                PointCloudColorHandlerCustom<PointT> colorHandler(cloud, color.r, color.g, color.b);
                vis.addPointCloud(cloud, colorHandler, cloudID.str(), vp);
            }

            PCL_INFO ("Press q to begin the registration.\n");
            std::cout << std::flush;
            vis.spin();
        }


        template<typename PointT>
        void showCloudsRight(int vp, const std::initializer_list<typename pcl::PointCloud<PointT>::ConstPtr> &clouds) {
            vis.removeAllPointClouds(vp);

            int index = 0;
            for (const auto &cloud : clouds) {
                std::stringstream cloudID;
                cloudID << "viewport" << vp << "_" << "cloud" << index++;

                PointCloudColorHandlerGenericField<PointT> colorHandler(cloud, "curvature");
                if (!colorHandler.isCapable())
                    PCL_WARN ("Cannot create curvature color handler!");

                vis.addPointCloud(cloud, colorHandler, cloudID.str(), vp);
            }

            vis.spinOnce();
        }

        template<typename PointT>
        void showIter(typename pcl::PointCloud<PointT>::ConstPtr cloudTarget,
                      typename pcl::PointCloud<PointT>::ConstPtr cloudSource, int vp) {
            std::string cloudTargetID, cloudSourceID;
            if (vp == vpBottomRight) {
                cloudTargetID = "bottomRight_target";
                cloudSourceID = "bottomRight_source";
            } else {
                cloudTargetID = "topRight_target";
                cloudSourceID = "topRight_source";
            }

            vis.removePointCloud(cloudSourceID);
            vis.removePointCloud(cloudTargetID);

            PointCloudColorHandlerCustom<PointT> cloud_tgt_h(cloudTarget, 0, 255, 0);
            PointCloudColorHandlerCustom<PointT> cloud_src_h(cloudSource, 255, 0, 0);
            vis.addPointCloud(cloudTarget, cloud_tgt_h, cloudTargetID, vp);
            vis.addPointCloud(cloudSource, cloud_src_h, cloudSourceID, vp);

            PCL_INFO ("Press q to continue the registration.\n");
            std::cout << std::flush;

            vis.spin();

            vis.removePointCloud(cloudSourceID);
            vis.removePointCloud(cloudTargetID);
        }
    };

} // namespace lab3

#endif //CV_HW2_LAB3_VISUALIZER_H
