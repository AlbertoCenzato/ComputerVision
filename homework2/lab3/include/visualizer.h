//
// Created by alberto on 02/03/18.
//

#ifndef CV_HW2_LAB3_VISUALIZER_H
#define CV_HW2_LAB3_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


class Visualizer {

public:
    Visualizer(int &argc, char** argv, const std::string &name) : vis(argc, argv, name) {
        vis.createViewPort(0.0, 0.0, 0.5, 5.0, vpTopLeft);
        vis.createViewPort(0.5, 0.0, 1.0, 5.0, vpBottomRight);
        vis.createViewPort(0.0, 0.5, 0.5, 1.0, vpBottomLeft);
        vis.createViewPort(0.5, 0.5, 1.0, 1.0, vpTopRight);
    }

    /**
     * \brief Display source and target on the second viewport of the visualizer
     */
    template<typename PointT>
    void showCloudsTopRight(const typename pcl::PointCloud<PointT>::Ptr cloud_target,
                            const typename pcl::PointCloud<PointT>::Ptr cloud_source)
    {
        showCloudsRight<PointT>(cloud_target, cloud_source, vpTopRight);
    }

    /**
     * \brief Display source and target on the second viewport of the visualizer
     */
    template<typename PointT>
    void showCloudsBottomRight(const typename pcl::PointCloud<PointT>::Ptr cloud_target,
                               const typename pcl::PointCloud<PointT>::Ptr cloud_source)
    {
        showCloudsRight<PointT>(cloud_target, cloud_source, vpBottomRight);
    }

    /**
     * \brief Display source and target on the first viewport of the visualizer
     */
    template<typename PointT>
    void showCloudsTopLeft(const typename pcl::PointCloud<PointT>::Ptr cloud_target,
                           const typename pcl::PointCloud<PointT>::Ptr cloud_source)
    {
        showCloudsLeft<PointT>(cloud_target, cloud_source, vpTopLeft);
    }

    /**
     * \brief Display source and target on the first viewport of the visualizer
     */
    template<typename PointT>
    void showCloudsBottomLeft(const typename pcl::PointCloud<PointT>::Ptr cloud_target,
                              const typename pcl::PointCloud<PointT>::Ptr cloud_source)
    {
        showCloudsLeft<PointT>(cloud_target, cloud_source, vpBottomLeft);
    }

    template<typename PointT>
    void showIterTop(const typename pcl::PointCloud<PointT>::Ptr cloudTarget,
                     const typename pcl::PointCloud<PointT>::Ptr cloudSource)
    {
        showIter<PointT>(cloudTarget, cloudSource, vpTopRight);
    }

    template<typename PointT>
    void showIterBottom(const typename pcl::PointCloud<PointT>::Ptr cloudTarget,
                        const typename pcl::PointCloud<PointT>::Ptr cloudSource)
    {
        showIter<PointT>(cloudTarget, cloudSource, vpBottomRight);
    }

private:

    pcl::visualization::PCLVisualizer vis;
    int vpTopLeft, vpTopRight, vpBottomLeft, vpBottomRight;

    template<typename PointT>
    void showCloudsLeft(const typename pcl::PointCloud<PointT>::Ptr cloud_target,
                        const typename pcl::PointCloud<PointT>::Ptr cloud_source, int vp)
    {
        std::string cloudTarget, cloudSource;
        if (vp == vpBottomLeft) {
            cloudTarget = "bottomLeft_target";
            cloudSource = "bottomLeft_source";
        }
        else {
            cloudTarget = "topLeft_target";
            cloudSource = "topLeft_source";
        }

        vis.removePointCloud(cloudTarget);
        vis.removePointCloud(cloudSource);

        PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 255,   0, 0);
        PointCloudColorHandlerCustom<PointT> src_h (cloud_source,   0, 255, 0);
        vis.addPointCloud(cloud_target, tgt_h, cloudTarget, vp);
        vis.addPointCloud(cloud_source, src_h, cloudSource, vp);

        PCL_INFO ("Press q to begin the registration.\n");
        std::cout << std::flush;
        vis.spin();
    }


    template<typename PointT>
    void showCloudsRight(const typename pcl::PointCloud<PointT>::Ptr cloud_target,
                         const typename pcl::PointCloud<PointT>::Ptr cloud_source, int vp)
    {
        std::string cloudTarget, cloudSource;
        if (vp == vpBottomRight) {
            cloudTarget = "bottomRight_target";
            cloudSource = "bottomRight_source";
        }
        else {
            cloudTarget = "topRight_target";
            cloudSource = "topRight_source";
        }

        vis.removePointCloud(cloudTarget);
        vis.removePointCloud(cloudSource);

        PointCloudColorHandlerGenericField<PointT> tgt_color_handler (cloud_target, "curvature");
        if (!tgt_color_handler.isCapable ())
            PCL_WARN ("Cannot create curvature color handler!");

        PointCloudColorHandlerGenericField<PointT> src_color_handler (cloud_source, "curvature");
        if (!src_color_handler.isCapable ())
            PCL_WARN ("Cannot create curvature color handler!");

        vis.addPointCloud (cloud_target, tgt_color_handler, cloudTarget, vp);
        vis.addPointCloud (cloud_source, src_color_handler, cloudSource, vp);

        vis.spinOnce();
    }

    template<typename PointT>
    void showIter(const typename pcl::PointCloud<PointT>::Ptr cloudTarget,
                  const typename pcl::PointCloud<PointT>::Ptr cloudSource, int vp)
    {
        std::string cloudTargetID, cloudSourceID;
        if (vp == vpBottomRight) {
            cloudTargetID = "bottomRight_target";
            cloudSourceID = "bottomRight_source";
        }
        else {
            cloudTargetID = "topRight_target";
            cloudSourceID = "topRight_source";
        }

        vis.removePointCloud(cloudSourceID);
        vis.removePointCloud(cloudTargetID);

        PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloudTarget, 0, 255, 0);
        PointCloudColorHandlerCustom<PointT> cloud_src_h (cloudSource, 255, 0, 0);
        vis.addPointCloud (cloudTarget, cloud_tgt_h, cloudTargetID, vp);
        vis.addPointCloud (cloudSource, cloud_src_h, cloudSourceID, vp);

        PCL_INFO ("Press q to continue the registration.\n");
        std::cout << std::flush;

        vis.spin();

        vis.removePointCloud(cloudSourceID);
        vis.removePointCloud(cloudTargetID);
    }
};

#endif //CV_HW2_LAB3_VISUALIZER_H
