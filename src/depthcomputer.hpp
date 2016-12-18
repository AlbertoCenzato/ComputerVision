#ifndef DEPTHCOMPUTER_H
#define DEPTHCOMPUTER_H

#include "global_includes.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud

class StereoCalibrator;

class DepthComputer
{
public:
    DepthComputer(const std::string &fileName, bool tuneParams = false);
    DepthComputer(const StereoCalibrator& calibrator, bool tuneParams = false);

    void compute(cv::Mat left, cv::Mat right, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool readCalibration(const std::string& fileName);
    bool readCalibration(const StereoCalibrator& calibrator);

    static void matToPointCloud(const cv::Mat& image, const cv::Mat &depthMap, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
    bool tuneParams;
};

#endif // DEPTHCOMPUTER_H
