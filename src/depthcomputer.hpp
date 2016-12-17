#ifndef DEPTHCOMPUTER_H
#define DEPTHCOMPUTER_H

#include "global_includes.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud

class StereoCalibrator;

class DepthComputer
{
public:
    DepthComputer(const std::string &fileName);
    DepthComputer(const StereoCalibrator& calibrator);

    void compute(cv::Mat left, cv::Mat right, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool readCalibration(const std::string& fileName);
    bool readCalibration(const StereoCalibrator& calibrator);

    void matToPointCloud(const cv::Mat& mat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
};

#endif // DEPTHCOMPUTER_H
