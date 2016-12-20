#ifndef DEPTHCOMPUTER_H
#define DEPTHCOMPUTER_H

#include "global_includes.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <opencv2/calib3d.hpp>

class StereoCalibrator;

class DepthComputer
{
public:

    const std::string DISPARITY_WINDOW = "Disparity";
    const std::string IMAGE3D_WINDOW = "3D image";

    cv::Ptr<cv::StereoSGBM> matcher;
    int minDisparities = 0, numDisparities = 144, blockSize = 9;
    int p1 = 100, p2 = 3000, disp12MaxDiff, preFilterCap = 4, uniquenessRatio, speckleWindowSize, speckleRange;

    DepthComputer(const std::string &fileName, bool tuneParams = false);
    DepthComputer(const StereoCalibrator &calibrator, bool tuneParams = false);

    void compute(const cv::Mat &left, const cv::Mat &right, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool readCalibration(const std::string &fileName);
    bool readCalibration(const StereoCalibrator &calibrator);

    static void matToPointCloud(const cv::Mat &image, const cv::Mat &depthMap, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
    bool tuneParams;

    void on_trackbar(int, void*);
};

#endif // DEPTHCOMPUTER_H
