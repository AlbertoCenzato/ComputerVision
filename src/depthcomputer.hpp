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

    static const std::string DISPARITY_WINDOW;
    static const std::string TRACKBAR_WINDOW;
    static const std::string STEREO_PARAMS_FILE;

    DepthComputer(const std::string &fileName, bool tuneParams = false);
    DepthComputer(const StereoCalibrator &calibrator, bool tuneParams = false);

    void compute(const cv::Mat &left, const cv::Mat &right, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool readCalibration(const std::string &fileName);
    bool readCalibration(const StereoCalibrator &calibrator);

    bool saveParams(const std::string &fileName);
    bool loadParams(const std::string &fileName);

    static void matToPointCloud(const cv::Mat &image, const cv::Mat &depthMap, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
    bool tuneParams;

    cv::Ptr<cv::StereoSGBM> matcher;
    int minDisparities = 0, numDisparities = 144, blockSize = 9;
    int p1 = 100, p2 = 3000, disp12MaxDiff = 10, preFilterCap = 4, uniquenessRatio = 1, speckleWindowSize = 150, speckleRange = 2;

    static void on_trackbar(int pos, void *obj);
};

#endif // DEPTHCOMPUTER_H
