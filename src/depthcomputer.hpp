#ifndef DEPTHCOMPUTER_H
#define DEPTHCOMPUTER_H

#include "global_includes.hpp"
#include <opencv2/calib3d.hpp>
#include "extra-module/disparity_filter.hpp"

class StereoCalibrator;

class DepthComputer
{
public:

    static const std::string DISPARITY_WINDOW;
    static const std::string TRACKBAR_MATCHER;
    static const std::string TRACKBAR_FILTER;
    static const std::string STEREO_PARAMS_FILE;

    DepthComputer(const std::string &fileName, bool tuneParams = false);
    DepthComputer(const StereoCalibrator &calibrator, bool tuneParams = false);

    void compute(const cv::Mat &left, const cv::Mat &right, cv::Mat &rectLeft, cv::Mat &image3d);
    bool readCalibration(const std::string &fileName);
    bool readCalibration(const StereoCalibrator &calibrator);

    bool saveParams(const std::string &fileName);
    bool loadParams(const std::string &fileName);

private:
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
    bool tuneParams;

    cv::Ptr<cv::StereoSGBM> matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    int minDisparities = 0, numDisparities = 144, blockSize = 9;
    int p1 = 100, p2 = 3000, disp12MaxDiff = 10, preFilterCap = 4, uniquenessRatio = 1, speckleWindowSize = 150, speckleRange = 2;
    int postFilter = 0, lambda = 8000, sigmaColor = 15, depthDiscRadius, lrcThresh;

    void constructor(bool tuneParams);
    void tuneMatcher(const cv::Mat &rectLeft, const cv::Mat &rectRight, cv::Mat &disparity);
    void tunePostFilter(const cv::Mat &disparity, const cv::Mat &rectLeft, cv::Mat &disparity_filtered, const cv::Mat &disparity_right);

    static void on_trackbar_matcher(int pos, void *o);
    static void on_trackbar_filter (int pos, void *o);
};

#endif // DEPTHCOMPUTER_H
