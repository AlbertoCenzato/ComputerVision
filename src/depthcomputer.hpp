#ifndef DEPTHCOMPUTER_H
#define DEPTHCOMPUTER_H

#include "global_includes.hpp"
#include <opencv2/calib3d.hpp>
#include "extra-module/disparity_filter.hpp"

class DepthComputer
{
public:

    static const std::string DISPARITY_WINDOW; // window name for GUI
    static const std::string TRACKBAR_MATCHER; // window name for GUI
    static const std::string TRACKBAR_FILTER;  // window name for GUI
    static const std::string MATCH_AND_FILTER_PARAMS_FILE; // default calibration params xml file name

    /**
     * @brief DepthComputer class constructor, loads stereo rectification params
     *        from the specified xml file and stereo matching and post filtering
     *        params from MATCH_AND_FILTER_PARAMS_FILE
     * @param stereoParamsFile stereo rectification params xml file path
     * @param tuneParams if true enables stereo parameters tuning with user interface
     */
    DepthComputer(const std::string &stereoParamsFile, bool tuneParams = false);

    /**
     * @brief compute computes the depth map and the rectified image from the left camera
     *        using the stereo rectification, matching and post filtering parameters
     *        loaded by the constructor. Optionally stereo matching and post filtering
     *        params can be tuned runtime by the user with a slider.
     * @param left image from left camera
     * @param right image from right camera
     * @param rectLeft output rectified left image
     * @param image3d output depth map
     */
    void compute(const cv::Mat &left, const cv::Mat &right, cv::Mat &rectLeft, cv::Mat &image3d);

    /**
     * @brief loadCalibrationParams function called by the constructor to load
     *        stereo rectification params from the specified xml file
     * @param fileName stereo rectification params xml file path
     * @return
     */
    bool loadCalibrationParams(const std::string &fileName);

    /**
     * @brief saveMatchAndFilterParams saves stereo matching and post processing filter
     *        params in the specified xml file
     * @param fileName stereo matching and post processing filter params xml file
     * @return
     */
    bool saveMatchAndFilterParams(const std::string &fileName);

    /**
     * @brief loadMatchAndFilterParams loads stereo matching and post processing filter
     *        params from the specified xml file
     * @param fileName stereo matching and post processing filter params xml file
     * @return
     */
    bool loadMatchAndFilterParams(const std::string &fileName);

private:

    bool tuneParams;
    cv::Ptr<cv::StereoSGBM> matcher;                        // disparity matcher
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;   // post processing filter

    //----- stereo rectification params -----
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;

    //----- stereo matching params default values -----
    int minDisparities = 0, numDisparities = 144, blockSize = 9, p1 = 100, p2 = 3000;
    int disp12MaxDiff = 10, preFilterCap = 4, uniquenessRatio = 1, speckleWindowSize = 150, speckleRange = 2;

    //----- post processing filter default values -----
    int postFilter = 0, lambda = 8000, sigmaColor = 15, depthDiscRadius, lrcThresh;

    /**
     * @brief tuneMatcher enables the user to perform an interactive tuning
     *        of stereo matching params and save them in a file
     * @param rectLeft rectified image from left camera
     * @param rectRight rectified image from right camera
     * @param disparity output disparity map
     */
    void tuneMatcher(const cv::Mat &rectLeft, const cv::Mat &rectRight, cv::Mat &disparity);

    /**
     * @brief tunePostFilter enables the user to perform an interactive tuning
     *        of post processing filter params and save them in a file
     * @param disparity disparity map computed from the left camera image
     * @param disparity_right disparity map computed from the right camera image
     * @param rectLeft rectified image from left camera
     * @param disparity_filtered output filtered disparity map
     */
    void tunePostFilter(const cv::Mat &disparity, const cv::Mat &disparity_right,
                        const cv::Mat &rectLeft, cv::Mat &disparity_filtered);

    // callbacks for GUI trackbars used by tuneMatcher() and tunePostFilter()
    static void on_trackbar_matcher(int pos, void *o);
    static void on_trackbar_filter (int pos, void *o);
};

#endif // DEPTHCOMPUTER_H
