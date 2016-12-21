#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "global_includes.hpp"

//class DepthComputer;

class StereoCalibrator
{
    friend class DepthComputer;

public:

    StereoCalibrator(cv::Size &chessboardSize, float chessSize);

    double compute(std::vector<cv::Mat> &left, std::vector<cv::Mat> &right, bool skipCheck = false);
    void getChessboardPattern(const cv::Size &chessboardSize, float chessSize, std::vector<cv::Point3f> &pattern);
    bool checkVectors(std::vector<cv::Mat> &left, std::vector<cv::Mat> &right);

    bool saveCalibration(const std::string &fileName);
    bool loadCalibration (const std::string &fileName);

    StereoCalibrator* setChessboardParams(cv::Size &chessboardSize, float chessSize);

private:

    cv::Size chessboardSize;
    float chessSize;
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
};

#endif // STEREOCALIBRATOR_H
