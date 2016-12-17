#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "global_includes.hpp"

class StereoCalibrator
{
public:

    cv::Size chessboardSize;
    float chessSize;
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;

    StereoCalibrator(cv::Size& chessboardSize, float chessSize);
    double calibrate(std::vector<cv::Mat>& left, std::vector<cv::Mat>& right, bool skipCheck = false);
    void computeChessboardPattern(const cv::Size& chessboardSize, float chessSize, std::vector<cv::Point3f>& pattern);
    bool checkVectors(std::vector<cv::Mat>& left, std::vector<cv::Mat>& right);

    bool writeCalibration(const std::string& fileName);
    bool readCalibration (const std::string& fileName);

    StereoCalibrator* setChessboardParams(cv::Size& chessboardSize, float chessSize);

};

#endif // STEREOCALIBRATOR_H
