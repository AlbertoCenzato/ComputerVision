#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include <vector>
#include <opencv2/core.hpp>

class StereoCalibrator
{
public:
    StereoCalibrator();
    double calibrateStereo(cv::Size& chessboardSize, float chessSize, std::vector<cv::Mat>& left, std::vector<cv::Mat>& right);
};

#endif // STEREOCALIBRATOR_H
