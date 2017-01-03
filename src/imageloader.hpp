#ifndef IMAGELOADER_H
#define IMAGELOADER_H

#include "global_includes.hpp"
#include <opencv2/opencv.hpp>

class ImageLoader : public cv::VideoCapture
{
public:

    float downscalingRatio = 1;

    ImageLoader();
    ImageLoader(const std::string &path);

    bool get(cv::Mat& image);
    bool get(std::vector<cv::Mat> &imageSequence);
    bool get(const std::string &path, cv::Mat &image);
    bool get(const std::string &path, std::vector<cv::Mat> &imageSequence);
};

#endif // IMAGELOADER_H
