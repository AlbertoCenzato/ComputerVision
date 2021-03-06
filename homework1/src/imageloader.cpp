#include "imageloader.hpp"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

ImageLoader::ImageLoader() : VideoCapture() {}
ImageLoader::ImageLoader(const string& path) : VideoCapture(path) {}

bool ImageLoader::get(Mat &image) {
    if(!isOpened())
        return false;

    *this >> image;
    if(image.empty())
        return false;

    if(downscalingRatio != 1) {
        int width = image.cols;
        int height = image.rows;
        resize(image, image, Size(width*downscalingRatio,height*downscalingRatio));
    }

    return true;
}

bool ImageLoader::get(vector<Mat> &imageSequence) {

    if(!isOpened())
        return false;

    imageSequence.clear();

    while(true) {
        Mat img;
        *this >> img;
        if(img.empty())
            break;

        if(downscalingRatio != 1) {
            int width = img.cols;
            int height = img.rows;
            resize(img, img, Size(width*downscalingRatio,height*downscalingRatio));
        }

        imageSequence.push_back(img);
    }

    return true;
}

bool ImageLoader::get(const std::string &path, cv::Mat &image) {
    open(path);
    return get(image);
}

bool ImageLoader::get(const std::string &path, std::vector<cv::Mat> &imageSequence) {
    open(path);
    return get(imageSequence);
}
