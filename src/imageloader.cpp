#include "imageloader.hpp"
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

ImageLoader::ImageLoader() : VideoCapture() {}
ImageLoader::ImageLoader(const string& path) : VideoCapture(path, cv::CAP_IMAGES) {}

bool ImageLoader::getImage(Mat &image) {
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

bool ImageLoader::getImage(vector<Mat> &imageSequence) {

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

bool ImageLoader::getImage(const std::string &path, cv::Mat &image) {
    open(path, cv::CAP_IMAGES);
    return getImage(image);
}

bool ImageLoader::getImage(const std::string &path, std::vector<cv::Mat> &imageSequence) {
    open(path, cv::CAP_IMAGES);
    return getImage(imageSequence);
}
