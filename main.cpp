#include <iostream>
#include <float.h>
#include <boost/lexical_cast.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
using namespace pcl;

bool loadImages(string& path, vector<Mat> &images);
void showImages(vector<Mat>& images);
void calibrateStereo(vector<Mat>& left, vector<Mat>& right);

int main(int argc, char *argv[])
{
    cout << "Hello World!" << endl;

    string pathLeft  = "/home/alberto/Downloads/dataset_kinect_stereo(1)/left/image_%04d.png";
    string pathRight = "/home/alberto/Downloads/dataset_kinect_stereo(1)/right/image_%04d.png";
    vector<Mat> left(20), right(20);

    if(loadImages(pathLeft,left) && loadImages(pathRight, right))
        cout << "Images loaded successfully!" << endl;
    else {
        cout << "Error loading images!" << endl;
        return 0;
    }

    showImages(left);

    return 0;
}

bool loadImages(string& path, vector<Mat> &images) {
    VideoCapture capture(path);
    if(!capture.isOpened())
        return false;

    images.clear();

    while(true) {
        Mat img;
        capture >> img;
        if(img.empty())
            break;
        else
            images.push_back(img);
    }

    return true;
}

void showImages(vector<Mat>& images) {
    string name = "Image ";
    for(int i = 0; i < images.size(); ++i) {
        string window_name = name + boost::lexical_cast<string>(i+1);
        namedWindow(window_name, WINDOW_NORMAL);
        imshow(window_name, images[i]);
    }

    waitKey(0);
}

double calibrateStereo(Size& chessboardSize, float chessSize, vector<Mat>& left, vector<Mat>& right) {
    int leftSize  = left .size();
    int rightSize = right.size();
    if(leftSize == 0; || rightSize == 0) {
        cerr << "Cannot calibrate cameras with an empty set of images!" << endl;
        return DBL_MAX;
    }
    if(leftSize != rightSize) {
        cout << "Warning! The numer of images from left camera\n" <<
                "does not match the number of images from right camera.\n" <<
                "The exceeding images will be discarded." << endl;
    }

    int size = leftSize < rightSize ? leftSize : rightSize;

    // build pattern vector...
    Mat pattern = Mat(chessboardSize.x-1,chessboardSize.y-1,CV_32FC3);
    for(int i = 0; i < chessboardSize.y - 1; ++i) {
        for(int j = 0; j < chessboardSize.x - 1; ++j) {
            pattern.at(i,j) = Vec3f(chessSize*i,chessSize*j,0);
        }
    }

    vector<Mat> objectPoints(size);
    for(int i = 0; i < size; ++i) {
        objectPoints[i] = pattern;
    }

    // find chessboard corners...
    cv::findChessboardCorners()

    // stereoCalibrate
    stereoCalibrate(InputArrayOfArrays objectPoints, left, right, InputOutputArray cameraMatrix1, InputOutputArray distCoeffs1, InputOutputArray cameraMatrix2, InputOutputArray distCoeffs2, Size imageSize, OutputArray R, OutputArray T, OutputArray E, OutputArray F, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6), int flags=CALIB_FIX_INTRINSIC )
}
