#include <float.h>
#include <boost/lexical_cast.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils.hpp"

using namespace std;
using namespace cv;
using namespace pcl;

bool loadImages(string& path, vector<Mat> &images);
void showImages(vector<Mat>& images);
double calibrateStereo(Size& chessboardSize, float chessSize, vector<Mat>& left, vector<Mat>& right);

int main(int argc, char *argv[])
{
    cout << "Loading images..." << endl;

    string pathLeft  = "/home/alberto/Downloads/dataset_kinect_stereo(1)/left/image_%04d.png";
    string pathRight = "/home/alberto/Downloads/dataset_kinect_stereo(1)/right/image_%04d.png";
    vector<Mat> left(20), right(20);

    if(loadImages(pathLeft,left) && loadImages(pathRight, right))
        cout << "Images loaded successfully!" << endl;
    else {
        cout << "Error loading images!" << endl;
        return 0;
    }

    Size size(7,6);
    calibrateStereo(size, 12, left, right);

    return 0;
}

bool loadImages(string& path, vector<Mat> &images) {
    VideoCapture capture(path);
    if(!capture.isOpened())
        return false;

    images.clear();

    while(true) {
        Mat img;
        if(!capture.grab())
            break;
        capture.retrieve(img);
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
    if(leftSize == 0 || rightSize == 0) {
        cerr << "Cannot calibrate cameras with an empty set of images!" << endl;
        return DBL_MAX;
    }
    if(leftSize != rightSize) {
        cout << "Warning! The numer of images from left camera\n" <<
                "does not match the number of images from right camera.\n" <<
                "The exceeding images will be discarded." << endl;
    }

    Size patternSize(chessboardSize.width-1,chessboardSize.height-1);
    int size = leftSize < rightSize ? leftSize : rightSize;

    //TODO (?): check if images are of different sizes?

    // build pattern vector...
    cout << "Building pattern..." << endl;
    vector<Point3f> pattern(patternSize.height*patternSize.width);
    for(int i = 0; i < patternSize.height; ++i) {
        for(int j = 0; j < patternSize.width; ++j) {
            pattern[j] = Point3f(chessSize*i,chessSize*j,0);
        }
        cout << "Row " << i << ":\n" << endl;
        print(pattern);
    }

    vector<vector<Point3f> > objectPoints(size);
    for(int i = 0; i < size; ++i) {
        objectPoints[i] = pattern;
    }

    // print(objectPoints);
    //cout << "objectPoints len: " << objectPoints.size() << endl;

    // find chessboard corners...

    cout << "Looking for left corners..." << endl;
    vector<vector<Point2f> > leftCorners(0);
    for(int i = 0; i < size; ++i) {
        vector<Point2f> corners(patternSize.width*patternSize.height);        //TODO: if findChessboradCorners returns false
        findChessboardCorners(left[i],patternSize, corners);                //      should do something like ramoving the image
        leftCorners.push_back(corners);                                     //      from the dataset
    }

    //print(leftCorners);
    //cout << "leftCorners len: " << leftCorners.size() << endl;

    cout << "\nLooking for right corners..." << endl;
    vector<vector<Point2f> > rightCorners(0);
    for(int i = 0; i < size; ++i) {
        vector<Point2f> corners(patternSize.width*patternSize.height);
        findChessboardCorners(right[i],patternSize, corners);
        rightCorners.push_back(corners);
    }

    //print(rightCorners);
    //cout << "rightCorners len: " << rightCorners.size() << endl;

    // calibrate left camera
    /*
    cout << "\nCalibrating left camera..." << endl;
    Mat intrinsicLeft, distLeft, rvecs, tvecs;
    double error = calibrateCamera(objectPoints, leftCorners, left[0].size(), intrinsicLeft, distLeft, rvecs, tvecs);
*/

    // stereoCalibrate
    cout << "Calibrating stereo cameras..." << endl;
    Mat cameraMatrix1 = Mat::eye(3, 3, CV_64F);
    Mat cameraMatrix2 = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs1, distCoeffs2;
    Mat R, T, E, F;
    double error = stereoCalibrate(objectPoints, leftCorners, rightCorners,
                                   cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, left[0].size(),
                                   R, T, E, F,  CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

    return error;
}

