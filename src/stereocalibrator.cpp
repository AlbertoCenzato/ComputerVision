#include "stereocalibrator.h"

#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

StereoCalibrator::StereoCalibrator()
{

}

double StereoCalibrator::calibrate(Size& chessboardSize, float chessSize, vector<Mat>& left, vector<Mat>& right) {
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

    cout << "Looking for left corners..." << endl;
    vector<vector<Point2f> > leftCorners(0);
    for(int i = 0; i < size; ++i) {
        vector<Point2f> corners(patternSize.width*patternSize.height);        //TODO: if findChessboradCorners returns false
        findChessboardCorners(left[i],patternSize, corners);                //      should do something like ramoving the image
        leftCorners.push_back(corners);                                     //      from the dataset
    }

    cout << "\nLooking for right corners..." << endl;
    vector<vector<Point2f> > rightCorners(0);
    for(int i = 0; i < size; ++i) {
        vector<Point2f> corners(patternSize.width*patternSize.height);
        findChessboardCorners(right[i],patternSize, corners);
        rightCorners.push_back(corners);
    }

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

