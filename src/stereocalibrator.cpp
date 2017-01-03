#include "stereocalibrator.hpp"
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;


// ----- Constructor -----

StereoCalibrator::StereoCalibrator(cv::Size &chessboardSize, float chessSize) {
    setChessboardParams(chessboardSize, chessSize);
}

// ----- Public member functions -----
void StereoCalibrator::setChessboardParams(cv::Size &chessboardSize, float chessSize) {
    this->chessboardSize.width  = chessboardSize.width;
    this->chessboardSize.height = chessboardSize.height;
    this->chessSize = chessSize;
}

double StereoCalibrator::compute(vector<Mat> &left, vector<Mat> &right, bool skipCheck) {

    //----- check if images are ok (optional) -----
    if(!skipCheck && !checkVectors(left,right))
        return DBL_MAX;

    //----- build chessboard pattern -----
    Size imgSize(left[0].cols,left[0].rows);
    Size patternSize(chessboardSize.width-1,chessboardSize.height-1);
    int lenght = left.size();

    vector<Point3f> pattern(0);
    getChessboardPattern(patternSize, chessSize, pattern);

    vector<vector<Point3f> > objectPoints(lenght);
    for(int i = 0; i < lenght; ++i) {
        objectPoints[i] = pattern;
    }

    // ----- extracting corner positions from images -----
    cout << "Looking for chessboard corners..." << endl;
    vector<vector<Point2f> > leftCorners (0);
    vector<vector<Point2f> > rightCorners(0);
    for(int i = 0; i < lenght; ++i) {
        vector<Point2f> cornersLeft (patternSize.width*patternSize.height);
        vector<Point2f> cornersRight(patternSize.width*patternSize.height);
        bool leftFound  = findChessboardCorners(left[i],  patternSize, cornersLeft,  CALIB_CB_ADAPTIVE_THRESH);
        bool rightFound = findChessboardCorners(right[i], patternSize, cornersRight, CALIB_CB_ADAPTIVE_THRESH);
        if (leftFound && rightFound) {
            leftCorners.push_back(cornersLeft);
            rightCorners.push_back(cornersRight);
        }
    }

    //----- stereo calibration -----
    cout << "Computing stereo calibration..." << endl;

    Mat R, T, E, F;
    cameraMatrix1 = initCameraMatrix2D(objectPoints,leftCorners, imgSize);
    cameraMatrix2 = initCameraMatrix2D(objectPoints,rightCorners,imgSize);
    double error = stereoCalibrate(objectPoints, leftCorners, rightCorners, cameraMatrix1,
                                   distCoeffs1, cameraMatrix2, distCoeffs2, imgSize,
                                   R, T, E, F, CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6));

    cout << "Computing stereo rectification..." << endl;
    stereoRectify(cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2, imgSize, R, T, R1, R2, P1, P2, Q);

    return error;
}

void StereoCalibrator::getChessboardPattern(const Size &chessboardSize, float chessSize, vector<Point3f> &pattern) {

    pattern.clear();
    for(int i = 0; i < chessboardSize.height; ++i) {
        for(int j = 0; j < chessboardSize.width; ++j) {
            pattern.push_back(Point3f(chessSize*j,chessSize*i,0));
        }
    }
}

bool StereoCalibrator::checkVectors(vector<Mat> &left, vector<Mat> &right) {

    //----- empty? -----
    int lenLeft  = left .size();
    int lenRight = right.size();
    if(lenLeft == 0 || lenRight == 0) {
        cerr << "Cannot calibrate cameras with an empty set of images!" << endl;
        return false;
    }

    //----- same lenght? -----
    if(lenLeft != lenRight) {
        cout << "Warning! The numer of images from left camera\n" <<
                "does not match the number of images from right camera.\n" <<
                "The exceeding images will be discarded." << endl;

        lenLeft < lenRight ? right.resize(lenLeft) : left.resize(lenRight);
    }

    //----- same sizes? -----
    int lenght = left.size();
    Size size = left[0].size();
    for(int i = 0; i < lenght; ++i) {
        Size sizeLeft = left[i].size();
        if(size.width != sizeLeft.width || size.height != sizeLeft.height) {
            cerr << "Cannot calibrate with images of different sizes." << endl;
            return false;
        }
        Size sizeRight = right[i].size();
        if(size.width != sizeRight.width || size.height != sizeRight.height) {
            cerr << "Cannot calibrate with images of different sizes." << endl;
            return false;
        }
    }

    return true;
}


bool StereoCalibrator::saveCalibration(const string &fileName) {
    FileStorage fs(fileName, FileStorage::WRITE);   // opencv file I/O class
    if(!fs.isOpened())
        return false;

    fs << "cameraMatrix1" << cameraMatrix1;
    fs << "distCoeffs1"   << distCoeffs1;
    fs << "cameraMatrix2" << cameraMatrix2;
    fs << "distCoeffs2"   << distCoeffs2;

    fs << "R1" << R1;
    fs << "R2" << R2;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "Q"  << Q;

    fs.release();
    return true;
}

bool StereoCalibrator::loadCalibration(const string &fileName) {
    FileStorage fs(fileName, FileStorage::READ);    // opencv file I/O class
    if(!fs.isOpened())
        return false;

    fs["cameraMatrix1"] >> cameraMatrix1;
    fs["distCoeffs1"  ] >> distCoeffs1;
    fs["cameraMatrix2"] >> cameraMatrix2;
    fs["distCoeffs2"  ] >> distCoeffs2;

    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q" ] >> Q;

    fs.release();
    return true;
}
