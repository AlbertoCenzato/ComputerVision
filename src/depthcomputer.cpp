#include "depthcomputer.hpp"

#include <math.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer
#include <pcl/visualization/histogram_visualizer.h>	// for histogram visualization
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud

#include "stereocalibrator.hpp"

using namespace std;
using namespace cv;
using namespace pcl;

const string DepthComputer::DISPARITY_WINDOW   = "Disparity map";
const string DepthComputer::TRACKBAR_MATCHER   = "Stereo matcher parameters";
const string DepthComputer::TRACKBAR_FILTER    = "Stereo filter parameters";
const string DepthComputer::STEREO_PARAMS_FILE = "stereo_params.xml";

// ----- Constructors -----

DepthComputer::DepthComputer(const string &fileName, bool tuneParams) {
    readCalibration(fileName);
    constructor(tuneParams);
}

DepthComputer::DepthComputer(const StereoCalibrator &calibrator, bool tuneParams) {
    readCalibration(calibrator);
    constructor(tuneParams);
}


// ----- Public member functions -----

void DepthComputer::compute(const Mat &left, const Mat &right, Mat &rectLeft, Mat &image3d) {
    cout << "Computing rectification maps..." << endl;
    if (left.empty()) {
        cout << "Empty image!" << endl;
    }

    Mat leftMap1, leftMap2, rightMap1, rightMap2;
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, left.size(),  CV_32FC1, leftMap1,  leftMap2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, right.size(), CV_32FC1, rightMap1, rightMap2);

    Mat rectRight;
    remap(left, rectLeft, leftMap1, leftMap2,  INTER_LINEAR, cv::BORDER_CONSTANT, Scalar(0, 0, 0));
    remap(right,rectRight,rightMap1,rightMap2, INTER_LINEAR, cv::BORDER_CONSTANT, Scalar(0, 0, 0));

    namedWindow("Left",  WINDOW_NORMAL);
    namedWindow("Right", WINDOW_NORMAL);
    imshow("Left",  rectLeft);
    imshow("Right", rectRight);
    waitKey(100);

    cout << "Done." << endl;

    Mat disparity, disparity_right, disparity_filtered;
    namedWindow(DISPARITY_WINDOW, WINDOW_NORMAL);

    if(tuneParams) {
        cout << "Tuning disparity matcher parameters..." << endl;
        tuneMatcher(rectLeft, rectRight, disparity);
    }
    else {
        cout << "Computing disparity match..." << endl;
        matcher->compute(rectLeft,rectRight,disparity);
    }


    disparity.copyTo(disparity_filtered);
    if (tuneParams) {
        cout << "Tuning post filtering..." << endl;
        Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(matcher);
        right_matcher->setSpeckleRange(speckleRange);
        right_matcher->setSpeckleWindowSize(speckleWindowSize);
        right_matcher->setDisp12MaxDiff(disp12MaxDiff);
        right_matcher->compute(rectLeft,rectRight, disparity_right);
        tunePostFilter(disparity,rectLeft,disparity_filtered,disparity_right);
    }
    else if (postFilter == 1){
        cout << "Performing post filtering step..." << endl;
        Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(matcher);
        right_matcher->setSpeckleRange(speckleRange);
        right_matcher->setSpeckleWindowSize(speckleWindowSize);
        right_matcher->setDisp12MaxDiff(disp12MaxDiff);
        right_matcher->compute(rectLeft,rectRight, disparity_right);
        wls_filter->filter(disparity,rectLeft,disparity_filtered,disparity_right);
    }

    normalize(disparity_filtered, disparity_filtered, 0, 255, CV_MINMAX, CV_8U);

    imshow(DISPARITY_WINDOW, disparity_filtered);
    waitKey(100);
    reprojectImageTo3D(disparity_filtered, image3d, Q, false);
}

bool DepthComputer::readCalibration(const string &fileName) {
    FileStorage fs(fileName, FileStorage::READ);
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

bool DepthComputer::readCalibration(const StereoCalibrator &calibrator) {
    cameraMatrix1 = calibrator.cameraMatrix1;
    distCoeffs1   = calibrator.distCoeffs1;
    cameraMatrix2 = calibrator.cameraMatrix2;
    distCoeffs2   = calibrator.distCoeffs2;
    R1 = calibrator.R1;
    R2 = calibrator.R2;
    P1 = calibrator.P1;
    P2 = calibrator.P2;
    Q  = calibrator.Q;

    return true;
}

bool DepthComputer::loadParams(const string &fileName) {
    FileStorage fs(fileName, FileStorage::READ);
    if(!fs.isOpened())
        return false;

    fs["minDisparities"] >> minDisparities;
    fs["numDisparities"] >> numDisparities;
    fs["blockSize"     ] >> blockSize;

    fs["p1"] >> p1;
    fs["p2"] >> p2;

    fs["disp12MaxDiff"    ] >> disp12MaxDiff;
    fs["preFilterCap"     ] >> preFilterCap;
    fs["uniquenessRatio"  ] >> uniquenessRatio;
    fs["speckleWindowSize"] >> speckleWindowSize;
    fs["speckleRange"     ] >> speckleRange;

    fs["lambda"           ] >> lambda;
    fs["sigmaColor"       ] >> sigmaColor;
    fs["depthDiscRadius"  ] >> depthDiscRadius;
    fs["lrcThresh"        ] >> lrcThresh;

    fs["postFilter"       ] >> postFilter;

    fs.release();
    return true;
}

bool DepthComputer::saveParams(const string &fileName) {
    FileStorage fs(fileName, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    fs << "minDisparities" << minDisparities;
    fs << "numDisparities" << numDisparities;
    fs << "blockSize"      << blockSize;

    fs << "p1" << p1;
    fs << "p2" << p2;

    fs << "disp12MaxDiff"     << disp12MaxDiff;
    fs << "preFilterCap"      << preFilterCap;
    fs << "uniquenessRatio"   << uniquenessRatio;
    fs << "speckleWindowSize" << speckleWindowSize;
    fs << "speckleRange"      << speckleRange;

    fs << "lambda"            << lambda;
    fs << "sigmaColor"        << sigmaColor;
    fs << "depthDiscRadius"   << depthDiscRadius;
    fs << "lrcThresh"         << lrcThresh;

    fs << "postFilter"        << postFilter;

    fs.release();
    return true;
}

// ----- Private member functions -----

void DepthComputer::constructor(bool tuneParams) {
    this->tuneParams = tuneParams;
    if (!loadParams(STEREO_PARAMS_FILE)) {
        cerr << "File " << STEREO_PARAMS_FILE << " not found or corrupted, using default parameters." << endl;
    }

    matcher = StereoSGBM::create(minDisparities,numDisparities, blockSize);
    matcher->setPreFilterCap(preFilterCap);
    matcher->setSpeckleRange(speckleRange);
    matcher->setSpeckleWindowSize(speckleWindowSize);
    matcher->setUniquenessRatio(uniquenessRatio);
    matcher->setDisp12MaxDiff(disp12MaxDiff);
    matcher->setP1(p1);
    matcher->setP2(p2);

    wls_filter = ximgproc::createDisparityWLSFilter(matcher);
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigmaColor/10);
    wls_filter->setDepthDiscontinuityRadius(depthDiscRadius);
    wls_filter->setLRCthresh(lrcThresh);
}

void DepthComputer::tuneMatcher(const Mat &rectLeft, const Mat &rectRight, Mat &disparity) {

    namedWindow(TRACKBAR_MATCHER,  WINDOW_NORMAL);
    createTrackbar("Min disparities",   TRACKBAR_MATCHER, &minDisparities,    1024, on_trackbar_matcher, this);
    createTrackbar("Num disparities",   TRACKBAR_MATCHER, &numDisparities,    1024, on_trackbar_matcher, this);
    createTrackbar("Block size",        TRACKBAR_MATCHER, &blockSize,         41,   on_trackbar_matcher, this);
    createTrackbar("P1",                TRACKBAR_MATCHER, &p1,                3000, on_trackbar_matcher, this);
    createTrackbar("P2",                TRACKBAR_MATCHER, &p2,                3000, on_trackbar_matcher, this);
    createTrackbar("PreFilterCap",      TRACKBAR_MATCHER, &preFilterCap,      500,  on_trackbar_matcher, this);
    createTrackbar("SpeckleRange",      TRACKBAR_MATCHER, &speckleRange,      100,  on_trackbar_matcher, this);
    createTrackbar("SpeckleWindowSize", TRACKBAR_MATCHER, &speckleWindowSize, 300,  on_trackbar_matcher, this);
    createTrackbar("UniquenessRatio",   TRACKBAR_MATCHER, &uniquenessRatio,   100,  on_trackbar_matcher, this);
    createTrackbar("disp12MaxDiff",     TRACKBAR_MATCHER, &disp12MaxDiff,     500,  on_trackbar_matcher, this);

    char ch;
    Mat disparity_norm;
    do {
        matcher->compute(rectLeft,rectRight,disparity);

        normalize(disparity, disparity_norm, 0, 255, CV_MINMAX, CV_8U);

        imshow(DISPARITY_WINDOW, disparity_norm);

        cout << "Press q to confirm stereo parameters and continue, r to recompute the scene." << endl;
        ch = waitKey(0);

    } while (ch != 'q' && ch != 'Q');

    do {
        cout << "Save stereo matcher parameters? [y/n]" << endl;
        cin >> ch;
    } while (ch != 'Y' && ch != 'y' && ch != 'N' && ch != 'n');

    if(ch == 'y' || ch == 'Y') {
       if(!saveParams(STEREO_PARAMS_FILE))
           cerr << "Error! Unable to save parameters" << endl;
    }

    destroyWindow(TRACKBAR_MATCHER);
}

void DepthComputer::tunePostFilter(const Mat &disparity, const Mat &rectLeft, Mat &disparity_filtered, const Mat &disparity_right) {

    namedWindow(TRACKBAR_FILTER,  WINDOW_NORMAL);
    createTrackbar("Enable post filter", TRACKBAR_FILTER, &postFilter,      1,     on_trackbar_filter, this);
    createTrackbar("Lambda",             TRACKBAR_FILTER, &lambda,          10000, on_trackbar_filter, this);
    createTrackbar("Sigma color",        TRACKBAR_FILTER, &sigmaColor,      100,   on_trackbar_filter, this);
    createTrackbar("Depth disc. radius", TRACKBAR_FILTER, &depthDiscRadius, 100,   on_trackbar_filter, this);
    createTrackbar("LRC threshold",      TRACKBAR_FILTER, &lrcThresh,       100,   on_trackbar_filter, this);


    char ch;
    Mat disparity_filtered_norm;
    do {
        cout << "Post filtering: " << (postFilter == 1 ? "ON" : "OFF") << endl;
        if (postFilter == 1) {
            wls_filter->filter(disparity,rectLeft,disparity_filtered,disparity_right);
        }
        else {
            disparity.copyTo(disparity_filtered);
        }

        normalize(disparity_filtered, disparity_filtered_norm, 0, 255, CV_MINMAX, CV_8U);
        imshow(DISPARITY_WINDOW, disparity_filtered_norm);

        cout << "Press q to confirm filter parameters and continue, r to recompute the scene." << endl;
        ch = waitKey(0);

    } while (ch != 'q' && ch != 'Q');

    do {
        cout << "Save stereo filter parameters? [y/n]" << endl;
        cin >> ch;
    } while (ch != 'Y' && ch != 'y' && ch != 'N' && ch != 'n');

    if(ch == 'y' || ch == 'Y') {
       if(!saveParams(STEREO_PARAMS_FILE))
           cerr << "Error! Unable to save parameters" << endl;
    }

    destroyWindow(TRACKBAR_FILTER);
}

void DepthComputer::on_trackbar_matcher(int pos, void *o) {

    DepthComputer *obj = (DepthComputer*)o;

    if(obj->numDisparities%16 != 0)
        obj->numDisparities += 16 - obj->numDisparities%16;
    if(obj->blockSize % 2 == 0)
        obj->blockSize++;

    //self->matcher = StereoSGBM::create(self->minDisparities,self->numDisparities,self->blockSize);
    obj->matcher->setMinDisparity(obj->minDisparities);
    obj->matcher->setNumDisparities(obj->numDisparities);
    obj->matcher->setBlockSize(obj->blockSize);
    obj->matcher->setPreFilterCap(obj->preFilterCap);
    obj->matcher->setSpeckleRange(obj->speckleRange);
    obj->matcher->setSpeckleWindowSize(obj->speckleWindowSize);
    obj->matcher->setUniquenessRatio(obj->uniquenessRatio);
    obj->matcher->setDisp12MaxDiff(obj->disp12MaxDiff);
    obj->matcher->setP1(obj->p1);
    obj->matcher->setP2(obj->p2);
}

void DepthComputer::on_trackbar_filter(int pos, void *o) {

    DepthComputer *obj = (DepthComputer*)o;

    if(obj->postFilter == 1) {
        obj->wls_filter->setLambda(obj->lambda);
        obj->wls_filter->setSigmaColor(obj->sigmaColor/10);
        obj->wls_filter->setDepthDiscontinuityRadius(obj->depthDiscRadius);
        obj->wls_filter->setLRCthresh(obj->lrcThresh);
    }
}


