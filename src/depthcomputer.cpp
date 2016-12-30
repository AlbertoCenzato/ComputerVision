#include "depthcomputer.hpp"

#include <math.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "extra-module/disparity_filter.hpp"

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
const string DepthComputer::TRACKBAR_WINDOW    = "Stereo matcher parameters";
const string DepthComputer::STEREO_PARAMS_FILE = "stereo_params.xml";

// ----- Constructors -----

DepthComputer::DepthComputer(const string &fileName, bool tuneParams) {
    cout << "Reading calibration params..." << endl;
    readCalibration(fileName);
    this->tuneParams = tuneParams;
    if (!loadParams(STEREO_PARAMS_FILE)) {
        cerr << "File " << STEREO_PARAMS_FILE << " not found or corrupted, using default parameters." << endl;
    }
}

DepthComputer::DepthComputer(const StereoCalibrator &calibrator, bool tuneParams) {
    cout << "Reading calibration params..." << endl;
    readCalibration(calibrator);
    this->tuneParams = tuneParams;
    if (!loadParams(STEREO_PARAMS_FILE)) {
        cerr << "File " << STEREO_PARAMS_FILE << " not found or corrupted, using default parameters." << endl;
    }
}


// ----- Public member functions -----

void DepthComputer::compute(const Mat &left, const Mat &right, PointCloud<PointXYZRGB>::Ptr cloud) {
    cout << "Computing rectification maps..." << endl;
    if (left.empty()) {
        cout << "Empty image!" << endl;
    }

    Mat leftMap1, leftMap2, rightMap1, rightMap2;
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, left.size(),  CV_32FC1, leftMap1,  leftMap2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, right.size(), CV_32FC1, rightMap1, rightMap2);

    Mat rectLeft, rectRight;
    remap(left, rectLeft, leftMap1, leftMap2,  INTER_LINEAR, cv::BORDER_CONSTANT, Scalar(0, 0, 0));
    remap(right,rectRight,rightMap1,rightMap2, INTER_LINEAR, cv::BORDER_CONSTANT, Scalar(0, 0, 0));

    namedWindow("Left",  WINDOW_NORMAL);
    namedWindow("Right", WINDOW_NORMAL);
    imshow("Left",  rectLeft);
    imshow("Right", rectRight);

    waitKey(100);

    /*
    Mat grayRectLeft, grayRectRight;
    cvtColor(rectLeft,  grayRectLeft,  CV_BGR2GRAY);
    cvtColor(rectRight, grayRectRight, CV_BGR2GRAY);
    */

    matcher = StereoSGBM::create(minDisparities,numDisparities, blockSize);
    matcher->setPreFilterCap(preFilterCap);
    matcher->setSpeckleRange(speckleRange);
    matcher->setSpeckleWindowSize(speckleWindowSize);
    matcher->setUniquenessRatio(uniquenessRatio);
    matcher->setDisp12MaxDiff(disp12MaxDiff);
    matcher->setP1(p1);
    matcher->setP2(p2);

    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(matcher);
    Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(matcher);

    Mat disparity, disparity_right, disparity_filtered, disparity_filtered_norm, image3d;
    namedWindow(DISPARITY_WINDOW, WINDOW_NORMAL);
    namedWindow(TRACKBAR_WINDOW,  WINDOW_NORMAL);

    if(tuneParams) {
        createTrackbar("Min disparities",   TRACKBAR_WINDOW, &minDisparities,    1024, on_trackbar, this);
        createTrackbar("Num disparities",   TRACKBAR_WINDOW, &numDisparities,    1024, on_trackbar, this);
        createTrackbar("Block size",        TRACKBAR_WINDOW, &blockSize,         41,   on_trackbar, this);
        createTrackbar("P1",                TRACKBAR_WINDOW, &p1,                3000, on_trackbar, this);
        createTrackbar("P2",                TRACKBAR_WINDOW, &p2,                3000, on_trackbar, this);
        createTrackbar("PreFilterCap",      TRACKBAR_WINDOW, &preFilterCap,      500,  on_trackbar, this);
        createTrackbar("SpeckleRange",      TRACKBAR_WINDOW, &speckleRange,      10,   on_trackbar, this);
        createTrackbar("SpeckleWindowSize", TRACKBAR_WINDOW, &speckleWindowSize, 300,  on_trackbar, this);
        createTrackbar("UniquenessRatio",   TRACKBAR_WINDOW, &uniquenessRatio,   100,  on_trackbar, this);
        createTrackbar("disp12MaxDiff",     TRACKBAR_WINDOW, &disp12MaxDiff,     500,  on_trackbar, this);
    }

    char ch;
    do {
        matcher->compute(rectLeft,rectRight,disparity);
        right_matcher->compute(rectRight,rectLeft, disparity_right);

        wls_filter->filter(disparity,rectLeft,disparity_filtered,disparity_right);

        normalize(disparity_filtered, disparity_filtered_norm, 0, 255, CV_MINMAX, CV_8U);
        reprojectImageTo3D(disparity_filtered, image3d, Q, false);
        //normalize(disparity, disparity_filtered_norm, 0, 255, CV_MINMAX, CV_8U);
        //reprojectImageTo3D(disparity, image3d, Q, false);

        matToPointCloud(rectLeft, image3d, cloud);

        imshow(DISPARITY_WINDOW, disparity_filtered_norm);

        cout << "Press q to confirm stereo parameters and continue, r to recompute the scene." << endl;
        ch = tuneParams ? waitKey(0) : 'q';

    } while (ch != 'q' && ch != 'Q');

    waitKey(100);

    if (tuneParams) {
        cout << "Save stereo matcher parameters? [y/n]" << endl;
        char ch;
        cin >> ch;
        if(ch == 'y')
            saveParams(STEREO_PARAMS_FILE);
    }

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

    fs.release();
    return true;
}


void DepthComputer::matToPointCloud(const Mat &image, const Mat &depthMap, PointCloud<PointXYZRGB>::Ptr cloud) {

    cloud->clear();
    PointXYZRGB p;
    cloud->points.resize(depthMap.cols * depthMap.rows);
    cloud->width  = depthMap.cols*depthMap.rows;
    cloud->height = 1;
    int count = 0;
    for(int i = 0; i < depthMap.rows; ++i) {
        for(int j = 0; j < depthMap.cols; ++j, ++count) {
            Vec3f position = depthMap.at<Vec3f>(i,j);
            if(isinf(position[2]) || isinf(position[1]) || isinf(position[0]) ||
               isnan(position[2]) || isnan(position[1]) || isnan(position[0]))
                continue;

            Vec3b color    = image.   at<Vec3b>(i,j);
            p.r = color[2];
            p.g = color[1];
            p.b = color[0];
            p.x = position[0];
            p.y = position[1];
            p.z = position[2];
            cloud->at(count) = p;
            //cout << p << endl;
        }
    }
}


// ----- Private member functions -----

void DepthComputer::on_trackbar(int pos, void *obj) {

    DepthComputer *self = (DepthComputer*)obj;

    if(self->numDisparities%16 != 0)
        self->numDisparities += 16 - self->numDisparities%16;
    if(self->blockSize % 2 == 0)
        self->blockSize++;

    self->matcher = StereoSGBM::create(self->minDisparities,self->numDisparities,self->blockSize);
    self->matcher->setPreFilterCap(self->preFilterCap);
    self->matcher->setSpeckleRange(self->speckleRange);
    self->matcher->setSpeckleWindowSize(self->speckleWindowSize);
    self->matcher->setUniquenessRatio(self->uniquenessRatio);
    self->matcher->setDisp12MaxDiff(self->disp12MaxDiff);
    self->matcher->setP1(self->p1);
    self->matcher->setP2(self->p2);
}

