#include "depthcomputer.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "stereocalibrator.hpp"

#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer
#include <pcl/visualization/histogram_visualizer.h>	// for histogram visualization
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud


using namespace std;
using namespace cv;
using namespace pcl;

Ptr<StereoSGBM> matcher;
int minDisparities = 0, numDisparities = 144, blockSize = 9;
int p1 = 100, p2 = 3000, disp12MaxDiff, preFilterCap = 4, uniquenessRatio, speckleWindowSize, speckleRange;
const string DISPARITY_WINDOW = "Disparity";
const string IMAGE3D_WINDOW = "3D image";

void on_trackbar(int, void*) {

    if(numDisparities%16 != 0)
        numDisparities += 16 - numDisparities%16;
    if(blockSize%2 == 0)
        ++blockSize;

    setTrackbarPos("Num disparities", DISPARITY_WINDOW, numDisparities);
    setTrackbarPos("Block size",      DISPARITY_WINDOW, blockSize);
    setTrackbarPos("P1",              DISPARITY_WINDOW, p1);
    setTrackbarPos("P2",              DISPARITY_WINDOW, p2);
    setTrackbarPos("PreFilterCap",    DISPARITY_WINDOW, preFilterCap);

    matcher = StereoSGBM::create(minDisparities,numDisparities, blockSize);
    matcher->setPreFilterCap(preFilterCap);
    matcher->setSpeckleRange(2);
    matcher->setSpeckleWindowSize(150);
    matcher->setUniquenessRatio(1);
    matcher->setDisp12MaxDiff(10);
    matcher->setP1(p1);
    matcher->setP2(p2);
}

DepthComputer::DepthComputer(const string& fileName, bool tuneParams) {
    cout << "Reading calibration params..." << endl;
    readCalibration(fileName);
    this->tuneParams = tuneParams;
}

DepthComputer::DepthComputer(const StereoCalibrator& calibrator, bool tuneParams) {
    cout << "Reading calibration params..." << endl;
    readCalibration(calibrator);
    this->tuneParams = tuneParams;
    /*
    cout << "CameraMatrix1:\n" << cameraMatrix1 << endl;
    cout << "DistCoeffs1:\n"   << distCoeffs1   << endl;
    cout << "CameraMatrix2:\n" << cameraMatrix2 << endl;
    cout << "DistCoeffs2:\n"   << distCoeffs2   << endl;
    cout << "R1:\n" << R1 << endl;
    cout << "R2:\n" << R2 << endl;
    cout << "P1:\n" << P1 << endl;
    cout << "P2:\n" << P2 << endl;
    cout << "Q:\n" << Q << endl;
    */
}

void DepthComputer::compute(const Mat left, const Mat right, PointCloud<PointXYZRGB>::Ptr cloud) {
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

    Mat grayRectLeft, grayRectRight;
    cvtColor(rectLeft,  grayRectLeft,  CV_BGR2GRAY);
    cvtColor(rectRight, grayRectRight, CV_BGR2GRAY);

    matcher = StereoSGBM::create(minDisparities,numDisparities, blockSize);
    matcher->setPreFilterCap(preFilterCap);
    matcher->setSpeckleRange(2);
    matcher->setSpeckleWindowSize(150);
    matcher->setUniquenessRatio(1);
    matcher->setDisp12MaxDiff(10);
    matcher->setP1(p1);
    matcher->setP2(p2);

    Mat disparity, image3d;
    namedWindow(DISPARITY_WINDOW, WINDOW_NORMAL);
    namedWindow(IMAGE3D_WINDOW,  WINDOW_NORMAL);

    if(tuneParams) {
        createTrackbar("Min disparities", DISPARITY_WINDOW, &minDisparities, 1024, on_trackbar);
        createTrackbar("Num disparities", DISPARITY_WINDOW, &numDisparities, 1024, on_trackbar);
        createTrackbar("Block size",      DISPARITY_WINDOW, &blockSize,      41,   on_trackbar);
        createTrackbar("P1",              DISPARITY_WINDOW, &p1,             3000, on_trackbar);
        createTrackbar("P2",              DISPARITY_WINDOW, &p2,             3000, on_trackbar);
        createTrackbar("PreFilterCap",    DISPARITY_WINDOW, &preFilterCap,   500,  on_trackbar);
    }

    char ch = 1;
    cout << "Press q to continue." << endl;
    do {
        matcher->compute(grayRectLeft,grayRectRight,disparity);

        normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);
        reprojectImageTo3D(disparity, image3d, Q, false);
        matToPointCloud(rectLeft, image3d, cloud);
        cout << cloud << endl;                                      // TODO: remove line

        imshow(DISPARITY_WINDOW, disparity);
        imshow(IMAGE3D_WINDOW,   image3d);
        ch = tuneParams ? waitKey(100) : 81;

    } while (ch != 81 && ch != 113);

    waitKey(100);

    //return leftMap1;
}

bool DepthComputer::readCalibration(const string& fileName) {
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

bool DepthComputer::readCalibration(const StereoCalibrator& calibrator) {
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

void DepthComputer::matToPointCloud(const Mat& image, const Mat& depthMap, PointCloud<PointXYZRGB>::Ptr cloud) {

    cloud->clear();
    PointXYZRGB p;
    cloud->points.resize(depthMap.cols * depthMap.rows);
    cloud->width  = depthMap.cols*depthMap.rows;
    cloud->height = 1;
    int count = 0;
    for(int i = 0; i < depthMap.rows; ++i) {
        for(int j = 0; j < depthMap.cols; ++j, ++count) {
            Vec3b color    = image.   at<Vec3b>(i,j);
            Vec3f position = depthMap.at<Vec3f>(i,j);
            p.r = color[2];
            p.g = color[1];
            p.b = color[0];
            p.x = position[0]/100;
            p.y = position[1]/100;
            p.z = position[2]/100;
            cloud->at(count) = p;
        }
    }
}


