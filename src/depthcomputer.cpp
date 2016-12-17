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

void on_trackbar(int, void*) {
    if(numDisparities%16 != 0)
        numDisparities += 16 - numDisparities%16;
    if(blockSize%2 == 0)
        ++blockSize;

    setTrackbarPos("Num disparities", "Disparity", numDisparities);
    setTrackbarPos("Block size", "Disparity", blockSize);
    setTrackbarPos("P1", "Disparity", p1);
    setTrackbarPos("P2", "Disparity", p2);
    setTrackbarPos("PreFilterCap", "Disparity", preFilterCap);
    matcher = StereoSGBM::create(minDisparities,numDisparities, blockSize);
    //matcher->setPreFilterSize(5);
    matcher->setPreFilterCap(preFilterCap);
    matcher->setSpeckleRange(2);
    matcher->setSpeckleWindowSize(150);
    matcher->setUniquenessRatio(1);
    matcher->setDisp12MaxDiff(10);
    matcher->setP1(p1);
    matcher->setP2(p2);
}

DepthComputer::DepthComputer(const string& fileName) {
    readCalibration(fileName);
}

DepthComputer::DepthComputer(const StereoCalibrator& calibrator) {
    cout << "Reading calibration params..." << endl;
    readCalibration(calibrator);
    cout << "CameraMatrix1:\n" << cameraMatrix1 << endl;
    cout << "DistCoeffs1:\n"   << distCoeffs1   << endl;
    cout << "CameraMatrix2:\n" << cameraMatrix2 << endl;
    cout << "DistCoeffs2:\n"   << distCoeffs2   << endl;
    cout << "R1:\n" << R1 << endl;
    cout << "R2:\n" << R2 << endl;
    cout << "P1:\n" << P1 << endl;
    cout << "P2:\n" << P2 << endl;
    cout << "Q:\n" << Q << endl;

}

void DepthComputer::compute(const Mat left, const Mat right, PointCloud<PointXYZ>::Ptr& cloud) {
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

    namedWindow("Left", WINDOW_NORMAL);
    namedWindow("Right", WINDOW_NORMAL);
    imshow("Left",  rectLeft);
    imshow("Right", rectRight);

    waitKey(0);


    cvtColor(rectLeft,  rectLeft,  CV_BGR2GRAY);
    cvtColor(rectRight, rectRight, CV_BGR2GRAY);

    Mat disparity;
    namedWindow("Disparity", WINDOW_NORMAL);


    createTrackbar("Min disparities", "Disparity", &minDisparities, 1024, on_trackbar);
    createTrackbar("Num disparities", "Disparity", &numDisparities, 1024, on_trackbar);
    createTrackbar("Block size", "Disparity", &blockSize, 41, on_trackbar);
    createTrackbar("P1", "Disparity", &p1, 3000, on_trackbar);
    createTrackbar("P2", "Disparity", &p2, 3000, on_trackbar);
    createTrackbar("PreFilterCap", "Disparity", &preFilterCap, 500, on_trackbar);
    //createTrackbar("Block size", "Disparity", &blockSize, 41, on_trackbar);
    //createTrackbar("Block size", "Disparity", &blockSize, 41, on_trackbar);


   // numDisparities = (rectLeft.cols/8 + 15) & -16;

    matcher = StereoSGBM::create(minDisparities,numDisparities, blockSize);
    //auto wls_filter = createDisparity

    /*
    matcher->setPreFilterCap(63);
    int sgbmWinSize = 3;
    matcher->setBlockSize(sgbmWinSize);

    int cn = 1;

    matcher->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    matcher->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    matcher->setMinDisparity(40);
    matcher->setNumDisparities(numDisparities);
    matcher->setUniquenessRatio(10);
    matcher->setSpeckleWindowSize(100);
    matcher->setSpeckleRange(32);
    matcher->setDisp12MaxDiff(1);
    */

    matcher->setPreFilterCap(preFilterCap);
    matcher->setSpeckleRange(2);
    matcher->setSpeckleWindowSize(150);
    matcher->setUniquenessRatio(1);
    matcher->setDisp12MaxDiff(10);
    matcher->setP1(p1);
    matcher->setP2(p2);

    namedWindow("3D image", WINDOW_NORMAL);

    //while (true) {

        matcher->compute(rectLeft,rectRight,disparity);

        Mat image3d;

        reprojectImageTo3D(disparity, image3d, Q, true);
        matToPointCloud(image3d, cloud);

        imshow("3D image", image3d);

        normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);

        imshow("Disparity",  disparity);
        waitKey(100);

   // }

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

void DepthComputer::matToPointCloud(const Mat& mat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    cloud->clear();
    for(int i = 0; i < mat.rows; ++i) {
        for(int j = 0; j < mat.cols; ++j) {
            Vec3d point = mat.at<Vec3d>(i,j);
            pcl::PointXYZ p(point[0],point[1],point[2]);
            cloud->push_back(p);
        }
    }

}
