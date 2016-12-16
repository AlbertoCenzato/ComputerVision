#include <float.h>
#include <boost/lexical_cast.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils.hpp"
#include "stereocalibrator.h"

using namespace std;
using namespace cv;
using namespace pcl;

bool loadImages(string& path, vector<Mat> &images);
void showImages(vector<Mat>& images);

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
    StereoCalibrator calib = StereoCalibrator();
    calib.calibrateStereo(size, 12, left, right);

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


