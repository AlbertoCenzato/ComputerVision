#include <float.h>
#include <boost/lexical_cast.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>	    // for computing normals
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud


#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "imageloader.hpp"
#include "stereocalibrator.hpp"
#include "depthcomputer.hpp"

using namespace std;
using namespace cv;
using namespace pcl;


const string CALIB_FILE = "calibration.xml";
const String KEYS =
    "{help h usage ? |       | print this message   }"
    "{@leftImage     |       | left stereo image    }"
    "{@rightImage    |       | right stereo image   }"
    "{calibrate      |       | if set calibrates camera before depth evauation (requires --left and --right)}"
    "{left           |       | path to left calibration image sequence expressed as /path/to/file/filename%0#d.jpg\n"
                              "where # is the number of digits used in filename}"
    "{right          |       | path to right calibration image sequence expressed as /path/to/file/filename%0#d.jpg\n"
                              "where # is the number of digits used in filename}"
    "{tune           |       | if set allows to tune the algorithm parameters during execution }";

void showImages(vector<Mat>& images);
void calibrateCameras(ImageLoader &loader, const CommandLineParser &parser);

int main(int argc, char *argv[])
{

    CommandLineParser parser(argc,argv,KEYS);

    if(argc < 3) {
        parser.printMessage();
        return 0;
    }

    ImageLoader loader = ImageLoader();
    loader.downscalingRatio = 0.25;

    if(parser.has("calibrate") && parser.has("left") && parser.has("right")) {
        calibrateCameras(loader, parser);
    }

    string pathLeft  = parser.get<string>(0);
    string pathRight = parser.get<string>(1);
    Mat imgLeft, imgRight;

    if(!loader.getImage(pathLeft, imgLeft) || !loader.getImage(pathRight,imgRight)) {
        cerr << "Empty images!" << endl;
        return -1;
    }

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    bool tuneParams = parser.has("tune");
    DepthComputer dptComputer = DepthComputer(CALIB_FILE, tuneParams);
    dptComputer.compute(imgLeft, imgRight, cloud);

    for(int i = 0; i < cloud->size(); ++i) {
        cout << cloud->at(i);
    }


    visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor  (0.0, 0.0, 0.5);
    viewer.addCoordinateSystem (0.1);
    viewer.initCameraParameters();
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<PointXYZRGB> (cloud, rgb, "input_cloud");

    /*
    visualization::CloudViewer viewer("Viewer");
    //viewer.setBackgroundColor (1.0, 1.0, 1.0);
    //viewer.initCameraParameters ();
    //visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
    //viewer.addPointCloud<PointXYZRGB> (cloud, rgb, "input_cloud");
    */

    //viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
        viewer.spin();
    }

    return 0;
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

void calibrateCameras(ImageLoader& loader, const CommandLineParser& parser) {
    cout << "Loading images..." << endl;

    string pathLeft  = parser.get<string>("left");
    string pathRight = parser.get<string>("right");
    vector<Mat> left(20), right(20);

    if(loader.getImage(pathLeft,left) && loader.getImage(pathRight,right))
        cout << "Images loaded successfully!" << endl;
    else {
        cout << "Error loading images!" << endl;
        return;
    }

    Size size(7,6);
    showImages(left);

    StereoCalibrator calib = StereoCalibrator(size, 12);
    cout << "Re-projection error: " << calib.calibrate(left, right) << endl;
    calib.writeCalibration(CALIB_FILE);
}


