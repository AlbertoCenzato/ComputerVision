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
    "{calibrate      |       | if set calibrates camera before depth evauation}"
    "{tune           |       | if set allows to tune the algorithm parameters during execution }";

void showImages(vector<Mat>& images);
bool calibrateCameras(ImageLoader &loader);

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc,argv,KEYS);

    if(argc < 3) {
        parser.printMessage();
        return 0;
    }

    ImageLoader loader = ImageLoader();
    //loader.downscalingRatio = 0.5;

    if(parser.has("calibrate")) {
        if (!calibrateCameras(loader)) {
            cout << "Calibration failed!" << endl;
            return 0;
        }
    }

    string pathLeft  = parser.get<string>(0);
    string pathRight = parser.get<string>(1);
    Mat imgLeft, imgRight;

    if(!loader.getImage(pathLeft, imgLeft) || !loader.getImage(pathRight,imgRight)) {
        cerr << "Empty images!" << endl;
        cout << "Maybe you mistyped an argument." << endl;
        parser.printMessage();
        return 0;
    }

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    bool tuneParams = parser.has("tune");
    DepthComputer dptComputer = DepthComputer(CALIB_FILE, tuneParams);

    auto begin = chrono::high_resolution_clock::now();
    dptComputer.compute(imgLeft, imgRight, cloud);
    auto end = chrono::high_resolution_clock::now();
    cout << "\nDepth: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;


    waitKey(0);

    visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor  (0.0, 0.0, 0.5);
    viewer.addCoordinateSystem (0.1);
    viewer.initCameraParameters();
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<PointXYZRGB> (cloud, rgb, "input_cloud");

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

bool calibrateCameras(ImageLoader &loader) {

    cout << "Please provide chessboard pattern width." << endl;
    int width;
    cin >> width;

    cout << "Please provide chessboard pattern height." << endl;
    int height;
    cin >> height;
    cin.ignore(25,'\n');

    //string empty;
    //getline(cin,empty); // TODO: this trick sucks... find a better way

    Size size(width,height);

    vector<Mat> left(20), right(20);
    char ch = 'q';
    do {
        cout << "Please provide the path to left calibration image sequence;\n"
             << "it must be expressed as /path/to/file/filename%0#d.jpg\n"
             << "where # is the number of digits used in filename." << endl;

        string pathLeft;
        getline(cin,pathLeft);

        cout << "Please provide the path to right calibration image sequence." << endl;

        string pathRight;
        getline(cin,pathRight);

        cout << "Loading images for calibration..." << endl;

        if(loader.getImage(pathLeft,left) && loader.getImage(pathRight,right)) {
            cout << "Images loaded successfully!" << endl;
            ch = 'q';
        }
        else {
            cout << "Error loading images!"
                 << "Would you like to retry? [y/n]" << endl;
            cin >> ch;
            cin.ignore(25,'\n');
            if(ch == 'n')
                return false;
            else
                ch = 'a';
        }
    } while(ch != 'q' && ch != 'Q');

    StereoCalibrator calib = StereoCalibrator(size, 12);


    auto begin = chrono::high_resolution_clock::now();
    cout << "Re-projection error: " << calib.compute(left, right) << endl;
    auto end = chrono::high_resolution_clock::now();
    cout << "\nCalibration: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

    calib.saveCalibration(CALIB_FILE);

    return true;
}


