#include <cfloat>
#include <boost/lexical_cast.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>	    // for computing normals
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/video/video.hpp>
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
    "{tune           |       | if set allows to tune the algorithm parameters during execution }"
    "{downscaling    |   1   | set to a value in [0,1] interval to have all images downscaled  }";

bool calibrateCameras(ImageLoader &loader);
void matToPointCloud(const Mat &depthMap, const Mat &image, PointCloud<PointXYZRGB>::Ptr cloud);

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc,argv,KEYS);      // opencv parser for command line arguments

    // if wrong arguments, print usage
    if(!parser.has("@leftImage") || !parser.has("@rightImage")) {
        parser.printMessage();
        return 0;
    }

    ImageLoader loader = ImageLoader();
    loader.downscalingRatio = parser.get<float>("downscaling");

    //----- calibration (optional) -----
    if(parser.has("calibrate")) {
        if (!calibrateCameras(loader)) {
            cout << "Calibration failed!" << endl;
            return 0;
        }
    }

    //----- load images -----
    string pathLeft  = parser.get<string>(0);
    string pathRight = parser.get<string>(1);
    Mat imgLeft, imgRight;

    if(!loader.get(pathLeft, imgLeft) || !loader.get(pathRight,imgRight)) {
        cerr << "Empty images!" << endl;
        cout << "Maybe you mistyped an argument." << endl;
        parser.printMessage();
        return 0;
    }

    //----- compute depth -----
    bool tuneParams = parser.has("tune");
    DepthComputer dptComputer(CALIB_FILE, tuneParams);

    Mat rectLeft, image3d;

    auto begin = chrono::high_resolution_clock::now();
    dptComputer.compute(imgLeft, imgRight, rectLeft, image3d);
    auto end = chrono::high_resolution_clock::now();
    cout << "\nDepth: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

    //----- conversion to point cloud -----
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    matToPointCloud(image3d, rectLeft, cloud);

    //----- visualization -----
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

/**
 * @brief calibrateCameras performs user interactions and uses StereoCalibrator
 *        class to calibrate the stereo cameras
 * @param loader
 * @return
 */
bool calibrateCameras(ImageLoader &loader) {

    cout << "Please provide chessboard pattern width." << endl;
    int width;
    cin >> width;

    cout << "Please provide chessboard pattern height." << endl;
    int height;
    cin >> height;
    cin.ignore(25,'\n');    // removes line feed from the buffer to have a clean call to getline()

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

        if (loader.get(pathLeft,left) && loader.get(pathRight,right)) {
            cout << "Images loaded successfully!" << endl;
            ch = 'q';
        }
        else {
            cout << "Error loading images!"
                 << "Would you like to retry? [y/n]" << endl;
            cin >> ch;
            cin.ignore(25,'\n');
            if (ch == 'n')
                return false;
        }
    } while (ch != 'q' && ch != 'Q');

    StereoCalibrator calib(size);

    auto begin = chrono::high_resolution_clock::now();
    cout << "Re-projection error: " << calib.compute(left, right) << endl;
    auto end = chrono::high_resolution_clock::now();
    cout << "\nCalibration: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

    calib.saveCalibration(CALIB_FILE);

    return true;
}

/**
 * @brief matToPointCloud converts an OpenCV depth map to a PCL point cloud
 * @param depthMap
 * @param image RGB information for the depth map
 * @param cloud output point cloud
 */
void matToPointCloud(const Mat &depthMap, const Mat &image, PointCloud<PointXYZRGB>::Ptr cloud) {

    cloud->clear();
    PointXYZRGB p;
    cloud->points.resize(depthMap.cols * depthMap.rows);    // resizing the cloud to avoid dynamic reallocation
    cloud->width  = depthMap.cols*depthMap.rows;
    cloud->height = 1;
    int count = 0;
    for(int i = 0; i < depthMap.rows; ++i) {
        for(int j = 0; j < depthMap.cols; ++j, ++count) {
            Vec3f position = depthMap.at<Vec3f>(i,j);

            // removing nan and infinity valued points for visualization
            if(isinf(position[2]) || isinf(position[1]) || isinf(position[0]) ||
               isnan(position[2]) || isnan(position[1]) || isnan(position[0]))
                continue;

            Vec3b color    = image.at<Vec3b>(i,j);
            p.r = color[2];
            p.g = color[1];
            p.b = color[0];
            p.x = position[0];
            p.y = position[1];
            p.z = position[2];
            cloud->at(count) = p;
        }
    }
}


