//
// Created by alberto on 05/03/18.
//

#ifndef CV_HW2_LAB3_UTILS_H
#define CV_HW2_LAB3_UTILS_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

//convenient structure to handle our pointclouds
template<typename PointT>
struct PCD
{
    typename pcl::PointCloud<PointT>::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new pcl::PointCloud<PointT>) {}
};

template<typename PointT>
struct PCDComparator
{
    bool operator() (const PCD<PointT>& p1, const PCD<PointT>& p2) {
        return (p1.f_name < p2.f_name);
    }

};


////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
template<typename PointT>
std::vector<PCD<PointT>, Eigen::aligned_allocator<PCD<PointT>>> loadData(int argc, char **argv, bool downsample)
{
    std::vector<PCD<PointT>, Eigen::aligned_allocator<PCD<PointT>>> models;

    std::string extension (".pcd");
    // Suppose the first argument is the actual test model
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string (argv[i]);
        // Needs to be at least 5: .plot
        if (fname.size () <= extension.size ())
            continue;

        std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

        //check that the argument is a pcd file
        if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
        {
            // Load the cloud and saves it into the global list of models
            PCD<PointT> m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile (argv[i], *m.cloud);
            //remove NAN points from the cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

            // Downsample for consistency and speed
            // \note enable this for large datasets
            if (downsample) {
                pcl::VoxelGrid<PointT> grid;
                grid.setLeafSize(0.05, 0.05, 0.05);
                grid.setInputCloud(m.cloud);
                grid.filter(*m.cloud);
            }

            models.push_back(m);
        }
    }

    return models;  // using vector<> move semantic to efficently return models
}



#endif //CV_HW2_LAB3_UTILS_H
