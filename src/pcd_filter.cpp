#include <iostream>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main (int argc, char** argv)
{
    std::string pc_path;

    // Setup options.
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "Print help messages")
        ("pc_path", po::value<std::string>(&pc_path));
    
    // parse
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::cout << "cloud1 path: " << pc_path << std::endl; 
    
    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pc_path, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read pcd file! \n");
        return (-1);
    }
    // remove NaN
    std::vector<int> target_indices; // mark the removed points  
    pcl::removeNaNFromPointCloud(*cloud,*cloud, target_indices);

    // bounding box with FOV_H: 40deg, FOV_V: 20deg @200m
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    bbox_ptr->push_back(pcl::PointXYZ(200, 72.79405, 35.2654));
    bbox_ptr->push_back(pcl::PointXYZ(200, -72.79405, 35.2654));
    bbox_ptr->push_back(pcl::PointXYZ(200, -72.79405, -35.2654));
    bbox_ptr->push_back(pcl::PointXYZ(200, 72.79405, -35.2654));
    bbox_ptr->push_back(pcl::PointXYZ(0, 0, 0));

    // convelHull object
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(bbox_ptr);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);

    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;
    bb_filter.setDim(3);
    bb_filter.setInputCloud(cloud);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(surface_hull);
    bb_filter.filter(*objects);
    std::cout << objects->points.size() << std::endl;

    /*******/
    // VIZ //
    /*******/
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
    viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0,0,0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    input_color (cloud, 255, 255, 255);
    viewer_final->addPointCloud<pcl::PointXYZ> (cloud, input_color, "input cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "input cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    surface_color (surface_hull, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (surface_hull, surface_color, "surface");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    10, "surface");
    viewer_final->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069*255, 0.2*255);
                                                    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    filtered_color (cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (objects, filtered_color, "objects");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "objects");                                            

    // Starting visualizer
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}