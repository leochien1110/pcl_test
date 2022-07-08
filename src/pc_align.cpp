#include <iostream>
#include <thread>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main (int argc, char** argv)
{
    std::string pc1_path;
    std::string pc2_path;
    std::string method;
    float x = 0, y = 0, z = 0, theta = 0;

    // Setup options.
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "Print help messages")
        ("pc1_path", po::value<std::string>(&pc1_path))
        ("pc2_path", po::value<std::string>(&pc2_path))
        ("method", po::value<std::string>(&method))
        ("x", po::value<float>(&x))
        ("y", po::value<float>(&y))
        ("z", po::value<float>(&z))
        ;
    
    // parse
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::cout << "cloud1 path: " << pc1_path << " cloud2 path: " << pc2_path << std::endl; 
    std::cout << "method: " << method << std::endl;
    std::cout << "initial pose: " << x << ", " << y << ", " << z << ", " << theta << std::endl;

    if (method != "ndt" && method != "icp") {
        std::cerr << "Not a valid alignment method! It must be either 'ndt' or 'icp'." << std::endl;
        return 0;
    }

    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pc1_path, *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    // remove NaN
    std::vector<int> target_indices; // mark the removed points  
    pcl::removeNaNFromPointCloud(*target_cloud,*target_cloud, target_indices);

    std::cout << "Loaded " << target_cloud->size () << " data points from target.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pc2_path, *input_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    // remove NaN
    std::vector<int> input_indices; // mark the removed points  
    pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud, input_indices);
    std::cout << "Loaded " << input_cloud->size () << " data points from inout.pcd" << std::endl;


    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size ()
                << " data points from room_scan2.pcd" << std::endl;

    /*******/
    // NDT //
    /*******/
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.005);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (50);

    // Setting point cloud to be aligned.
    ndt.setInputSource (filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    // Eigen::Translation3f init_translation (1.59387, 0.220047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*ndt_output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
                << " score: " << ndt.getFitnessScore () << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    // pcl::transformPointCloud (*input_cloud, *ndt_output_cloud, ndt.getFinalTransformation ());
    std::cout << "NDT transform: \n" << ndt.getFinalTransformation() << std::endl;

    // Saving transformed input cloud.
    // pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *ndt_output_cloud);

    /*******/
    // ICP //
    /*******/
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(target_cloud);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher
    // distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*icp_output_cloud, init_guess);

    std::cout << "IterativeClosestPoint has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << "ICP transform: \n" << icp.getFinalTransformation() << std::endl;

    /*******/
    // VIZ //
    /*******/
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
    viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    target_color (target_cloud, 255, 255, 255);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");

    // Coloring and visualizing ndt transformed cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    ndt_output_color (ndt_output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (ndt_output_cloud, ndt_output_color, "ndt output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "ndt output cloud");

    // Coloring and visualizing icp transformed cloud (red)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    icp_output_color (icp_output_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (icp_output_cloud, icp_output_color, "icp output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "icp output cloud");

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