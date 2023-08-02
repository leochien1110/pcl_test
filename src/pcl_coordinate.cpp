#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	pcl::visualization::PCLVisualizer viewer("cloud viewer");
	viewer.addCoordinateSystem();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}