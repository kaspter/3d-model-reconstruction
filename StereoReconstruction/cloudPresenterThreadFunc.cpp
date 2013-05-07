#include "stdafx.h"

DWORD CALLBACK cloudPresenterThreadFunc(LPVOID threadParameter)
{
	_ASSERT(threadParameter != NULL);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D model vertex cloud"));

	{ // Delete cloud-in-memory as soon as possible
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud(
			static_cast<pcl::PointCloud<pcl::PointXYZRGB>*>(threadParameter));

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "object cloud"); cloud->clear();
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "object cloud");
	}

	viewer->setBackgroundColor(0.231, 0.231, 0.231);
	viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}