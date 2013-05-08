#include "stdafx.h"

#include "CloudData.h"

DWORD CALLBACK cloudPresenterThreadFunc(LPVOID threadParameter)
{
	_ASSERT(threadParameter != NULL);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D model vertex cloud"));

	{ // Delete cloud-in-memory as soon as possible
		boost::shared_ptr<CloudData> cloudData(static_cast<CloudData*>(threadParameter));

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudData->_cloud_ptr);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloudData->_cloud_ptr, rgb, "object cloud"); cloudData->_cloud_ptr->clear();
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