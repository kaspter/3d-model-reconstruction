#pragma once

struct CloudData
{
	const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>	_cloud_ptr;
	const std::vector<cv::Matx34d>								_camera_data;

	CloudData(pcl::PointCloud<pcl::PointXYZRGB> *cloud_ptr, const std::vector<cv::Matx34d> &camera_data)
		: _cloud_ptr(cloud_ptr), _camera_data(camera_data) { }
};