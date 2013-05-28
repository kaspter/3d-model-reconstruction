/*****************************************************************************
*   ExploringSfMWithOpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include "Visualization.h"

#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Eigen>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

inline pcl::PointXYZ Eigen2PointXYZ(Eigen::Vector3f v) { return pcl::PointXYZ(v[0],v[1],v[2]); }
inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb) { pcl::PointXYZRGB p(rgb[0],rgb[1],rgb[2]); p.x = v[0]; p.y = v[1]; p.z = v[2]; return p; }
inline pcl::PointNormal Eigen2PointNormal(Eigen::Vector3f v, Eigen::Vector3f n) { pcl::PointNormal p; p.x=v[0];p.y=v[1];p.z=v[2];p.normal_x=n[0];p.normal_y=n[1];p.normal_z=n[2]; return p;}
inline float* Eigen2float6(Eigen::Vector3f v, Eigen::Vector3f rgb) { static float buf[6]; buf[0]=v[0];buf[1]=v[1];buf[2]=v[2];buf[3]=rgb[0];buf[4]=rgb[1];buf[5]=rgb[2]; return buf; }
inline Eigen::Matrix<float,6,1> Eigen2Eigen(Eigen::Vector3f v, Eigen::Vector3f rgb) { return (Eigen::Matrix<float,6,1>() << v[0],v[1],v[2],rgb[0],rgb[1],rgb[2]).finished(); }
inline std::vector<Eigen::Matrix<float,6,1> > AsVector(const Eigen::Matrix<float,6,1>& p1, const Eigen::Matrix<float,6,1>& p2) { std::vector<Eigen::Matrix<float,6,1> > v(2); v[0] = p1; v[1] = p2; return v; }

//void SORFilter() 
//{
//	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
//	
//	std::cout << "Cloud before SOR filtering: " << cloud->width * cloud->height << " data points" << std::endl;
//	
//	// Create the filtering object
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//	sor.setInputCloud (cloud);
//	sor.setMeanK (50);
//	sor.setStddevMulThresh (1.0);
//	sor.filter (*cloud_filtered);
//	
//	copyPointCloud(*cloud_filtered,*cloud);
//	copyPointCloud(*cloud,*orig_cloud);
//
//	std::cout << "Cloud after SOR filtering: " << cloud->width * cloud->height << " data points " << std::endl;
//}	

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CloudVisualizer
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CloudVisualizer::populatePointCloud(const std::vector<cv::Point3d>& pointcloud, const std::vector<cv::Vec3b>& pointcloud_RGB, pclColoredPointCloud::Ptr& mycloud)
{
	_ASSERT_EXPR(pointcloud.size() == pointcloud_RGB.size() || pointcloud_RGB.size() == 0,
		L"point cloud space and color data vectors must be aligned");

	for (unsigned int i = 0, imax = pointcloud.size(); i < imax; ++i) 
	{
		// get the RGB color value for the point
		// check for erroneous coordinates (NaN, Inf, etc.)
		if (pointcloud[i].x != pointcloud[i].x || 
			pointcloud[i].y != pointcloud[i].y || 
			pointcloud[i].z != pointcloud[i].z || 
#ifndef WIN32
			isnan(pointcloud[i].x) ||
			isnan(pointcloud[i].y) || 
			isnan(pointcloud[i].z) ||
#else
			_isnan(pointcloud[i].x) ||
			_isnan(pointcloud[i].y) || 
			_isnan(pointcloud[i].z) ||
#endif
			false
		) {
			continue;
		}
		
		pcl::PointXYZRGB pclp(pointcloud_RGB[i][2], pointcloud_RGB[i][1], pointcloud_RGB[i][0]);
		
		// 3D coordinates
		pclp.x = pointcloud[i].x;
		pclp.y = pointcloud[i].y;
		pclp.z = pointcloud[i].z;
		
		mycloud->push_back(pclp);
	}
	
	mycloud->width  = static_cast<uint32_t>(mycloud->points.size());
	mycloud->height = static_cast<uint32_t>(mycloud->width != 0);
}

void CloudVisualizer::visualizerCallback(CloudVisualizer* me, void* dummy)
{
	pcl::visualization::PCLVisualizer viewer("3D Model Space");   

	void *objects_essential[] = {me, &viewer};
	viewer.registerKeyboardCallback(KeyboardEventCallback, objects_essential);
	
    while (!viewer.wasStopped())
    {
		me->_cloud_to_show_mutex.lock();
		if (me->cloud_to_show.get() != NULL) 
		{
			viewer.removePointCloud("pcloud");
			viewer.addPointCloud(me->cloud_to_show, "pcloud");
			me->cloud_to_show.reset();
		}
		me->_cloud_to_show_mutex.unlock();

		if (me->show_cameras)
		{
			me->show_cameras = false;

			me->_camera_data_mutex.lock();

			for (unsigned i = 0, imax = me->camera_meshes.size(); i < imax; ++i)
			{
				viewer.removeShape(me->camera_meshes[i].first);
				viewer.addPolygonMesh(me->camera_meshes[i].second, me->camera_meshes[i].first);

				std::vector<Eigen::Matrix<float,6,1> > oneline = me->camera_los[i].second;
				pcl::PointXYZRGB A(oneline[0][3],oneline[0][4],oneline[0][5]),
								 B(oneline[1][3],oneline[1][4],oneline[1][5]);
				for(int j = 0; j < 3; ++j) { A.data[j] = oneline[0][j]; B.data[j] = oneline[1][j]; }
				viewer.removeShape(me->camera_los[i].first);
				viewer.addLine<pcl::PointXYZRGB,pcl::PointXYZRGB>(A, B, me->camera_los[i].first);
			}

			me->_camera_data_mutex.unlock();
		}

		viewer.spinOnce(50);
    }
}

void CloudVisualizer::KeyboardEventCallback (const pcl::visualization::KeyboardEvent& event_, void *void_this)
{
	_ASSERT_EXPR(void_this != NULL, L"It seems to be parameters vector is corrupted");
	CloudVisualizer *me = reinterpret_cast<CloudVisualizer*>(*(uintptr_t*)void_this);

	if ((event_.getKeyCode() == '.' || event_.getKeyCode() == '>') && event_.keyDown()) me->SelectCloudToShow(CSD_FORWARD);
	if ((event_.getKeyCode() == ',' || event_.getKeyCode() == '<') && event_.keyDown()) me->SelectCloudToShow(CSD_BACKWARD);
}	

void CloudVisualizer::LoadClouds(const RawCloudDataCollection &cloud_data)
{
	if (cloud_data.empty()) return;
	
	_cloud_registry_mutex.lock();

	cloud_registry.clear();
	for (RawCloudDataCollection::const_iterator i = cloud_data.begin(), iend = cloud_data.end(); i != iend; ++i)
	{
		pclColoredPointCloud::Ptr new_cloud(new pclColoredPointCloud);
		populatePointCloud(i->first, i->second, new_cloud);
		cloud_registry.push_front(new_cloud);
	}
	SelectCloudToShow(CSD_BACKWARD);

	_cloud_registry_mutex.unlock();
}

#define POLYGON_GROUP_SIZE 3
void CloudVisualizer::LoadCameras(const std::vector<std::pair<double, cv::Matx34d>> cam_data, const Eigen::Vector3f &color, double s)
{
	static const int polygon_idx[][POLYGON_GROUP_SIZE] = { {0,1,2}, {0,3,1}, {0,4,3}, {0,2,4}, {3,1,4}, {2,4,1} };

	_camera_data_mutex.lock();

	camera_meshes.clear(); camera_los.clear();
	for(unsigned i = 0, imax = cam_data.size(); i < imax; ++i) 
	{
		cv::Matx34d P(cam_data[i].second);
		if (cv::countNonZero(P) == 0) continue;

		Eigen::Matrix<float,3,3,Eigen::RowMajor> R(cv::Matx33f(
				P(0,0), P(0,1), P(0,2),
				P(1,0), P(1,1), P(1,2),
				P(2,0), P(2,1), P(2,2)
			).val);
		Eigen::Vector3f t(P(0,3), P(1,3), P(2,3)); t = -R.transpose() * t;
		Eigen::Vector3f vforward =  R.row(2).normalized() * s;

		pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
		{
			Eigen::Vector3f apex = t;
			Eigen::Vector3f vright	 =  R.row(0).normalized() * s / 2.0;
			Eigen::Vector3f vup		 = -R.row(1).normalized() * s / 2.0;

			double aspect = cam_data[i].first;
			if (aspect > 1.0) vright *= aspect; else vup /= aspect;

			mesh_cld.push_back(Eigen2PointXYZRGB(apex, color)); apex += vforward;
			mesh_cld.push_back(Eigen2PointXYZRGB(apex + vright + vup, color));
			mesh_cld.push_back(Eigen2PointXYZRGB(apex + vright - vup, color));
			mesh_cld.push_back(Eigen2PointXYZRGB(apex - vright + vup, color));
			mesh_cld.push_back(Eigen2PointXYZRGB(apex - vright - vup, color));
		}

		pcl::PolygonMesh pm; pm.polygons.resize(6); 
		for(int j = 0; j < _countof(polygon_idx); ++j)
		{
			for(int v = 0; v < POLYGON_GROUP_SIZE; ++v)
				pm.polygons[j].vertices.push_back(polygon_idx[j][v]);
		}
		pcl::toROSMsg(mesh_cld,pm.cloud);

		std::stringstream ss; ss << "camera #" << i;
		camera_meshes.push_back(std::make_pair(ss.str(),pm));

		ss << " line";
		camera_los.push_back(std::make_pair(ss.str(),
			AsVector(Eigen2Eigen(t, color),Eigen2Eigen(t + vforward * 3.0, color))
			));
	}

	_ASSERT_EXPR(camera_meshes.size() == camera_los.size(), L"Camera meshes vector is unaligned with camera lines one.");

	_camera_data_mutex.unlock();
	
	show_cameras = !cam_data.empty();
}

void CloudVisualizer::SelectCloudToShow(CLOUD_SELECTION_DIRECTION csd)
{
	_cloud_registry_mutex.lock();

	if (!cloud_registry.empty())
	{
		switch(csd)
		{
		case CSD_BACKWARD:
			cloud_registry.push_front(cloud_registry.back());
			cloud_registry.pop_back();
			break;
		case CSD_FORWARD:
			cloud_registry.push_back(cloud_registry.front());
			cloud_registry.pop_front();
			break;

		case CSD_INDIFFERENT:
			std::srand(boost::get_system_time().time_of_day().fractional_seconds());
			cloud_registry.push_front(*cloud_registry.erase(
				cloud_registry.begin() + (std::rand() % cloud_registry.size())
				));
			break;

		default:
			_ASSERTE("Bad direction to select cloud");
		}
	}

	_cloud_to_show_mutex.lock();

	if (cloud_registry.empty()) cloud_to_show.reset(); else cloud_to_show = cloud_registry.front();

	_cloud_registry_mutex.unlock();
	_cloud_to_show_mutex.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VisualizerListener
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VisualizerListener::update(const std::vector<cv::Point3d> &pcld_a, const std::vector<cv::Vec3b> &pcld_a_rgb, 
								const std::vector<cv::Point3d> &pcld_b, const std::vector<cv::Vec3b> &pcld_b_rgb, 
								const std::vector<std::pair<double, cv::Matx34d>> &cameras
) {
	RawCloudDataCollection cloud_data;
	cloud_data.push_back(RawCloudData(pcld_a, pcld_a_rgb));
	cloud_data.push_back(RawCloudData(pcld_b, pcld_b_rgb));

	LoadClouds(cloud_data);
	LoadCameras(cameras, Eigen::Vector3f(255, 0, 0), 0.2);	

}



