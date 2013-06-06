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
#include <pcl/common/pca.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

inline pcl::PointXYZ Eigen2PointXYZ(Eigen::Vector3f v) { return pcl::PointXYZ(v[0],v[1],v[2]); }
inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb) { pcl::PointXYZRGB p(rgb[0],rgb[1],rgb[2]); p.x = v[0]; p.y = v[1]; p.z = v[2]; return p; }
inline pcl::PointNormal Eigen2PointNormal(Eigen::Vector3f v, Eigen::Vector3f n) { pcl::PointNormal p; p.x=v[0];p.y=v[1];p.z=v[2];p.normal_x=n[0];p.normal_y=n[1];p.normal_z=n[2]; return p;}
inline float* Eigen2float6(Eigen::Vector3f v, Eigen::Vector3f rgb) { static float buf[6]; buf[0]=v[0];buf[1]=v[1];buf[2]=v[2];buf[3]=rgb[0];buf[4]=rgb[1];buf[5]=rgb[2]; return buf; }
inline Eigen::Matrix<float,6,1> Eigen2Eigen(Eigen::Vector3f v, Eigen::Vector3f rgb) { return (Eigen::Matrix<float,6,1>() << v[0],v[1],v[2],rgb[0],rgb[1],rgb[2]).finished(); }
inline std::vector<Eigen::Matrix<float,6,1> > AsVector(const Eigen::Matrix<float,6,1>& p1, const Eigen::Matrix<float,6,1>& p2) { std::vector<Eigen::Matrix<float,6,1> > v(2); v[0] = p1; v[1] = p2; return v; }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CloudVisualizer
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CloudVisualizer::populatePointCloud(const std::vector<cv::Point3d>& pointcloud, const std::vector<cv::Vec3b>& pointcloud_RGB, pclColoredPointCloud::Ptr& mycloud)
{
	_ASSERT_EXPR(pointcloud.size() == pointcloud_RGB.size() || pointcloud_RGB.size() == 0,
		L"point cloud space and color data vectors must be aligned");

	for (unsigned int i = 0, imax = pointcloud.size(); i < imax; ++i) 
	{
		cv::Point3d src_pt(pointcloud[i]);
		// get the RGB color value for the point
		// check for erroneous coordinates (NaN, Inf, etc.)
		if (src_pt.x != src_pt.x || src_pt.y != src_pt.y || src_pt.z != src_pt.z 
#ifndef WIN32
			||  isnan(src_pt.x) ||  isnan(src_pt.y) ||  isnan(src_pt.z)
#else
			|| _isnan(src_pt.x) || _isnan(src_pt.y) || _isnan(src_pt.z)
#endif
		) continue;
		
		pcl::PointXYZRGB pclp(pointcloud_RGB[i][2], pointcloud_RGB[i][1], pointcloud_RGB[i][0]);
		pclp.x = src_pt.x; pclp.y = src_pt.y; pclp.z = src_pt.z;
		
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
			viewer.removeShape("mesh");

			viewer.removePointCloud("cloud");
			viewer.addPointCloud(me->cloud_to_show, "cloud");
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

		if (me->show_scene == SCENE_SHOW)
		{
			me->show_scene = SCENE_VISIBLE;

			me->_scene_mesh_mutex.lock();

			viewer.removePointCloud("cloud");

			viewer.removeShape("mesh");
			viewer.addPolygonMesh(me->scene_mesh, "mesh");

			me->_scene_mesh_mutex.unlock();
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
	if ((event_.getKeyCode() == 'm' || event_.getKeyCode() == 'M') && event_.keyDown()) me->SwitchSceneMesh();
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

	if (s == -1.0)
	{	
		pcl::PCA<pcl::PointXYZRGB> pca(*cloud_registry.front(), true);
		Eigen::Vector3f eigen_vals = pca.getEigenValues();
		s = std::abs(eigen_vals(2) / eigen_vals(0));
	}

	assert(s > 0.0);

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

	_ASSERT_EXPR(camera_meshes.size() == camera_los.size(), 
		L"Camera meshes vector is unaligned with camera lines one.");

	_camera_data_mutex.unlock();
	
	show_cameras = !cam_data.empty();
}

void CloudVisualizer::SelectCloudToShow(CLOUD_SELECTION_DIRECTION csd)
{
	if (show_scene != SCENE_HIDDEN) return;

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

		case CSD_LAST: break;

		default:
			_ASSERTE("Bad direction to select cloud");
		}
	}

	_cloud_to_show_mutex.lock();

	if (cloud_registry.empty() || csd ==CSD_RESET) 
		cloud_to_show.reset(); else cloud_to_show = cloud_registry.front();

	_cloud_registry_mutex.unlock();
	_cloud_to_show_mutex.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VisualizerListener
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VisualizerListener::update(const MultiCameraPnP &whos_updated) 
{
	RawCloudDataCollection cloud_data;
	cloud_data.push_back(RawCloudData(whos_updated.getPointCloud(),			whos_updated.getPointCloudRGB()));
	cloud_data.push_back(RawCloudData(whos_updated.getPointCloud(false),	whos_updated.getPointCloudRGB(false)));
	LoadClouds(cloud_data);

	std::vector<std::pair<double, cv::Matx34d>> cam_data;
	std::vector<std::pair<int, cv::Matx34d>>	cameras		= whos_updated.getCameras();
	for (std::vector<std::pair<int, cv::Matx34d>>::iterator i = cameras.begin(), iend = cameras.end(); i != iend; ++i)
	{
		cv::Size img_size = whos_updated.get_im_orig(i->first).size();
		cam_data.push_back(std::make_pair(double(img_size.width) / double(img_size.height), i->second));
	}

	LoadCameras(cam_data, Eigen::Vector3f(255, 0, 0));	
}

void VisualizerListener::finish(const MultiCameraPnP &whos_updated)
{
	if (mesh_builder != NULL)
	{
		mesh_builder->build(whos_updated.getPointCloud());

		std::vector<pcl::PolygonMesh> &mesh_segments = mesh_builder->get();
		for (std::vector<pcl::PolygonMesh>::iterator it = mesh_segments.begin(), it_end = mesh_segments.end(); it != it_end; ++it) LoadSceneMesh(*it);
	}
}