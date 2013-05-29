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

#pragma once

#include <vector>

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudVisualizer {
	typedef pcl::PointCloud<pcl::PointXYZRGB>								pclColoredPointCloud;
	typedef std::pair<std::string,pcl::PolygonMesh>							pclCameraRepresntation;
	typedef std::pair<std::string,std::vector<Eigen::Matrix<float,6,1>>>	pclColoredLine;

	std::deque<pclColoredPointCloud::Ptr>	cloud_registry; boost::recursive_mutex _cloud_registry_mutex;
	pclColoredPointCloud::Ptr				cloud_to_show;	boost::recursive_mutex _cloud_to_show_mutex;

	std::deque<pclCameraRepresntation>		camera_meshes;	boost::recursive_mutex _camera_data_mutex;
	std::deque<pclColoredLine>				camera_los;
	volatile bool							show_cameras;

	pcl::PolygonMesh scene_mesh; boost::recursive_mutex _scene_mesh_mutex;
	volatile enum SCENE_STATE : unsigned
	{
		SCENE_HIDDEN,
		SCENE_VISIBLE,
		SCENE_SHOW
	} show_scene;

	static void populatePointCloud(const std::vector<cv::Point3d>& pointcloud, const std::vector<cv::Vec3b>& pointcloud_RGB, pclColoredPointCloud::Ptr& mycloud);

	boost::scoped_ptr<boost::thread> visualizer_thread;
	static void visualizerCallback(CloudVisualizer* me, void* dummy);

protected:
	static void KeyboardEventCallback (const pcl::visualization::KeyboardEvent& event_, void *void_this);

public:
	enum CLOUD_SELECTION_DIRECTION {
		CSD_BACKWARD	= -1,
		CSD_INDIFFERENT =  0,
		CSD_FORWARD		=  1,
		CSD_LAST		=  CSD_BACKWARD & ~CSD_FORWARD,
		CSD_RESET		= -CSD_LAST
	};

	typedef std::pair<const std::vector<cv::Point3d>, const std::vector<cv::Vec3b>> RawCloudData;
	typedef std::vector<RawCloudData> RawCloudDataCollection;

	CloudVisualizer() : show_cameras(false), show_scene(SCENE_HIDDEN) { /* empty */  }

	void LoadClouds(const RawCloudDataCollection &cloud_data);
	void LoadCameras(const std::vector<std::pair<double, cv::Matx34d>> cam_data, const Eigen::Vector3f &color, double s = 0.01);
	void LoadSceneMesh(const pcl::PolygonMesh scene_polygons) 
	{ 
		_scene_mesh_mutex.lock();
		scene_mesh = scene_polygons; 
		SwitchSceneMesh();
		_scene_mesh_mutex.unlock();
	}

	void SelectCloudToShow(CLOUD_SELECTION_DIRECTION csd = CSD_INDIFFERENT);
	bool SwitchSceneMesh()
	{
		_scene_mesh_mutex.lock();
		show_scene = !scene_mesh.polygons.empty() && show_scene == SCENE_HIDDEN	? SCENE_SHOW : SCENE_HIDDEN; 
		_scene_mesh_mutex.unlock();

		SelectCloudToShow(!!show_scene ? CSD_RESET : CSD_LAST);
		return !!show_scene;
	}

	void RunVisualizationThread()		{ if (visualizer_thread.get() != NULL) return; visualizer_thread.reset(new boost::thread(visualizerCallback, this, (void*)NULL)); }
	void WaitForVisualizationThread()	{ if (visualizer_thread.get() == NULL) return; visualizer_thread->join(); visualizer_thread.reset(); }
};

#include <sfm/MultiCameraPnP.h>

#include "DataExport.h"

class VisualizerListener : public CloudVisualizer, public MultiCameraPnP::UpdateListener {

	SceneMesh *mesh_builder;

public:
	VisualizerListener(SceneMesh *builder = NULL) : mesh_builder(builder) { /* empty */ }
	void		setBuilder(SceneMesh *builder)	{ mesh_builder = builder; }
	SceneMesh*	getBuilder()					{ return mesh_builder; }

	void update(const MultiCameraPnP &whos_updated);
	void finish(const MultiCameraPnP &whos_finished);
};
