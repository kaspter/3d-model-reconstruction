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

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudVisualizer {
	typedef pcl::PointCloud<pcl::PointXYZRGB> pclColoredPointCloud;

	std::deque<pclColoredPointCloud::Ptr>	cloud_registry; boost::recursive_mutex _cloud_registry_mutex;
	pclColoredPointCloud::Ptr				cloud_to_show;	boost::recursive_mutex _cloud_to_show_mutex;

	static void populatePointCloud(const std::vector<cv::Point3d>& pointcloud, const std::vector<cv::Vec3b>& pointcloud_RGB, pclColoredPointCloud::Ptr& mycloud);

	boost::scoped_ptr<boost::thread> visualizer_thread;
	static void visualizerCallback(CloudVisualizer* me, void* dummy);

protected:
	static void KeyboardEventCallback (const pcl::visualization::KeyboardEvent& event_, void *void_this);

public:
	enum CLOUD_SELECTION_DIRECTION {
		CSD_BACKWARD	= -1,
		CSD_INDIFFERENT =  0,
		CSD_FORWARD		=  1
	};

	typedef std::pair<const std::vector<cv::Point3d>&, const std::vector<cv::Vec3b>&> RawCloudData;
	typedef std::vector<RawCloudData> RawCloudDataCollection;

	void LoadClouds(const RawCloudDataCollection &cloud_data);
	void SelectCloudToShow(CLOUD_SELECTION_DIRECTION csd = CSD_INDIFFERENT);

	void RunVisualizationThread()		{ if (visualizer_thread.get() != NULL) return; visualizer_thread.reset(new boost::thread(visualizerCallback, this, (void*)NULL)); }
	void WaitForVisualizationThread()	{ if (visualizer_thread.get() == NULL) return; visualizer_thread->join(); visualizer_thread.reset(); }
};

#include <sfm/MultiCameraPnP.h>

class VisualizerListener : public CloudVisualizer, public MultiCameraPnP::UpdateListener {
public:
	void update(const std::vector<cv::Point3d> &pcld_a, const std::vector<cv::Vec3b> &pcld_a_rgb, 
				const std::vector<cv::Point3d> &pcld_b, const std::vector<cv::Vec3b> &pcld_b_rgb, 
				const std::vector<cv::Matx34d> &cameras);
};
