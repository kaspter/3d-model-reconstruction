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

#include "Common.h"
#include "MultiCameraDistance.h"

class MultiCameraPnP : public MultiCameraDistance {
public:
	class UpdateListener
	{
	public:
		/*const std::vector<cv::Point3d> &pcld_a, const std::vector<cv::Vec3b> &pcld_a_rgb, 
			const std::vector<cv::Point3d> &pcld_b, const std::vector<cv::Vec3b> &pcld_b_rgb, const std::vector<std::pair<double, cv::Matx34d>> &cameras*/
		virtual void update(const MultiCameraPnP &) = 0;
		virtual void finish(const MultiCameraPnP &) = 0;
	};

private:

	std::vector<CloudPoint> pointcloud_beforeBA;
	std::vector<cv::Vec3b>	pointCloudRGB_beforeBA;
	
	int m_first_view, m_second_view;
	std::set<int> done_views, good_views;

	void PruneMatchesBasedOnF();
	void AdjustCurrentBundle();
	void GetBaseLineTriangulation();
	void Find2D3DCorrespondences(int working_view, std::vector<cv::Point3f>& ppcloud, std::vector<cv::Point2f>& imgPoints);
	bool FindPoseEstimation(int working_view, cv::Mat_<double>& rvec, cv::Mat_<double>& t, 
		cv::Mat_<double>& R, std::vector<cv::Point3f> ppcloud, std::vector<cv::Point2f> imgPoints);

	bool TriangulatePointsBetweenViews(int working_view, int second_view,
		std::vector<struct CloudPoint>& new_triangulated, std::vector<int>& add_to_cloud);

	int FindHomographyInliers2Views(int vi, int vj);

	std::vector<UpdateListener*> listeners;

    void update(bool finish = false) 
	{ 
		for (int i = 0; i < listeners.size(); i++) 
			if (finish) listeners[i]->finish(*this); else listeners[i]->update(*this);
	}

public:
	MultiCameraPnP(FEATURE_MATCHER_MODE mode, const std::vector<cv::Mat>& imgs_, const std::vector<std::string>& imgs_names_, const cv::Mat &intrinsics, const cv::Mat distortion_vector)
		: MultiCameraDistance(mode, imgs_,imgs_names_,intrinsics, distortion_vector) { /* empty */ }

	virtual void RecoverDepthFromImages();

	std::vector<cv::Point3d> getPointCloud(bool adjusted = true) const
	{ 
		return adjusted ? MultiCameraDistance::getPointCloud() 
			: CloudPointsToPoints(pointcloud_beforeBA); 
	}
	const std::vector<cv::Vec3b> getPointCloudRGB(bool adjusted = true) const
	{ 
		return adjusted ? _pointCloudRGBInit() : pointCloudRGB_beforeBA; 
	}

	void attach(UpdateListener *sul) { listeners.push_back(sul); }
};
