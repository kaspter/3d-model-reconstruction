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
#include "SfMUpdateListener.h"

class MultiCameraPnP : public MultiCameraDistance {
public:
	class UpdateListener
	{
	public:
		virtual void update(const std::vector<cv::Point3d> &pcld, const std::vector<cv::Vec3b> &pcldrgb, 
			const std::vector<cv::Point3d> &pcld_alternate, const std::vector<cv::Vec3b> &pcldrgb_alternate, const std::vector<cv::Matx34d> &cameras) = 0;
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

	std::vector <UpdateListener*> listeners;

    void update()
    {
        for (int i = 0; i < listeners.size(); i++)
			listeners[i]->update(getPointCloud(), _pointCloudRGBInit(), 
				getPointCloud(false), pointCloudRGB_beforeBA, getCameras());
    }

public:
	MultiCameraPnP(const std::vector<cv::Mat>& imgs_, const std::vector<std::string>& imgs_names_, const std::string& imgs_path_)
		: MultiCameraDistance(imgs_,imgs_names_,imgs_path_) { /* empty */ }

	virtual void RecoverDepthFromImages();

	std::vector<cv::Point3d> getPointCloud(bool adjusted = true)
	{ 
		return adjusted ? MultiCameraDistance::getPointCloud() 
			: CloudPointsToPoints(pointcloud_beforeBA); 
	}
	const std::vector<cv::Vec3b> getPointCloudRGB(bool adjusted = true)
	{ 
		return adjusted ? _pointCloudRGBInit() : pointCloudRGB_beforeBA; 
	}

	void attach(UpdateListener *sul) { listeners.push_back(sul); }
};
