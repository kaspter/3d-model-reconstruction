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

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

#include "IDistance.h"
#include "Triangulation.h"
#include "IFeatureMatcher.h"
#include "FindCameraMatrices.h"


class MultiCameraDistance  : public IDistance {	
protected:
	std::vector<std::vector<cv::KeyPoint>> imgpts;
	std::vector<std::vector<cv::KeyPoint>> fullpts;
	std::vector<std::vector<cv::KeyPoint>> imgpts_good;

	std::map<std::pair<int,int> ,std::vector<cv::DMatch>> matches_matrix;
	
	std::vector<cv::Mat>				imgs;
	std::vector<std::string>			imgs_names;
	std::vector<cv::Mat_<cv::Vec3b>>	imgs_orig;
	
	std::map<int,cv::Matx34d> Pmats;

	cv::Mat_<double> K;
	cv::Mat_<double> Kinv;
	
	cv::Mat cam_matrix, distortion_coeff;
	cv::Mat distcoeff_32f; 
	cv::Mat K_32f;

	std::vector<CloudPoint>		pcloud;
	std::vector<cv::Vec3b>		pointCloudRGB;
	std::vector<cv::KeyPoint>	correspImg1Pt;
	
	cv::Ptr<IFeatureMatcher>	feature_matcher;
	
	mutable bool features_matched;

protected:	
	void GetRGBForPointCloud(const std::vector<struct CloudPoint>& pcloud, std::vector<cv::Vec3b>& RGBforCloud);
	std::vector<cv::Vec3b>& _pointCloudRGBInit() { if (pointCloudRGB.empty()) GetRGBForPointCloud(pcloud,pointCloudRGB); return pointCloudRGB; }
public:
	bool use_rich_features;
	bool use_gpu;
	
	MultiCameraDistance(const std::vector<cv::Mat>& imgs_, const std::vector<std::string>& imgs_names_, const std::string& imgs_path_);	
	
	virtual void OnlyMatchFeatures(int strategy = STRATEGY_USE_FEATURE_MATCH);	

	const cv::Mat& get_im_orig(int frame_num)			{ return imgs_orig[frame_num]; }
	const std::vector<cv::KeyPoint>& getcorrespImg1Pt() { return correspImg1Pt; }

	std::vector<cv::Point3d>	getPointCloud()		{ return CloudPointsToPoints(pcloud); }
	std::vector<cv::Vec3b>		getPointCloudRGB()	{ return _pointCloudRGBInit(); } 
	
	std::vector<std::pair<int, cv::Matx34d>> getCameras(bool intrinsic = false)		
	{ 
		std::vector<std::pair<int, cv::Matx34d>> v; 
		for(std::map<int ,cv::Matx34d>::const_iterator it = Pmats.begin(); it != Pmats.end(); ++it)
		{
			if (it->second != cv::Matx34d::zeros())
			{
				cv::Mat P(it->second); if (intrinsic)  P = K * P;
				v.push_back(std::make_pair(it->first, P));
			}
		}

		return v;
    }
};
