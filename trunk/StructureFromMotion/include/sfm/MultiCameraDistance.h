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
	//std::vector<std::vector<cv::KeyPoint>> fullpts;
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

	std::vector<CloudPoint>			pcloud;
	mutable std::vector<cv::Vec3b>	pointCloudRGB;
	std::vector<cv::KeyPoint>		correspImg1Pt;
		
	mutable bool features_matched;

	void GetRGBForPointCloud(const std::vector<struct CloudPoint>& pcloud, std::vector<cv::Vec3b>& RGBforCloud) const;
	std::vector<cv::Vec3b>& _pointCloudRGBInit() const { if (pointCloudRGB.empty()) GetRGBForPointCloud(pcloud,pointCloudRGB); return pointCloudRGB; }

	bool use_gpu() { return !!(matcher_mode & FEATURE_MATCHER_USE_GPU); };

public:

	enum FEATURE_MATCHER_MODE : unsigned {
		FEATURE_MATCHER_UNKNOWN = 0x00,
		FEATURE_MATCHER_FAST	= 0x02,
		FEATURE_MATCHER_RICH	= 0x04,
		FEATURE_MATCHER_CACHED	= 0x01,
		FEATURE_MATCHER_USE_GPU	= 0x10
	} matcher_mode;
	
	//MultiCameraDistance(IDataProvider &input_data)
	MultiCameraDistance(FEATURE_MATCHER_MODE mode, const std::vector<cv::Mat>& imgs_, const std::vector<std::string>& imgs_names_, const cv::Mat &intrinsics, const cv::Mat &distortion_vector);	
	
	virtual void OnlyMatchFeatures();	

	void LoadFeaturesCache(const std::vector<std::vector<cv::KeyPoint>> &features_cache);
	void ObtainFeaturesCache(std::vector<std::vector<cv::KeyPoint>> &features_cache);

	const cv::Mat& get_im_orig(int frame_num)			const { return imgs_orig[frame_num]; }
	const std::vector<cv::KeyPoint>& getcorrespImg1Pt()	const { return correspImg1Pt; }

	std::vector<cv::Point3d>	getPointCloud()		const { return CloudPointsToPoints(pcloud); }
	std::vector<cv::Vec3b>		getPointCloudRGB()	const { return _pointCloudRGBInit(); } 
	
	cv::Matx33d	getCameraMatrix() const { return cam_matrix; }
	std::vector<std::pair<int, cv::Matx34d>> getCameras(bool intrinsic = false)	const	
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
