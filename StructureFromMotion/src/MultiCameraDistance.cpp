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

#include "MultiCameraDistance.h"
#include "RichFeatureMatcher.h"
#include "OFFeatureMatcher.h"
#include "GPUSURFFeatureMatcher.h"

MultiCameraDistance::MultiCameraDistance(FEATURE_MATCHER_MODE mode, const std::vector<cv::Mat>& imgs_, const std::vector<std::string>& imgs_names_, const cv::Mat &intrinsics, const cv::Mat &distortion_vector) 
	: imgs_names(imgs_names_), features_matched(false), matcher_mode(mode)
{
	CV_Assert(intrinsics.size() == cv::Size(3,3) && intrinsics.type() == CV_64FC1
		&& (distortion_vector.size() >= cv::Size(1,5) || distortion_vector.size() <= cv::Size(1,7)) && distortion_vector.type() == CV_64FC1);

	cv::Size min_size;
	for (unsigned i = 0, imax = imgs_.size(); i < imax; ++i)
		if (min_size.area() == 0 || imgs_[i].size() < min_size) min_size = imgs_[i].size();
	if (min_size.height > min_size.width) std::swap(min_size.width, min_size.height);

	//ensure images are CV_8UC3
	for (unsigned int i=0; i<imgs_.size(); i++) {
		if (imgs_[i].empty()) continue;

		imgs_orig.push_back(cv::Mat_<cv::Vec3b>());
		if (imgs_[i].type() == CV_8UC1) {
			cvtColor(imgs_[i], imgs_orig[i], CV_GRAY2BGR);
		} else if (imgs_[i].type() == CV_32FC3 || imgs_[i].type() == CV_64FC3) {
			imgs_[i].convertTo(imgs_orig[i],CV_8UC3,255.0);
		} else {
			imgs_[i].copyTo(imgs_orig[i]);
		}
				
		if (imgs_orig[i].size() != min_size)
		{
			cv::Size new_size(min_size);
			if (imgs_orig[i].size().height > imgs_orig[i].size().width) std::swap(new_size.width, new_size.height);
			cv::resize(imgs_orig[i], imgs_orig[i], new_size, 0.0, 0.0, cv::INTER_LANCZOS4);
		}

		imgs.push_back(cv::Mat());
		cvtColor(imgs_orig[i],imgs[i], CV_BGR2GRAY);
		
		imgpts.push_back(std::vector<cv::KeyPoint>());
		imgpts_good.push_back(std::vector<cv::KeyPoint>());
	}
		
	double max_w_h = std::max(min_size.height,min_size.width);	

	intrinsics.copyTo(cam_matrix);
	cam_matrix.at<double>(0,0) *= max_w_h;
	cam_matrix.at<double>(1,1) *= max_w_h;
	cam_matrix.at<double>(0,2) *= min_size.width;
	cam_matrix.at<double>(1,2) *= min_size.height;
	
	K = cam_matrix;
	invert(K, Kinv); //get inverse of camera matrix

	distortion_vector.copyTo(distortion_coeff);
	distortion_coeff.convertTo(distcoeff_32f,CV_32FC1);
	K.convertTo(K_32f,CV_32FC1);
}

void MultiCameraDistance::OnlyMatchFeatures()
{
	if(features_matched) return;

	bool precached	= !!(matcher_mode & FEATURE_MATCHER_CACHED);
	bool gpu_use	= !!(matcher_mode & FEATURE_MATCHER_USE_GPU);

	cv::Ptr<IFeatureMatcher> feature_matcher;
	if (!!(matcher_mode & FEATURE_MATCHER_RICH)) {
		feature_matcher = new RichFeatureMatcher(imgs, imgpts, precached); // TODO: GPU is appliable here
	} else if (!!(matcher_mode & FEATURE_MATCHER_FAST)) {
		feature_matcher = new OFFeatureMatcher(imgs, imgpts, precached, gpu_use);
	} else _ASSERTE("Bad flag setting (No matcher type specified).");

	int loop1_top = imgs.size() - 1, loop2_top = imgs.size();
	int frame_num_i = 0;
	
#pragma omp parallel for
	for (frame_num_i = 0; frame_num_i < loop1_top; frame_num_i++) {
		for (int frame_num_j = frame_num_i + 1; frame_num_j < loop2_top; frame_num_j++)
		{
			std::cout << "------------ Match " << imgs_names[frame_num_i] << ","<<imgs_names[frame_num_j]<<" ------------\n";
			std::vector<cv::DMatch> matches_tmp;
			feature_matcher->MatchFeatures(frame_num_i,frame_num_j,&matches_tmp);
			if (matches_tmp.size() == 0) continue; // No matches found. Do not add to matches matrix

			std::vector<cv::DMatch> matches_tmp_flip = FlipMatches(matches_tmp);
			matches_matrix[std::make_pair(frame_num_j,frame_num_i)] = matches_tmp_flip;
			matches_matrix[std::make_pair(frame_num_i,frame_num_j)] = matches_tmp;
		}
	}

	features_matched = true;
}

void MultiCameraDistance::GetRGBForPointCloud(const std::vector<struct CloudPoint>& _pcloud, std::vector<cv::Vec3b>& RGBforCloud) const
{
	RGBforCloud.resize(_pcloud.size());
	for (unsigned i=0; i<_pcloud.size(); ++i) 
	{
		std::vector<cv::Vec3b> point_colors;
		for(unsigned good_view = 0; good_view < imgs_orig.size(); ++good_view) 
		{
			int pt_idx = _pcloud[i].imgpt_for_img[good_view];
			if(pt_idx != -1) point_colors.push_back(imgs_orig[good_view].at<cv::Vec3b>(imgpts[good_view][pt_idx].pt));

/*			if(pt_idx >= imgpts[good_view].size()) 
			{
				std::cerr << "BUG: point id:" << pt_idx << " should not exist for img #" << good_view << " which has only " << imgpts[good_view].size() << std::endl;
				continue;
			}	*/		
		}

		cv::Scalar res_color	= !point_colors.empty() ? cv::mean(point_colors) : cv::Scalar(255,0,0);
		RGBforCloud[i]			= cv::Vec3b(res_color[0],res_color[1],res_color[2]);
	}
}

void MultiCameraDistance::LoadFeaturesCache(const std::vector<cv::Mat> &features_cache)
{
	assert(!imgs.empty() && imgs.size() == features_cache.size());

	imgpts.clear(); imgpts.resize(imgs.size());
	for (size_t i = 0, imax = features_cache.size(); i < imax; ++i)
	{
		const cv::Mat				&features_src = features_cache[i];
		std::vector<cv::KeyPoint>	&features_dst = imgpts[i];

		assert(features_src.type() == CV_MAKETYPE(CV_32F, 6));

		features_dst.resize(features_src.size().area());
		for (int r = 0, idx = 0; r < features_src.rows; ++r)
		{
			for (int c = 0; c < features_src.cols; ++c)
			{
				const cv::Vec<float, 6> &key_pt = features_src.at<cv::Vec<float, 6>>(r, c);
				new (&features_dst[idx]) cv::KeyPoint(
						key_pt.val[0], key_pt.val[1], key_pt.val[2],
						key_pt.val[3], key_pt.val[4], static_cast<int>(key_pt.val[5])
					);
				++idx;
			}
		}
	}
}

void MultiCameraDistance::ObtainFeaturesCache(std::vector<cv::Mat> &features_cache)
{
	features_cache.clear();

	assert(!imgs.empty() || features_matched);
	features_cache.resize(imgs.size());

	for (size_t i = 0, imax = features_cache.size(); i < imax; ++i)
	{
		std::vector<cv::KeyPoint>	&features_src = imgpts[i];
		cv::Mat						*features_dst = new (&features_cache[i]) cv::Mat(1, features_src.size(), CV_MAKETYPE(CV_32F, 6));

		for (size_t j = 0, jmax = features_src.size(); j < jmax; ++j)
		{
			const cv::KeyPoint &key_pt = features_src[j];
			new (features_dst->ptr<cv::Vec<float, 6>>(1, j)) cv::Vec<float, 6> (
					key_pt.pt.x, key_pt.pt.y, key_pt.size, 
					key_pt.angle, key_pt.response, key_pt.octave
				);
		}
	}
}