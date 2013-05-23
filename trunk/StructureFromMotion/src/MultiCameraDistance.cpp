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

MultiCameraDistance::MultiCameraDistance(
	const std::vector<cv::Mat>& imgs_, 
	const std::vector<std::string>& imgs_names_, 
	const std::string& imgs_path_):
imgs_names(imgs_names_),features_matched(false),use_rich_features(true),use_gpu(true)
{
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
		
	//load calibration matrix
	cv::FileStorage fs;
	double max_w_h = std::max(min_size.height,min_size.width);
	if(fs.open(imgs_path_+ "\\out_camera_data.xml", cv::FileStorage::READ | cv::FileStorage::FORMAT_XML, "utf8")) {
		fs["camera_distortion"]>>distortion_coeff;
		fs["camera_intrinsics"]>>cam_matrix;
		fs.release();
	} else {
		//no calibration matrix file - mockup calibration	
		distortion_coeff	= cv::Mat_<double>::zeros(1,4);
		cam_matrix			= cv::Mat_<double>(3,3) <<	1.0,	0.0,	0.5, 
														0.0,	1.0,	0.5, 
														0.0,	0.0,	1.0;
	}
	
	cam_matrix.at<double>(0,0) *= max_w_h;
	cam_matrix.at<double>(1,1) *= max_w_h;
	cam_matrix.at<double>(0,2) *= min_size.width;
	cam_matrix.at<double>(1,2) *= min_size.height;
	
	K = cam_matrix;
	invert(K, Kinv); //get inverse of camera matrix

	distortion_coeff.convertTo(distcoeff_32f,CV_32FC1);
	K.convertTo(K_32f,CV_32FC1);
}

void MultiCameraDistance::OnlyMatchFeatures(int strategy) 
{
	if(features_matched) return;
	
	if (use_rich_features) {
		if (use_gpu) {
			feature_matcher = new GPUSURFFeatureMatcher(imgs,imgpts);
		} else {
			feature_matcher = new RichFeatureMatcher(imgs,imgpts);
		}
	} else {
		feature_matcher = new OFFeatureMatcher(use_gpu,imgs,imgpts);
	}

	if(strategy & STRATEGY_USE_OPTICAL_FLOW)
		use_rich_features = false;

	int loop1_top = imgs.size() - 1, loop2_top = imgs.size();
	int frame_num_i = 0;
	//#pragma omp parallel for schedule(dynamic)
	
	//if (use_rich_features) {
	//	for (frame_num_i = 0; frame_num_i < loop1_top; frame_num_i++) {
	//		for (int frame_num_j = frame_num_i + 1; frame_num_j < loop2_top; frame_num_j++)
	//		{
	//			std::vector<cv::KeyPoint> fp,fp1;
	//			std::cout << "------------ Match " << imgs_names[frame_num_i] << ","<<imgs_names[frame_num_j]<<" ------------\n";
	//			std::vector<cv::DMatch> matches_tmp;
	//			feature_matcher->MatchFeatures(frame_num_i,frame_num_j,&matches_tmp);
	//			
	//			//#pragma omp critical
	//			{
	//				matches_matrix[std::make_pair(frame_num_i,frame_num_j)] = matches_tmp;
	//			}
	//		}
	//	}
	//} else {
#pragma omp parallel for
	for (frame_num_i = 0; frame_num_i < loop1_top; frame_num_i++) {
		for (int frame_num_j = frame_num_i + 1; frame_num_j < loop2_top; frame_num_j++)
		{
			std::cout << "------------ Match " << imgs_names[frame_num_i] << ","<<imgs_names[frame_num_j]<<" ------------\n";
			std::vector<cv::DMatch> matches_tmp;
			feature_matcher->MatchFeatures(frame_num_i,frame_num_j,&matches_tmp);
			if (matches_tmp.size() == 0) continue; // No matches. Do not add to matches matrix

			std::vector<cv::DMatch> matches_tmp_flip = FlipMatches(matches_tmp);
			matches_matrix[std::make_pair(frame_num_j,frame_num_i)] = matches_tmp_flip;
			matches_matrix[std::make_pair(frame_num_i,frame_num_j)] = matches_tmp;
		}
	}
	//}

	//for (unsigned i = 0, imax = imgpts.size(); i < imax; ++i)
	//{
	//	std::vector<cv::KeyPoint> &_keypoints = imgpts[i];
	//	if (_keypoints.size() == 0) continue;
	//	
	//	std::vector<cv::Point2f> _2d_points;
	//	KeyPointsToPoints(_keypoints, _2d_points);
	//	cv::undistortPoints(_2d_points, _2d_points, K, distortion_coeff);
	//	for (unsigned j = 0, jmax = _keypoints.size(); j < jmax; ++j) _keypoints[j].pt = _2d_points[j];
	//}

	features_matched = true;
}

void MultiCameraDistance::GetRGBForPointCloud(const std::vector<struct CloudPoint>& _pcloud, std::vector<cv::Vec3b>& RGBforCloud) 
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
