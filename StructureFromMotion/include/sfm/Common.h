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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <iostream>
#include <list>
#include <set>

struct CloudPoint {
	cv::Point3d pt;
	std::vector<int> imgpt_for_img;
	double reprojection_error;
};

std::vector<cv::DMatch> FlipMatches(const std::vector<cv::DMatch>& matches);
inline void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps) {
	ps.clear();
	for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}

inline void PointsToKeyPoints(const std::vector<cv::Point2f>& ps, std::vector<cv::KeyPoint>& kps) {
	kps.clear();
	for (unsigned int i=0; i<ps.size(); i++) kps.push_back(cv::KeyPoint(ps[i],1.0f));
}

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts);

void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
							   const std::vector<cv::KeyPoint>& imgpts2,
							   const std::vector<cv::DMatch>& matches,
							   std::vector<cv::KeyPoint>& pt_set1,
							   std::vector<cv::KeyPoint>& pt_set2);

void drawArrows(cv::Mat& frame, const std::vector<cv::Point2f>& prevPts, const std::vector<cv::Point2f>& nextPts, const std::vector<uchar>& status, const std::vector<float>& verror, const cv::Scalar& line_color = cv::Scalar(0, 0, 255));

#ifdef USE_PROFILING
#define CV_PROFILE(msg,code)	{\
	std::cout << msg << " ";\
	double __time_in_ticks = (double)cv::getTickCount();\
	{ code }\
	std::cout << "DONE " << ((double)cv::getTickCount() - __time_in_ticks)/cv::getTickFrequency() << "s" << std::endl;\
}
#else
#define CV_PROFILE(msg,code) code
#endif

void load_calibration_data(const std::string &file_name, cv::Mat &intrinsics_common, cv::Mat &distortion_vector);
void open_imgs_dir(const std::string &dir_name, std::vector<cv::Mat>& images, std::vector<std::string>& images_names, double downscale_factor);
void imshow_250x250(const std::string& name_, const cv::Mat& patch);

inline bool operator == (const cv::Size &a, const cv::Size &b) { return a.area() == b.area(); }
inline bool operator != (const cv::Size &a, const cv::Size &b) { return a.area() != b.area(); }
inline bool operator <  (const cv::Size &a, const cv::Size &b) { return a.area() <  b.area(); }
inline bool operator >  (const cv::Size &a, const cv::Size &b) { return a.area() >  b.area(); }
inline bool operator <= (const cv::Size &a, const cv::Size &b) { return a.area() <= b.area(); }
inline bool operator >= (const cv::Size &a, const cv::Size &b) { return a.area() >= b.area(); }

