#pragma once

#include "comparison_predicate.h"

template <typename _Vt>
std::vector<std::vector<cv::Point_<_Vt>>> matchedKeypointsCoords(const std::vector<std::vector<cv::KeyPoint>> &keypoints, 
	const std::vector<cv::DMatch> &matches, const std::vector<uchar> &match_status = std::vector<uchar>())
{
	const uchar	*_match_status = match_status.empty() ? NULL : &match_status[0];
	CV_Assert( keypoints.size() == 2 && (_match_status == NULL || match_status.size() == matches.size()) );
	const cv::KeyPoint *_points_src[] = { &keypoints[0][0], &keypoints[1][0] };
	
	size_t inliers_count = _match_status == NULL ? matches.size() 
		: std::count_if(match_status.begin(), match_status.end(), comparison_predicate<uchar, NOT_EQUAL>(0x00));
	std::vector<std::vector<cv::Point_<_Vt>>> fundamental_inliers(2, std::vector<cv::Point_<_Vt>>(inliers_count));

	if (inliers_count != 0)
	{
		cv::Point_<_Vt>	*_points_dst[] = { &fundamental_inliers[0][0], &fundamental_inliers[1][0] };

		size_t current = 0;
		for (size_t i=0, imax = matches.size(); i<imax; ++i) 
		{
			if ( _match_status == NULL || _match_status[i] != 0x00 )
			{
				const cv::DMatch &match = matches[i];
				_points_dst[0][current] = cv::Point_<_Vt>(_points_src[0][match.queryIdx].pt.x, _points_src[0][match.queryIdx].pt.y);
				_points_dst[1][current] = cv::Point_<_Vt>(_points_src[1][match.trainIdx].pt.x, _points_src[1][match.trainIdx].pt.y);
				++current;
			}
		}

		fundamental_inliers[0].shrink_to_fit();
		fundamental_inliers[1].shrink_to_fit();
	}

	return fundamental_inliers;
}

template <typename _Vt>
std::vector<std::vector<cv::Point_<_Vt>>> filterPointsByStatus(
	const std::vector<std::vector<cv::Point_<_Vt>>> &fundamental_inliers, const std::vector<uchar> &status)
{
	CV_Assert( fundamental_inliers.size() == 2 && (fundamental_inliers[0].size() == status.size() && fundamental_inliers[1].size() == status.size()) );

	const uchar				*_status	   =   &status[0];
	const cv::Point_<_Vt>	*_points_src[] = { &fundamental_inliers[0][0], &fundamental_inliers[1][0] };

	size_t inliers_count = std::count_if(status.begin(), status.end(), comparison_predicate<uchar, NOT_EQUAL>(0x00));
	std::vector<std::vector<cv::Point_<_Vt>>> points_filtered(2, std::vector<cv::Point_<_Vt>>(inliers_count));
	
	if (inliers_count != 0)
	{
		cv::Point_<_Vt>	*_points_dst[] = { &points_filtered[0][0], &points_filtered[1][0] };

		size_t current = 0;
		for (int i = 0, imax = status.size(); i < imax; ++i)
		{
			if (_status[i]) 
			{
				_points_dst[0][current] = _points_src[0][i];
				_points_dst[1][current] = _points_src[1][i];
				++current;
			}
		}

		points_filtered[0].shrink_to_fit();
		points_filtered[1].shrink_to_fit();
	}

	return points_filtered;
}

template <typename _ElemT>
void pointsFromHomogeneous(cv::InputArray src, cv::OutputArray dst)
{
	if (src.empty()) return;

	cv::Mat _dst, _src = src.getMat();
	CV_Assert( _src.rows == 4 && _src.channels() == 1 && _src.elemSize1() == sizeof(_ElemT) );

	dst.create(_src.cols, 1, CV_MAKETYPE(_src.depth(), 3));
	_dst = dst.getMat();

	_ElemT *point_cliche[] = { 
		_src.ptr<_ElemT>(0),
		_src.ptr<_ElemT>(1),
		_src.ptr<_ElemT>(2),
		_src.ptr<_ElemT>(3)
	};

	for (int i = 0; i < _src.cols; ++i)
	{
		_ElemT scale = point_cliche[3][i] != _ElemT(0)
			? _ElemT(1) / point_cliche[3][i] : std::numeric_limits<_ElemT>::infinity();
		_dst.at<cv::Point3_<_ElemT>>(i) = cv::Point3_<_ElemT>(
				point_cliche[0][i] * scale,
				point_cliche[1][i] * scale,
				point_cliche[2][i] * scale
			);
	}
}

bool HZEssentialDecomposition(cv::InputArray _E, cv::OutputArray R1, cv::OutputArray R2, cv::OutputArray t1, cv::OutputArray t2);
inline bool isCoherent(const cv::Mat &R) { return std::abs(cv::determinant(R)) - 1.0 < 1e-07; }