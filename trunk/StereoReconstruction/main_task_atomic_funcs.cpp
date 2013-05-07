
#include "stdafx.h"

#include "main_task_atomic_funcs.h"

bool HZEssentialDecomposition(cv::InputArray _E, cv::OutputArray R1, cv::OutputArray R2, cv::OutputArray t1, cv::OutputArray t2)
{
	cv::Mat E = _E.getMat();

	CV_Assert(E.depth() == CV_64F);

	int new_sz[] = { 3, 3 };
	cv::SVD _fm_svd(E.reshape(1, _countof(new_sz), new_sz));

	R1.create(E.size(), CV_64FC1);
	R2.create(E.size(), CV_64FC1);

	t1.create(3, 1, CV_64FC1);
	t2.create(3, 1, CV_64FC1);

	double _singular_ratio = std::abs(_fm_svd.w.at<double>(0) / _fm_svd.w.at<double>(1));
	if (_singular_ratio > 1.0) _singular_ratio = 1.0 / _singular_ratio; // flip ratio to keep it [0,1]
	if (_singular_ratio < 0.7) return false;

	cv::Matx33d W(0,-1, 0,	//HZ 9.13
				  1, 0, 0,
				  0, 0, 1);

	R1.getMat() = _fm_svd.u * cv::Mat(W)	 * _fm_svd.vt;	//HZ 9.19
	R2.getMat() = _fm_svd.u * cv::Mat(W.t()) * _fm_svd.vt;	//HZ 9.19
	
	_fm_svd.u.col(2).copyTo(t1.getMat());	//u3
	t2.getMat() = -_fm_svd.u.col(2);		//u3

	return true;
}