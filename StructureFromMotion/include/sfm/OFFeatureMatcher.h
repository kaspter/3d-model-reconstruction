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

#include "AbstractFeatureMatcher.h"

class OFFeatureMatcher : public AbstractFeatureMatcher {

	const std::vector<cv::Mat>				&_imgs; 
	std::vector<std::vector<cv::KeyPoint>>	&_imgpts;
	
public:
	OFFeatureMatcher(const std::vector<cv::Mat> &imgs, std::vector<std::vector<cv::KeyPoint>> &imgpts, bool precached = false, bool use_gpu = false);
	void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch> *matches = NULL);
	std::vector<cv::KeyPoint> GetImagePoints(int idx) { return _imgpts[idx]; }
};
