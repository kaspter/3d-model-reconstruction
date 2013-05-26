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

#include "RichFeatureMatcher.h"

#include "FindCameraMatrices.h"

#include <boost/thread.hpp>

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <set>

#include "imp_init.h"
#include "imp_susan.h"
#include "imp_extension.h"

using namespace std;
using namespace cv;

//#define LN_2 0.69314718055994530941723212145818
RichFeatureMatcher::RichFeatureMatcher(std::vector<cv::Mat>& imgs_, std::vector<std::vector<cv::KeyPoint> >& imgpts_) 
	: imgpts(imgpts_), imgs(imgs_)
{
	assert(imgpts_.size() == imgs_.size() && imgs_.size() > 0);

	imp::initModule();
	detector = FeatureDetector::create("PyramidSUSAN");
	imp::PyramidAdapterHack* pyramid = static_cast<imp::PyramidAdapterHack*>(
			dynamic_cast<PyramidAdaptedFeatureDetector*>(detector.obj));

	assert(pyramid);

	imp::SUSAN* susan = static_cast<imp::SUSAN*>(pyramid->detector.obj);
	susan->set("radius",	7);
	susan->set("tparam",	29.33);
	//susan->set("gparam",	120.00);
	susan->set("prefilter", true);
	susan->set("subpixel",	true);

	for (unsigned i = 0, imax = imgs.size(); i < imax; ++i)
	{
		double levelCoeff = std::log(std::min(imgs[i].cols, imgs[i].rows) / (2.0 * susan->get<int>("radius") + 1));
		pyramid->maxLevel = std::max(static_cast<int>(levelCoeff), 2);
		pyramid->detect(imgs[i], imgpts[i]);
		susan->reset_pass_counter();
	}

	initModule_nonfree();
	extractor = DescriptorExtractor::create("SIFT");
	extractor->compute(imgs, imgpts, descriptors);
}	

void RichFeatureMatcher::MatchFeatures(int idx_i, int idx_j, vector<DMatch> *matches) {
    
#ifdef __SFM__DEBUG__
    const Mat& img_1 = imgs[idx_i];
    const Mat& img_2 = imgs[idx_j];
#endif
    const vector<KeyPoint>& imgpts1 = imgpts[idx_i];
    const vector<KeyPoint>& imgpts2 = imgpts[idx_j];
    const Mat& descriptors_1 = descriptors[idx_i];
    const Mat& descriptors_2 = descriptors[idx_j];
    
    std::vector< DMatch > good_matches_,very_good_matches_;
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    
	stringstream ss; ss << "imgpts1 has " << imgpts1.size() << " points (descriptors " << descriptors_1.rows << ")" << endl;
    std::cout << ss.str();
	stringstream ss1; ss1 << "imgpts2 has " << imgpts2.size() << " points (descriptors " << descriptors_2.rows << ")" << endl;
    std::cout << ss1.str();
    
    keypoints_1 = imgpts1;
    keypoints_2 = imgpts2;
    
    if(descriptors_1.empty()) CV_Error(0,"descriptors_1 is empty");
    if(descriptors_2.empty()) CV_Error(0,"descriptors_2 is empty");
    
    //matching descriptor vectors using Flann Based matcher
	vector<DMatch> _null_matches;
	if (matches == NULL) matches = &_null_matches;
    matches->clear();

	cv::Ptr<cv::DescriptorMatcher> matcher;
	
	// TODO: remove this block!
	static boost::mutex block_mutex;   
	{	// Temporary workaround for non thread-safe lazy initialization implementation of cv::Algorithm based classes 
		boost::lock_guard<boost::mutex> block_guard(block_mutex);
		matcher = cv::DescriptorMatcher::create("FlannBased");
	}
	
	assert(!matcher.empty());
	matcher->match(descriptors_1, descriptors_2, *matches);
	assert(matches->size() > 0);
	
//    double max_dist = 0; double min_dist = 1000.0;
//    //-- Quick calculation of max and min distances between keypoints
//    for(unsigned int i = 0; i < matches->size(); i++ )
//    { 
//        double dist = matches[i].distance;
//		if (dist>1000.0) { dist = 1000.0; }
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }
//    
//#ifdef __SFM__DEBUG__
//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );
//#endif
    
    vector<KeyPoint> imgpts1_good,imgpts2_good;
    
//    if (min_dist <= 0) {
//        min_dist = 10.0;
//    }
    
    // Eliminate any re-matching of training points (multiple queries to one training)
//    double cutoff = 4.0*min_dist;
    std::set<int> existing_trainIdx;
    for(unsigned int i = 0; i < matches->size(); i++ )
    { 
        //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
        if ((*matches)[i].trainIdx <= 0) {
            (*matches)[i].trainIdx = (*matches)[i].imgIdx;
        }
        
        if( existing_trainIdx.find((*matches)[i].trainIdx) == existing_trainIdx.end() && 
           (*matches)[i].trainIdx >= 0 && (*matches)[i].trainIdx < (int)(keypoints_2.size()) /*&&
           matches[i].distance > 0.0 && matches[i].distance < cutoff*/ ) 
        {
            good_matches_.push_back((*matches)[i]);
            imgpts1_good.push_back(keypoints_1[(*matches)[i].queryIdx]);
            imgpts2_good.push_back(keypoints_2[(*matches)[i].trainIdx]);
            existing_trainIdx.insert((*matches)[i].trainIdx);
        }
    }
    
    
#ifdef __SFM__DEBUG__
    std::cout << "keypoints_1.size() " << keypoints_1.size() << " imgpts1_good.size() " << imgpts1_good.size() << endl;
    std::cout << "keypoints_2.size() " << keypoints_2.size() << " imgpts2_good.size() " << imgpts2_good.size() << endl;
    
    {
        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                    good_matches_, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );        
        //-- Show detected matches
		stringstream ss; ss << "Feature Matches " << idx_i << "-" << idx_j;
        imshow(ss.str() , img_matches );
        waitKey(500);
        destroyWindow(ss.str());
    }
#endif
    
    vector<uchar> status;
    vector<KeyPoint> imgpts2_very_good,imgpts1_very_good;
    
    assert(imgpts1_good.size() > 0 && imgpts2_good.size() > 0);
	assert(good_matches_.size() > 0 &&  imgpts1_good.size() == good_matches_.size() && imgpts1_good.size() == imgpts2_good.size());
	
    //Select features that make epipolar sense
    GetFundamentalMat(keypoints_1,keypoints_2,imgpts1_very_good,imgpts2_very_good,good_matches_);
	
    //Draw matches
#ifdef __SFM__DEBUG__
    {
        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                    good_matches_, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );        
        //-- Show detected matches
        imshow( "Good Matches", img_matches );
        waitKey(100);
        destroyWindow("Good Matches");
    }
#endif
}
