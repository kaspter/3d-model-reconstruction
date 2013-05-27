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

#include <boost/shared_ptr.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <sfm/MultiCameraPnP.h>

#include "Visualization.h"

using namespace std;

static std::vector<cv::Mat>		images;
static std::vector<std::string> image_names;

int main(int argc, char** argv) {
	//RunVisualizationThread();

	// TODO: CommandLineParser 
	if (argc < 2) 
	{
		cerr << "USAGE: " << argv[0] << " <path_to_images> [use rich features (RICH/OF) = RICH] [use GPU (GPU/CPU) = GPU] [downscale factor = 1.0]" << endl;
		return 0;
	}
	
	double downscale_factor = 1.0;
	if(argc >= 5) downscale_factor = atof(argv[4]);

	open_imgs_dir(argv[1], images, image_names, downscale_factor);
	if(images.empty()) 
	{ 
		cerr << "can't get image files" << endl; 
		return 1; 
	}
	boost::shared_ptr<VisualizerListener> visualizerListener(new VisualizerListener);
	visualizerListener->RunVisualizationThread();

	cv::Ptr<MultiCameraPnP> sfm = new MultiCameraPnP(images, image_names, string(argv[1]));
	sfm->use_rich_features	= argc < 3 || (strcmp(argv[2], "RICH") == 0);
	sfm->use_gpu			= argc < 4 ?  (cv::gpu::getCudaEnabledDeviceCount() > 0) : (strcmp(argv[3], "GPU") == 0);
	sfm->attach(visualizerListener.get());
	sfm->RecoverDepthFromImages();

	visualizerListener->WaitForVisualizationThread();
}
