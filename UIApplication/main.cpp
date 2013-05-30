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

#include <conio.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/gpu/gpu.hpp>

#include <sfm/MultiCameraPnP.h>

#include "Visualization.h"
#include "DataExport.h"

// Startup parameters: E:/in-out/scenes >bin\log.txt 2>&1 
int main(int argc, char** argv) 
{
	int retResult = 0;

	std::string	images_dir, calib_file_path;
	unsigned	features_flag	 = 0x00;
	double		downscale_factor = 0.0;

	std::vector<cv::Mat>	 images;
	std::vector<std::string> image_names;

	// Blockwise code structure prevents of the 'C2362' compiler error
	{ // Parameters parser block

		// TODO: extend parameter list by exported model file path/name
		cv::CommandLineParser arguments(argc, argv, "{1|||images dir}{2||out_camera_data.xml|calibration file}{f|features|of|features type}{ds|downscale|1.0|downscale factor}{?|help|false|show help}");

		bool showHelp;
		if (!(showHelp = arguments.get<bool>("help")))
		{
			if ((images_dir = arguments.get<std::string>("1")).empty()) 
				images_dir = boost::filesystem::current_path().string();

			features_flag = -1;
			std::string features	= arguments.get<std::string>("features");
			std::transform(features.begin(), features.end(), features.begin(), std::tolower);
			if (features != "of" && features != "optical_flow") 
			{
				char rich[] = "rich";
				if (features.compare(0, _countof(rich) - 1, rich) == 0)
				{
					std::string rich_behavior = _countof(rich) <= features.length() ? features.substr(_countof(rich) - 1) : std::string();
					if (!rich_behavior.empty())
					{
						rich_behavior.erase(rich_behavior.begin());
						if (rich_behavior == "cache") features_flag = 
							MultiCameraPnP::FEATURE_MATCHER_RICH | MultiCameraPnP::FEATURE_MATCHER_CACHED;
					} 
					else features_flag = MultiCameraPnP::FEATURE_MATCHER_RICH;
				}
			}
			else features_flag = MultiCameraPnP::FEATURE_MATCHER_FAST;

			showHelp |= !boost::filesystem::exists(images_dir) || features_flag == -1;
		}

		if (showHelp)
		{	
			std::string app_path(argv[0]); 
			std::cout << app_path.substr(app_path.find_last_of("\\/") + 1) 
				<< " [image set folder path] - current used if not specified;\n\t[-f=[of|rich[:cache]]] - features tracking method;\n\t[-ds] - images downscale factor.";
			goto EXIT;
		}

		calib_file_path  = images_dir.append("/") + arguments.get<std::string>("2");
		downscale_factor = arguments.get<double>("downscale");
	}

	{ // Main program block
		open_imgs_dir(images_dir, images, image_names, downscale_factor);
		if(images.empty()) 
		{ 
			std::cerr << "Can't get image files at: \'" << images_dir << "\'" << endl;
			retResult = -1;
			goto EXIT;
		}

		cv::Mat intrinsics, distortion;
		load_calibration_data(calib_file_path, intrinsics, distortion);

		boost::scoped_ptr<SceneData> sceneMeshBuilder(new SceneData);
		boost::scoped_ptr<VisualizerListener> visualizerListener(new VisualizerListener(sceneMeshBuilder.get()));
		visualizerListener->RunVisualizationThread();

		boost::scoped_ptr<MultiCameraPnP> sfm(new MultiCameraPnP(images, image_names, intrinsics, distortion));
		sfm->use_rich_features = features_flag;
		sfm->attach(visualizerListener.get());
		sfm->RecoverDepthFromImages();

		sceneMeshBuilder->save(*sfm, "test.dae");

		visualizerListener->WaitForVisualizationThread();
		goto QUIT;
	}

EXIT:
	std::cout << std::endl << "Hit any key to exit...";
	_getch();

QUIT:
	return retResult;
}
