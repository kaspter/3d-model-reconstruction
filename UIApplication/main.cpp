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

	std::string	images_dir, calib_file_path, out_file_path;
	double downscale_factor = 0.0;

	MultiCameraDistance::FEATURE_MATCHER_MODE features_flag	 = MultiCameraDistance::FEATURE_MATCHER_UNKNOWN;

	std::vector<cv::Mat>	 images;
	std::vector<std::string> image_names;

	// Blockwise code structure prevents of the 'C2362' compiler error
	{ // Parameters parser block

		// TODO: extend parameter list by exported model file path/name
		cv::CommandLineParser arguments(argc, argv, "{1|||model output *.dae file}{-d|directory||images dir}{-c|calibration|out_camera_data.xml|calibration file}{f|features|of|features type}{ds|downscale|1.0|downscale factor}{?|help|false|show help}");

		bool showHelp;
		if (!(showHelp = arguments.get<bool>("help")))
		{
			if ((images_dir = arguments.get<std::string>("directory")).empty()) 
				images_dir = boost::filesystem::current_path().string();

			std::string features = arguments.get<std::string>("features");
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
						if (rich_behavior == "cache") features_flag = static_cast<MultiCameraDistance::FEATURE_MATCHER_MODE>(
							MultiCameraDistance::FEATURE_MATCHER_RICH | MultiCameraDistance::FEATURE_MATCHER_CACHED);
					} 
					else features_flag = MultiCameraDistance::FEATURE_MATCHER_RICH;
				}
			}
			else features_flag = MultiCameraDistance::FEATURE_MATCHER_FAST;

			showHelp |= !boost::filesystem::exists(images_dir) || features_flag == -1;
		}

		if (showHelp)
		{	
			std::string app_path(argv[0]); 
			std::cout << app_path.substr(app_path.find_last_of("\\/") + 1) 
				<< " [image set folder path] - current used if not specified;\n\t[-f=[of|rich[:cache]]] - features tracking method;\n\t[-ds] - images downscale factor.";
			goto EXIT;
		}

		calib_file_path  = images_dir.append("/") + arguments.get<std::string>("calibration");
		out_file_path	 = arguments.get<std::string>("1");
		downscale_factor = arguments.get<double>("downscale");
	}

	{ // Main program block
		boost::scoped_ptr<std::vector<cv::Mat>> cache_ptr;
		bool precached = !!(features_flag & MultiCameraDistance::FEATURE_MATCHER_CACHED);
		if (precached) cache_ptr.reset(new std::vector<cv::Mat>);

		open_imgs_dir(images_dir, images, cache_ptr.get(), image_names, downscale_factor);
		if(images.empty()) 
		{ 
			std::cerr << "Can't get image files at: \'" << images_dir << "\'" << endl;
			retResult = -1;
			goto EXIT;
		}

		cv::Mat intrinsics, distortion;
		load_calibration_data(calib_file_path, intrinsics, distortion);
		if(intrinsics.empty() || distortion.empty()) 
		{ 
			std::cerr << "Can't get calibration data from: \'" << calib_file_path << "\'" << endl;
			retResult = -1;
			goto EXIT;
		}

		boost::scoped_ptr<SceneData>				sceneMeshBuilder	(new SceneData);
		boost::scoped_ptr<VisualizerListener>		visualizerListener	(new VisualizerListener(sceneMeshBuilder.get()));
		boost::scoped_ptr<MultiCameraPnP>			sfm					(new MultiCameraPnP(features_flag, images, image_names, intrinsics, distortion));

		visualizerListener->RunVisualizationThread();
		
						sfm->attach(visualizerListener.get());		
		if (precached)	sfm->LoadFeaturesCache(*cache_ptr.get());
						sfm->RecoverDepthFromImages();
		if (precached)	sfm->ObtainFeaturesCache(*cache_ptr.get());

		if (!out_file_path.empty()) sceneMeshBuilder->save(*sfm, out_file_path);

		visualizerListener->WaitForVisualizationThread();

		goto QUIT;
	}

EXIT:
	std::cout << std::endl << "Hit any key to exit...";
	_getch();

QUIT:
	return retResult;
}
