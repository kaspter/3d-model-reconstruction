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

#include "Visualization.h"

#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Eigen>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>





#define pclp3(eigenv3f) pcl::PointXYZ(eigenv3f.x(),eigenv3f.y(),eigenv3f.z())

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cloud1, cloud_no_floor, orig_cloud;
std::string cloud_to_show_name = "";
volatile bool show_cloud = false;
volatile bool sor_applied = false;
volatile bool show_cloud_A = true;

////////////////////////////////// Show Camera ////////////////////////////////////
std::deque<std::pair<std::string,pcl::PolygonMesh>>				  cam_meshes;
std::deque<std::pair<std::string,std::vector<Eigen::Matrix<float,6,1>>>> linesToShow;
//TODO define mutex
bool bShowCam;
int	 iCamCounter = 0;
int	 iLineCounter = 0;
int	 ipolygon[18] = {0,1,2,  0,3,1,  0,4,3,  0,2,4,  3,1,4,   2,4,1};

inline pcl::PointXYZ Eigen2PointXYZ(Eigen::Vector3f v) { return pcl::PointXYZ(v[0],v[1],v[2]); }
inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb) { pcl::PointXYZRGB p(rgb[0],rgb[1],rgb[2]); p.x = v[0]; p.y = v[1]; p.z = v[2]; return p; }
inline pcl::PointNormal Eigen2PointNormal(Eigen::Vector3f v, Eigen::Vector3f n) { pcl::PointNormal p; p.x=v[0];p.y=v[1];p.z=v[2];p.normal_x=n[0];p.normal_y=n[1];p.normal_z=n[2]; return p;}
inline float* Eigen2float6(Eigen::Vector3f v, Eigen::Vector3f rgb) { static float buf[6]; buf[0]=v[0];buf[1]=v[1];buf[2]=v[2];buf[3]=rgb[0];buf[4]=rgb[1];buf[5]=rgb[2]; return buf; }
inline Eigen::Matrix<float,6,1> Eigen2Eigen(Eigen::Vector3f v, Eigen::Vector3f rgb) { return (Eigen::Matrix<float,6,1>() << v[0],v[1],v[2],rgb[0],rgb[1],rgb[2]).finished(); }
inline std::vector<Eigen::Matrix<float,6,1> > AsVector(const Eigen::Matrix<float,6,1>& p1, const Eigen::Matrix<float,6,1>& p2) { std::vector<Eigen::Matrix<float,6,1> > v(2); v[0] = p1; v[1] = p2; return v; }

void visualizerShowCamera(const Eigen::Matrix3f& R, const Eigen::Vector3f& _t, float r, float g, float b, double s = 0.01, const std::string& name = "") {
	std::string name_ = name,line_name = name + "line";
	if (name.length() <= 0) {
		stringstream ss; ss<<"camera"<<iCamCounter++;
		name_ = ss.str();
		ss << "line";
		line_name = ss.str();
	}
	
	Eigen::Vector3f t = -R.transpose() * _t;

	Eigen::Vector3f vright = R.row(0).normalized() * s;
	Eigen::Vector3f vup = -R.row(1).normalized() * s;
	Eigen::Vector3f vforward = R.row(2).normalized() * s;

	Eigen::Vector3f rgb(r,g,b);

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	mesh_cld.push_back(Eigen2PointXYZRGB(t,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 + vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 - vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 + vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 - vup/2.0,rgb));

	//TODO Mutex acquire
	pcl::PolygonMesh pm;
	pm.polygons.resize(6); 
	for(int i=0;i<6;i++)
		for(int _v=0;_v<3;_v++)
			pm.polygons[i].vertices.push_back(ipolygon[i*3 + _v]);
	pcl::toROSMsg(mesh_cld,pm.cloud);
	bShowCam = true;
	cam_meshes.push_back(std::make_pair(name_,pm));
	//TODO mutex release

	linesToShow.push_back(std::make_pair(line_name,
		AsVector(Eigen2Eigen(t,rgb),Eigen2Eigen(t + vforward*3.0,rgb))
		));
}
void visualizerShowCamera(const float R[9], const float t[3], float r, float g, float b) {
	visualizerShowCamera(Eigen::Matrix3f(R).transpose(),Eigen::Vector3f(t),r,g,b);
}
void visualizerShowCamera(const float R[9], const float t[3], float r, float g, float b, double s) {
	visualizerShowCamera(Eigen::Matrix3f(R).transpose(),Eigen::Vector3f(t),r,g,b,s);
}
void visualizerShowCamera(const cv::Matx33f& R, const cv::Vec3f& t, float r, float g, float b, double s, const std::string& name) {
	visualizerShowCamera(Eigen::Matrix<float,3,3,Eigen::RowMajor>(R.val),Eigen::Vector3f(t.val),r,g,b,s,name);
}
/////////////////////////////////////////////////////////////////////////////////

void SORFilter() 
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	std::cout << "Cloud before SOR filtering: " << cloud->width * cloud->height << " data points" << std::endl;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);
	
	copyPointCloud(*cloud_filtered,*cloud);
	copyPointCloud(*cloud,*orig_cloud);

	std::cout << "Cloud after SOR filtering: " << cloud->width * cloud->height << " data points " << std::endl;
}	

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CloudVisualizer
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CloudVisualizer::populatePointCloud(const std::vector<cv::Point3d>& pointcloud, const std::vector<cv::Vec3b>& pointcloud_RGB, pclColoredPointCloud::Ptr& mycloud)
{
	_ASSERT_EXPR(pointcloud.size() == pointcloud_RGB.size() || pointcloud_RGB.size() == 0,
		L"point cloud space and color data vectors must be aligned");

	for (unsigned int i = 0, imax = pointcloud.size(); i < imax; ++i) 
	{
		// get the RGB color value for the point
		// check for erroneous coordinates (NaN, Inf, etc.)
		if (pointcloud[i].x != pointcloud[i].x || 
			pointcloud[i].y != pointcloud[i].y || 
			pointcloud[i].z != pointcloud[i].z || 
#ifndef WIN32
			isnan(pointcloud[i].x) ||
			isnan(pointcloud[i].y) || 
			isnan(pointcloud[i].z) ||
#else
			_isnan(pointcloud[i].x) ||
			_isnan(pointcloud[i].y) || 
			_isnan(pointcloud[i].z) ||
#endif
			false
		) {
			continue;
		}
		
		pcl::PointXYZRGB pclp(pointcloud_RGB[i][2], pointcloud_RGB[i][1], pointcloud_RGB[i][0]);
		
		// 3D coordinates
		pclp.x = pointcloud[i].x;
		pclp.y = pointcloud[i].y;
		pclp.z = pointcloud[i].z;
		
		mycloud->push_back(pclp);
	}
	
	mycloud->width  = static_cast<uint32_t>(mycloud->points.size());	// number of points
	mycloud->height = static_cast<uint32_t>(mycloud->width != 0);		// a list, one row of data
}

void CloudVisualizer::visualizerCallback(CloudVisualizer* me, void* dummy)
{

	pcl::visualization::PCLVisualizer viewer("3D Model Space");   

	void *objects_essential[] = {me, &viewer};
	viewer.registerKeyboardCallback(KeyboardEventCallback, objects_essential);
	
    while (!viewer.wasStopped())
    {
		me->_cloud_to_show_mutex.lock();
		if (me->cloud_to_show.get() != NULL) 
		{
			viewer.removePointCloud("pcloud");
			viewer.addPointCloud(me->cloud_to_show, "pcloud");
			me->cloud_to_show.reset();
		}
		me->_cloud_to_show_mutex.unlock();

		if(cam_meshes.size() > 0) {
			int num_cams = cam_meshes.size();
			cout << "showing " << num_cams << " cameras" << endl;
			while(cam_meshes.size()>0) {
				viewer.removeShape(cam_meshes.front().first);
				viewer.addPolygonMesh(cam_meshes.front().second,cam_meshes.front().first);
				cam_meshes.pop_front();
			}
		}
		if(linesToShow.size() > 0) {
			cout << "showing " << linesToShow.size() << " lines" << endl;
			while(linesToShow.size()>0) {
				std::vector<Eigen::Matrix<float,6,1> > oneline = linesToShow.front().second;
				pcl::PointXYZRGB	A(oneline[0][3],oneline[0][4],oneline[0][5]),
									B(oneline[1][3],oneline[1][4],oneline[1][5]);
				for(int j=0;j<3;j++) {A.data[j] = oneline[0][j]; B.data[j] = oneline[1][j];}
				viewer.removeShape(linesToShow.front().first);
				viewer.addLine<pcl::PointXYZRGB,pcl::PointXYZRGB>(A,B,linesToShow.front().first);
				linesToShow.pop_front();
			} 
			linesToShow.clear();
		}
		viewer.spinOnce(50);
    }
}

void CloudVisualizer::KeyboardEventCallback (const pcl::visualization::KeyboardEvent& event_, void *void_this)
{
	_ASSERT(void_this != NULL, L"It seems to be parameters vector is corrupted");
	CloudVisualizer *me = reinterpret_cast<CloudVisualizer*>(*(uintptr_t*)void_this);

	if ((event_.getKeyCode() == '.' || event_.getKeyCode() == '>') && event_.keyDown()) me->SelectCloudToShow(CSD_FORWARD);
	if ((event_.getKeyCode() == ',' || event_.getKeyCode() == '<') && event_.keyDown()) me->SelectCloudToShow(CSD_BACKWARD);
}	

void CloudVisualizer::LoadClouds(const RawCloudDataCollection &cloud_data)
{
	if (cloud_data.empty()) return;
	
	_cloud_registry_mutex.lock();

	cloud_registry.clear();
	for (RawCloudDataCollection::const_iterator i = cloud_data.begin(), iend = cloud_data.end(); i != iend; ++i)
	{
		pclColoredPointCloud::Ptr new_cloud(new pclColoredPointCloud);
		populatePointCloud(i->first, i->second, new_cloud);
		cloud_registry.push_front(new_cloud);
	}
	SelectCloudToShow(CSD_BACKWARD);

	_cloud_registry_mutex.unlock();
}

void CloudVisualizer::SelectCloudToShow(CLOUD_SELECTION_DIRECTION csd)
{
	_cloud_registry_mutex.lock();

	if (!cloud_registry.empty())
	{
		switch(csd)
		{
		case CSD_BACKWARD:
			cloud_registry.push_front(cloud_registry.back());
			cloud_registry.pop_back();
			break;
		case CSD_FORWARD:
			cloud_registry.push_back(cloud_registry.front());
			cloud_registry.pop_front();
			break;

		case CSD_INDIFFERENT:
			std::srand(boost::get_system_time().time_of_day().fractional_seconds());
			cloud_registry.push_front(*cloud_registry.erase(
				cloud_registry.begin() + (std::rand() % cloud_registry.size())
				));
			break;

		default:
			_ASSERTE("Bad direction to select cloud");
		}
	}

	_cloud_to_show_mutex.lock();

	if (cloud_registry.empty()) cloud_to_show.reset(); else cloud_to_show = cloud_registry.front();

	_cloud_registry_mutex.unlock();
	_cloud_to_show_mutex.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VisualizerListener
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VisualizerListener::update(const std::vector<cv::Point3d> &pcld_a, const std::vector<cv::Vec3b> &pcld_a_rgb, 
								const std::vector<cv::Point3d> &pcld_b, const std::vector<cv::Vec3b> &pcld_b_rgb, 
								const std::vector<cv::Matx34d> &cameras
) {
	RawCloudDataCollection cloud_data;
	cloud_data.push_back(RawCloudData(pcld_a, pcld_a_rgb));
	cloud_data.push_back(RawCloudData(pcld_b, pcld_b_rgb));

	LoadClouds(cloud_data);
		
	for(unsigned i = 0, imax = cameras.size(); i < imax; ++i) {
		cv::Matx34d c = cameras[i];
		std::stringstream ss; ss << "camera #" << i;
		visualizerShowCamera(cv::Matx33f(c(0,0), c(0,1), c(0,2),
											c(1,0), c(1,1), c(1,2),
											c(2,0), c(2,1), c(2,2)),
								cv::Vec3f(  c(0,3), c(1,3), c(2,3)),
								255,0,0,0.2,ss.str()
						);
	}
}



