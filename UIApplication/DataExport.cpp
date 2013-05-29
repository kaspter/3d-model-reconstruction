#include "DataExport.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SceneMesh
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

void SceneMesh::export(const std::string &path) // id: "collada"
{

}

void SceneMesh::build(const std::vector<cv::Point3d> &cloudPoints, bool SORFilter)
{
	// TODO: Mesh construction improvements
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (unsigned int i = 0, imax = cloudPoints.size(); i < imax; ++i) 
		{
			cv::Point3d src_pt = cloudPoints[i];
			// get the RGB color value for the point
			// check for erroneous coordinates (NaN, Inf, etc.)
			if (src_pt.x != src_pt.x || src_pt.y != src_pt.y || src_pt.z != src_pt.z 
#			ifndef WIN32
				|| isnan(src_pt.x)  || isnan(src_pt.y)  || isnan(src_pt.z)
#			else
				|| _isnan(src_pt.x) || _isnan(src_pt.y) || _isnan(src_pt.z)
#			endif
			) continue;
		
			_cloud->push_back(pcl::PointXYZ(src_pt.x, src_pt.y, src_pt.z));
		}
	
		_cloud->width  = static_cast<uint32_t>(_cloud->points.size());
		_cloud->height = static_cast<uint32_t>(_cloud->width != 0);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		if (SORFilter)
		{
			//Create the filtering object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(_cloud);
			sor.setMeanK(50);
			sor.setStddevMulThresh(1.0);
			sor.filter(*cloud);
		} 
		else copyPointCloud(*_cloud, *cloud);
	}

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(_mesh);
}