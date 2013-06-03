
#include "MeshBuilder.h"

#include <boost/math/special_functions/fpclassify.hpp>

#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

void ConvexMeshBuilder::_buildImpl(const std::vector<cv::Point3d> &cloudPoints, bool filter)
{
	// TODO: Mesh construction improvements
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (unsigned int i = 0, imax = cloudPoints.size(); i < imax; ++i) 
		{
			const cv::Point3d &src_pt = cloudPoints[i];
				
			// check for erroneous coordinates (NaN, Inf, etc.)
			if  (boost::math::isfinite(src_pt.x) && boost::math::isfinite(src_pt.y) && boost::math::isfinite(src_pt.z)) 
				_cloud->push_back(pcl::PointXYZ(src_pt.x, src_pt.y, src_pt.z));
		}
	
		_cloud->width  = static_cast<uint32_t>(_cloud->points.size());
		_cloud->height = static_cast<uint32_t>(_cloud->width != 0);

		if (filter)
		{
			pcl::PCA<pcl::PointXYZ> pca(*_cloud, true);
			Eigen::Vector3f eigen_vals = pca.getEigenValues();

			//Create the filtering object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud		(_cloud);
			sor.setMeanK			(eigen_vals(0) / eigen_vals(2));
			sor.setStddevMulThresh	(1.0);
			sor.filter				(*cloud);
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
	gp3.setSearchRadius(0.64);

	// Set typical values for the parameters
	gp3.setMu						(2.5);
	gp3.setMaximumNearestNeighbors	(8);
	gp3.setMaximumSurfaceAngle		(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle				(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle				(M_PI / 3 * 2); // 120 degrees
	gp3.setNormalConsistency		(true);

	// Get result
	_mesh.resize(1);
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(_mesh[0]);
}