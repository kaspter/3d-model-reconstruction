
#include "MeshBuilder.h"

#include <boost/scoped_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

void validateNormalPointsCloud(pcl::PointCloud<pcl::PointNormal> &normal_cloud)
{
	std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal>> valid_pts;

	valid_pts.reserve(normal_cloud.points.size());
	for (size_t i = 0, imax = normal_cloud.points.size(); i < imax; ++i)
	{
		bool is_nan = false;

		pcl::PointNormal &pt = normal_cloud[i];
		for (size_t f = 0; f < _countof(pt.data); ++f)
		{
			if ((is_nan = !boost::math::isfinite(pt.data  [f])
					    ||!boost::math::isfinite(pt.data_c[f])
						|| boost::math::isnan   (pt.data_n[f]))) break;
			pt.data_n[f] = boost::math::sign(pt.data_n[f]);
		}
		if (is_nan) continue;
		valid_pts.push_back(pt);
	}
	normal_cloud.points.swap(valid_pts);

	normal_cloud.width  = static_cast<uint32_t>(normal_cloud.points.size());
	normal_cloud.height = static_cast<uint32_t>(normal_cloud.width != 0);
}

void ConvexMeshBuilder::_buildImpl(const std::vector<cv::Point3d> &cloudPoints, bool filter)
{
	Eigen::Vector3f	eigen_mean, eigen_vals;

	pcl::PointCloud<pcl::PointXYZ>::Ptr			cloud	(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr		tree	(new pcl::search::KdTree<pcl::PointXYZ>);
	{	// generic to pcl point cloud conversion
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
		
		pcl::PCA<pcl::PointXYZ> pca(*_cloud, true);
		
		double w = pca.getMean()(3); if (std::abs(w) 
			< std::numeric_limits<float>::epsilon()) w = 1.0;
		eigen_mean = pca.getMean().head(3) / w;
		eigen_vals = pca.getEigenValues();	

		if (filter)
		{
			//Eigen::Vector4f min, max;
			//pcl::getMinMax3D(*_cloud, min, max); 
			//max -= min;

			//for (int i = 0; i < max.size(); ++i) 
			//	if (std::abs(max.data()[i]) < std::numeric_limits<float>::epsilon()) max.data()[i] = 1.0F;
			//float cloud_volume = std::abs(max(0) * max(1) * max(2));

			std::vector<int>	search_idx;
			std::vector<float>	search_sdist;
			float				search_radius = std::abs(eigen_vals(2) / eigen_vals(1));
			pcl::PointXYZ		search_anchor(eigen_mean.x(), eigen_mean.y(), eigen_mean.z());

			tree->setInputCloud(_cloud);
			tree->radiusSearch(search_anchor, search_radius, search_idx, search_sdist);
			
			//Create the filtering object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

			sor.setStddevMulThresh	(search_radius);
			sor.setMeanK			(std::max(int(search_idx.size()), int(3)));
			sor.setInputCloud		(_cloud);

			sor.filter				(*cloud);
		} 
		else copyPointCloud(*_cloud, *cloud);
	}
	tree->setInputCloud(cloud);

	//{	// point cloud segmentation (foreground extraction)

	//}

	pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
	{	// Normal estimation*
		// Init object (second point type is for the normals, even if unused)
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
		// Set parameters
		mls.setPolynomialFit	(true);
		mls.setComputeNormals	(true);
		mls.setSearchRadius		(std::max(std::abs(eigen_vals(2) / eigen_vals(1)), 0.009F));
		mls.setSearchMethod		(tree);
		mls.setInputCloud		(cloud);
		// Reconstruct
		mls.process					(*normal_cloud);
		validateNormalPointsCloud	(*normal_cloud);

		//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	
		//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		//n.setInputCloud(cloud);
		//n.setSearchMethod(tree);
		//n.setKSearch(20);
		//n.compute(*normals);
		////* normals should not contain the point normals + surface curvatures

		//pcl::concatenateFields(*cloud, *normals, *normal_cloud);
	}

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr normal_tree (new pcl::search::KdTree<pcl::PointNormal>); 
	
	normal_tree->setInputCloud(normal_cloud);

	// Initialize objects
	pcl::ConvexHull<pcl::PointNormal> chull;

	//chull.setAlpha			(0.215);
	chull.setInputCloud		(normal_cloud);
	chull.setSearchMethod	(normal_tree);

	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	//gp3.setSearchMethod	(normal_tree);
	//gp3.setInputCloud	(normal_cloud);
	//// Set typical values for the parameters
	//gp3.setMu						(2.5);
	//gp3.setSearchRadius				(0.64); // Set the maximum distance between connected points (maximum edge length)
	//gp3.setMaximumNearestNeighbors	(8);
	//gp3.setMaximumSurfaceAngle		(M_PI / 4); // 45 degrees
	//gp3.setMinimumAngle				(M_PI / 18); // 10 degrees
	//gp3.setMaximumAngle				(M_PI / 3 * 2); // 120 degrees
	//gp3.setNormalConsistency		(true);

	//// Get result
	_mesh.resize(1);
	chull.reconstruct(_mesh[0]);
	//gp3.reconstruct(_mesh[0]);
}