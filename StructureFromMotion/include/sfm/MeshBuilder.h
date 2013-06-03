#pragma once

#include <vector>

#include <opencv2/core/core.hpp>

#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>

class MeshBuilder
{
protected:
	std::vector<pcl::PolygonMesh> _mesh;

	virtual void _buildImpl(const std::vector<cv::Point3d> &cloudPoints, bool filter) = 0;

public:
	bool	empty()				const { return _mesh.empty(); }
	size_t	segments_count()	const { return _mesh.size(); }

	void build(const std::vector<cv::Point3d> &cloudPoints, bool filter = true) 
	{ 
		_mesh.clear();
		_buildImpl(cloudPoints, filter); 
	};

	void clear() { _mesh.clear(); } 

			std::vector<pcl::PolygonMesh> &get()		{ return _mesh; }
	const	std::vector<pcl::PolygonMesh> &get() const	{ return _mesh; }

			pcl::PolygonMesh &get_segment(size_t at)		{ return _mesh.at(at); }
	const	pcl::PolygonMesh &get_segment(size_t at) const	{ return _mesh.at(at); }
			pcl::PolygonMesh &operator [] (size_t at)		{ return _mesh[at]; }
	const	pcl::PolygonMesh &operator [] (size_t at) const	{ return _mesh[at]; }
};

class ConvexMeshBuilder : public MeshBuilder 
{
	void _buildImpl(const std::vector<cv::Point3d> &cloudPoints, bool filter);
};