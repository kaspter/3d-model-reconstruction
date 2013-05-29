#pragma once

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

class SceneMesh {
	pcl::PolygonMesh _mesh;

public:
	static void export(const std::string &path);

	void build(const std::vector<cv::Point3d> &cloudPoints, bool SORFilter = true);
	pcl::PolygonMesh  get() const	{ return _mesh; }
	pcl::PolygonMesh &get()			{ return _mesh; }
};