#pragma once

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sfm/MultiCameraPnP.h>

class SceneData {
public:
	typedef struct tagCamera {
		std::string _name;
		cv::Matx33d	_internal;
		cv::Matx34d	_external;
	} Camera;

private:
	std::vector<pcl::PolygonMesh> _mesh;

public:
	static SceneData Export(const MultiCameraPnP &sfm, const std::string &path);

	void save(const std::vector<Camera> &cameras, const std::string &path);
	void save(const MultiCameraPnP &sfm, const std::string &path);

	void build(const std::vector<cv::Point3d> &cloudPoints, bool SORFilter = true);
	void build(const MultiCameraPnP &sfm, bool SORFilter = true) { build(sfm.getPointCloud(), SORFilter); };

	size_t meshCount() const { return _mesh.size(); }

	std::vector<pcl::PolygonMesh>  get() const { return _mesh; }
	std::vector<pcl::PolygonMesh> &get()	   { return _mesh; }

	pcl::PolygonMesh  getSegment(size_t at) const { return at < _mesh.size() ? _mesh[at] : pcl::PolygonMesh(); }
	pcl::PolygonMesh &getSegment(size_t at)	      { return _mesh.at(at); }
};