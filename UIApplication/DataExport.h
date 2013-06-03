#pragma once

#include <sfm/MeshBuilder.h>

#include <string>

namespace DataExport {
	bool Collada(const MeshBuilder &builder, const std::string &path);
};

//class SceneData {
//public:
//	typedef struct tagCamera {
//		std::string _name;
//		cv::Matx33d	_internal;
//		cv::Matx34d	_external;
//	} Camera;
//
//private:
//	Eigen::Quaterniond	_bcube_orientation;
//	Eigen::Vector3d		_bcube_translation;
//	Eigen::Vector3d		_bcube_dimensions;
//
//	std::vector<pcl::PolygonMesh> _mesh;
//
//public:
//	//static SceneData Export(const MultiCameraPnP &sfm, const std::string &path);
//
//	void save(const std::vector<Camera> &cameras, const std::string &path);
//	void save(const MultiCameraPnP &sfm, const std::string &path);
//
//	void build(const std::vector<cv::Point3d> &cloudPoints, bool filter = true);
//	void build(const MultiCameraPnP &sfm, bool filter = true) { build(sfm.getPointCloud(), filter); };
//
//	size_t meshCount() const { return _mesh.size(); }
//
//	std::vector<pcl::PolygonMesh>  get() const { return _mesh; }
//	std::vector<pcl::PolygonMesh> &get()	   { return _mesh; }
//
//	pcl::PolygonMesh  getSegment(size_t at) const { return at < _mesh.size() ? _mesh[at] : pcl::PolygonMesh(); }
//	pcl::PolygonMesh &getSegment(size_t at)	      { return _mesh.at(at); }
//};