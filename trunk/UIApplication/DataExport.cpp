#include "DataExport.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SceneMesh
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <assimp/scene.h>
#include <assimp/camera.h>
#include <assimp/mesh.h>

#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

inline void _cameraToAssimp(const SceneData::Camera &src, aiCamera &dst)
{
	dst.mAspect			= std::abs(static_cast<float>(src._internal(1,2) / src._internal(0,2)));
	dst.mHorizontalFOV	= std::abs(static_cast<float>(std::atan2(src._internal(0,2), src._internal(0,0))));
	dst.mClipPlaneFar	= static_cast<float>(std::max(src._internal(0,0), src._internal(1,1)));
	dst.mClipPlaneNear	= 1.0F;
	dst.mName			= src._name;
		
	cv::Matx14d row; 
	cv::normalize(src._external.row(2), row);
	dst.mLookAt = aiVector3D(
			static_cast<float>(row.val[0]),
			static_cast<float>(row.val[1]),
			static_cast<float>(row.val[2])
		);
	cv::normalize(src._external.row(1), row);
	dst.mUp = aiVector3D(
			static_cast<float>(row.val[0]),
			static_cast<float>(row.val[1]),
			static_cast<float>(row.val[2])
		);

	cv::Matx31d col = src._external.col(3);
	dst.mPosition = aiVector3D(
			static_cast<float>(col.val[0]),
			static_cast<float>(col.val[1]),
			static_cast<float>(col.val[2])						
		);
}

// Weird workaround for 'LNK2019' caused by an absense of compiled aiScene 
// constructor/desctructor pair's code inside of prebuilt assimp bundle  
aiScene::aiScene()
{
	this->mAnimations		= NULL;
	this->mCameras			= NULL;
	this->mFlags			= 0x00;
	this->mLights			= NULL;
	this->mMaterials		= NULL;
	this->mMeshes			= NULL;
	this->mNumAnimations	= 0;
	this->mNumCameras		= 0;
	this->mNumLights		= 0;
	this->mNumMaterials		= 0;
	this->mNumMeshes		= 0;
	this->mNumTextures		= 0;
}
aiScene::~aiScene() { }

void SceneData::save(const std::vector<Camera> &cameras, const std::string &path) // id: "collada"
{
	std::vector<aiCamera>  scene_cameras_buffer(cameras.size());
	std::vector<aiCamera*> scene_cameras(scene_cameras_buffer.size());
	for (size_t i = 0, imax = cameras.size(); i < imax; ++i)
	{
		aiCamera *cam = new (&scene_cameras_buffer[i]) aiCamera;
		_cameraToAssimp(cameras[i], *cam);
		scene_cameras[i] = cam;
	}

	std::vector<aiVector3D> mesh_vertices(_mesh.cloud.width * _mesh.cloud.height);
	std::vector<aiVector3D> mesh_normals (mesh_vertices.size());
	for (uint32_t r = 0, idx = 0; r < _mesh.cloud.height; ++r)
	{
		uchar *row = &_mesh.cloud.data[r * _mesh.cloud.row_step];
		for (uint32_t c = 0; c < _mesh.cloud.width; ++c)
		{
			float *data = reinterpret_cast<float*>(row + c * _mesh.cloud.point_step);

			new (&mesh_vertices[idx]) aiVector3D(data[0], data[1], data[2]);
			new (&mesh_normals [idx]) aiVector3D(data[4], data[5], data[6]); // TODO: Check normals extraction
			++idx;
		}
	}

	std::vector<aiFace> mesh_faces(_mesh.polygons.size());
	for (size_t p = 0, pmax = _mesh.polygons.size(); p < pmax; ++p)
	{
		aiFace *face = new (&mesh_faces[p]) aiFace();

		std::vector<size_t> &vertices = _mesh.polygons[p].vertices;
		face->mIndices		= &vertices[0];
		face->mNumIndices	= std::min(size_t(AI_MAX_FACE_INDICES), vertices.size());	 
	}

	aiMesh scene_mesh;
	scene_mesh.mName		= "object mesh";
	scene_mesh.mFaces		= &mesh_faces[0];
	scene_mesh.mNumFaces	= mesh_faces.size();
	scene_mesh.mVertices	= &mesh_vertices[0];
	scene_mesh.mNormals		= &mesh_normals[0];
	scene_mesh.mNumVertices = mesh_vertices.size();
	aiMesh *scene_meshes[]  = { &scene_mesh };

	unsigned scene_mesh_idx[_countof(scene_meshes)] = { 0 };

	aiNode scene_root; 
	scene_root.mName		= "scene root node";
	scene_root.mNumMeshes	= _countof(scene_meshes);
	scene_root.mMeshes		= scene_mesh_idx;

	aiScene scene;
	scene.mFlags		= AI_SCENE_FLAGS_INCOMPLETE | AI_SCENE_FLAGS_NON_VERBOSE_FORMAT;
	scene.mCameras		= &scene_cameras[0];
	scene.mNumCameras	= scene_cameras.size();
	scene.mMeshes		= scene_meshes;
	scene.mNumMeshes	= _countof(scene_meshes);
	scene.mRootNode		= &scene_root;

	Assimp::Exporter e;

	const aiExportFormatDesc *export_description = e.GetExportFormatDescription(0);
	std::string export_path = path + '.' + export_description->fileExtension;
	e.Export(&scene, export_description->id, export_path); // TODO: Memory access violation bug fix
}

void SceneData::save(const MultiCameraPnP &sfm, const std::string &path)
{
	std::vector<std::pair<int, cv::Matx34d>> sfmExternal = sfm.getCameras();
	cv::Matx33d sfmInternal = sfm.getCameraMatrix();

	std::vector<SceneData::Camera> cameras;
	for (int i = 0, imax = sfmExternal.size(); i < imax; ++i)
	{
		cameras.push_back(Camera());

		std::pair<int, cv::Matx34d> &cam_src = sfmExternal[i];

		cv::Size viewportSize = sfm.get_im_orig(cam_src.first).size();
		cameras.back()._internal = cv::Matx33d(
				sfmInternal(0,0),	0.0,				viewportSize.width  / 2.0,
				0.0,				sfmInternal(1,1),	viewportSize.height / 2.0,
				0.0,				0.0,				1.0
			);
		
		stringstream cam_name; cam_name << "camera view #" << i;
		cameras.back()._name		= cam_name.str();
		cameras.back()._external	= cam_src.second;
	}

	save(cameras, path);
}

void SceneData::build(const std::vector<cv::Point3d> &cloudPoints, bool SORFilter)
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