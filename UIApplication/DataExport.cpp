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

// Weird workaround for 'LNK2019' caused by an absense of compiled aiScene 
// constructor/desctructor pair's code inside of prebuilt assimp bundle  
aiScene::aiScene()
{
	this->mFlags			= 0x00;
	this->mAnimations		= NULL;
	this->mCameras			= NULL;
	this->mLights			= NULL;
	this->mMaterials		= NULL;
	this->mTextures			= NULL;
	this->mPrivate			= NULL;
	this->mMeshes			= NULL;
	this->mRootNode			= NULL;
	this->mNumAnimations	= 0;
	this->mNumCameras		= 0;
	this->mNumLights		= 0;
	this->mNumMaterials		= 0;
	this->mNumMeshes		= 0;
	this->mNumTextures		= 0;
}
aiScene::~aiScene() { }

	//std::vector<aiCamera>  scene_cameras_buffer	(cameras.size());
	//std::vector<aiCamera*> scene_cameras		(cameras.size());
	//for (size_t i = 0, imax = cameras.size(); i < imax; ++i)
	//{
	//	const Camera	*src = &cameras[i];
	//	aiCamera		*dst = scene_cameras[i] = new (&scene_cameras_buffer[i]) aiCamera;

	//	dst->mAspect		= std::abs(static_cast<float>(src->_internal(1,2) / src->_internal(0,2)));
	//	dst->mHorizontalFOV	= std::abs(static_cast<float>(std::atan2(src->_internal(0,2), src->_internal(0,0))));
	//	dst->mClipPlaneFar	= static_cast<float>(std::max(src->_internal(0,0), src->_internal(1,1)));
	//	dst->mClipPlaneNear	= 1.0F;
	//	dst->mName			= src->_name;
	//	
	//	cv::Matx14d row; 
	//	cv::normalize(src->_external.row(2), row);
	//	dst->mLookAt = aiVector3D(
	//			static_cast<float>(row.val[0]),
	//			static_cast<float>(row.val[1]),
	//			static_cast<float>(row.val[2])
	//		);
	//	cv::normalize(src->_external.row(1), row);
	//	dst->mUp = aiVector3D(
	//			static_cast<float>(row.val[0]),
	//			static_cast<float>(row.val[1]),
	//			static_cast<float>(row.val[2])
	//		);
	//}

	//
	//std::vector<aiNode>  cam_nodes_buffer	(cameras.size());
	//std::vector<aiNode*> cam_nodes			(cameras.size());
	//for (size_t c = 0, cmax = cameras.size(); c < cmax; ++c)
	//{
	//	const Camera	*cam		= &cameras[c];
	//	aiNode			*cam_node	= cam_nodes[c] = new (&cam_nodes_buffer[c]) aiNode(scene_cameras[c]->mName.C_Str());

	//	cv::Matx31d col = cam->_external.col(3); 
	//	cam_node->mTransformation = aiMatrix4x4 ( 
	//					1.0F, 0.0F, 0.0F, static_cast<float>(col.val[0]),
	//					0.0F, 1.0F, 0.0F, static_cast<float>(col.val[1]),
	//					0.0F, 0.0F, 1.0F, static_cast<float>(col.val[0]),
	//					0.0F, 0.0F, 0.0F, 1.0F	    
	//				);
	//}

	//std::vector<aiVector3D> mesh_vertices(_mesh.cloud.width * _mesh.cloud.height);
	//std::vector<aiVector3D> mesh_normals (mesh_vertices.size());
	//for (uint32_t r = 0, idx = 0; r < _mesh.cloud.height; ++r)
	//{
	//	uchar *row = &_mesh.cloud.data[r * _mesh.cloud.row_step];
	//	for (uint32_t c = 0; c < _mesh.cloud.width; ++c)
	//	{
	//		float *data = reinterpret_cast<float*>(row + c * _mesh.cloud.point_step);

	//		new (&mesh_vertices[idx]) aiVector3D(data[0], data[1], data[2]);
	//		new (&mesh_normals [idx]) aiVector3D(data[4], data[5], data[6]);

	//		++idx;
	//	}
	//}

	//std::vector<aiFace> mesh_faces(_mesh.polygons.size());
	//for (size_t p = 0, pmax = _mesh.polygons.size(); p < pmax; ++p)
	//{
	//	aiFace *face = new (&mesh_faces[p]) aiFace();

	//	std::vector<size_t> &vertices = _mesh.polygons[p].vertices;
	//	face->mIndices		= &vertices[0];
	//	face->mNumIndices	= std::min(size_t(AI_MAX_FACE_INDICES), vertices.size());	 
	//}

	//aiMesh scene_mesh;
	//scene_mesh.mName		= "object_mesh";
	//scene_mesh.mFaces		= &mesh_faces[0];
	//scene_mesh.mNumFaces	= mesh_faces.size();
	//scene_mesh.mVertices	= &mesh_vertices[0];
	//scene_mesh.mNormals		= &mesh_normals[0];
	//scene_mesh.mNumVertices = mesh_vertices.size();
	//aiMesh *scene_meshes[]  = { &scene_mesh };

	//aiMaterial	scene_material_auto;
	//aiMaterial* scene_material = &scene_material_auto;

	//aiColor3D mesh_color_diffuse(212);
	//scene_material->AddProperty(&mesh_color_diffuse, 1, AI_MATKEY_COLOR_DIFFUSE); 

	//unsigned scene_mesh_idx[_countof(scene_meshes)] = { 0 };

	//aiNode scene_root;
	//scene_root.mName		= "scene_root_node";
	//scene_root.mChildren	= &cam_nodes[0];
	//scene_root.mNumChildren	= cam_nodes.size();
	//scene_root.mNumMeshes	= _countof(scene_meshes);
	//scene_root.mMeshes		= scene_mesh_idx;

	//aiScene scene;
	//scene.mFlags		= AI_SCENE_FLAGS_NON_VERBOSE_FORMAT;
	//scene.mCameras		= &scene_cameras[0];
	//scene.mNumCameras	= scene_cameras.size();
	//scene.mMeshes		= scene_meshes;
	//scene.mNumMeshes	= _countof(scene_meshes);
	//scene.mMaterials	= &scene_material;
	//scene.mNumMaterials	= 1;
	//scene.mRootNode		= &scene_root;


void SceneData::save(const std::vector<Camera> &cameras, const std::string &path)
{
	//std::vector<aiCamera>  scene_cameras_buffer	(cameras.size());
	//std::vector<aiCamera*> scene_cameras		(cameras.size());
	//for (size_t i = 0, imax = cameras.size(); i < imax; ++i)
	//{
	//	aiCamera *dst = scene_cameras[i] = new (&scene_cameras_buffer[i]) aiCamera;

	//	const Camera *src	= &cameras[i];
	//	dst->mName			= src->_name;
	//	dst->mAspect		= std::abs(static_cast<float>(src->_internal(1,2) / src->_internal(0,2)));
	//	dst->mHorizontalFOV	= std::abs(static_cast<float>(std::atan2(src->_internal(0,2), src->_internal(0,0))));
	//	dst->mClipPlaneFar	= static_cast<float>(std::max(src->_internal(0,0), src->_internal(1,1)));
	//	dst->mClipPlaneNear	= 1.0F;
	//	
	//	cv::Matx14d row; 
	//	cv::normalize(src->_external.row(2), row);
	//	dst->mLookAt = aiVector3D(
	//			static_cast<float>(row.val[0]),
	//			static_cast<float>(row.val[1]),
	//			static_cast<float>(row.val[2])
	//		);
	//	cv::normalize(src->_external.row(1), row);
	//	dst->mUp = aiVector3D(
	//			static_cast<float>(row.val[0]),
	//			static_cast<float>(row.val[1]),
	//			static_cast<float>(row.val[2])
	//		);
	//}

	aiNode scene_root;
	scene_root.mName		= "scene_root";
	//scene_root.mNumChildren	= cameras.size();
	//scene_root.mChildren	= new aiNode* [scene_root.mNumChildren];

	//for (size_t c = 0, cmax = cameras.size(); c < cmax; ++c)
	//{
	//	const Camera	*cam  = &cameras[c];
	//	aiNode			*node = scene_root.mChildren[c] 
	//						  = new aiNode(scene_cameras[c]->mName.C_Str());

	//	cv::Matx31d col = cam->_external.col(3); 
	//	aiMatrix4x4::Translation(aiVector3D(
	//			static_cast<float>(col.val[0]),
	//			static_cast<float>(col.val[1]),
	//			static_cast<float>(col.val[2])
	//		), node->mTransformation);
	//}

	scene_root.mNumMeshes	= _mesh.size();
	scene_root.mMeshes		= new unsigned[scene_root.mNumMeshes];

	std::vector<aiMesh>  scene_meshes_buffer (_mesh.size());
	std::vector<aiMesh*> scene_meshes		 (_mesh.size());
	for (size_t i = 0, imax = _mesh.size(); i < imax; ++i)
	{
		aiMesh *dst_mesh = scene_meshes[i] = new (&scene_meshes_buffer[i]) aiMesh;

		stringstream ss; ss << "object_segment" << i;
		dst_mesh->mName = ss.str();

		pcl::PolygonMesh *src_mesh	= &_mesh[i];
		dst_mesh->mNumVertices		= src_mesh->cloud.width * src_mesh->cloud.height;
		dst_mesh->mVertices			= new aiVector3D[dst_mesh->mNumVertices];
		dst_mesh->mNormals			= new aiVector3D[dst_mesh->mNumVertices];
		for (uint32_t r = 0, idx = 0; r < src_mesh->cloud.height; ++r)
		{
			uchar *row = &src_mesh->cloud.data[r * src_mesh->cloud.row_step];
			for (uint32_t c = 0; c < src_mesh->cloud.width; ++c)
			{
				float *data = reinterpret_cast<float*>(row + c * src_mesh->cloud.point_step);

				new (&dst_mesh->mVertices[idx]) aiVector3D(data[0], data[1], data[2]);
				new (&dst_mesh->mNormals [idx]) aiVector3D(data[4], data[5], data[6]);

				++idx;
			}
		}

		dst_mesh->mNumFaces			= src_mesh->polygons.size();
		dst_mesh->mFaces			= new aiFace[dst_mesh->mNumFaces];
		for (size_t p = 0; p < dst_mesh->mNumFaces; ++p)
		{
			aiFace *face = new (&dst_mesh->mFaces[p]) aiFace();

			std::vector<size_t> &vertices = src_mesh->polygons[p].vertices;
			face->mNumIndices	= std::min(size_t(AI_MAX_FACE_INDICES), vertices.size());	
			face->mIndices		= new unsigned[face->mNumIndices];

			std::copy(vertices.begin(), vertices.end(), face->mIndices);
		}

		scene_root.mMeshes[i] = i;
	}


	aiMaterial	scene_material_auto;
	aiMaterial* scene_material = &scene_material_auto;

	aiString  material_name("material_default");
	aiColor3D mesh_color_diffuse(212), mesh_color_specular(250);
	scene_material->AddProperty(&mesh_color_diffuse,  1, AI_MATKEY_COLOR_DIFFUSE);
	scene_material->AddProperty(&mesh_color_specular, 1, AI_MATKEY_COLOR_SPECULAR);
	scene_material->AddProperty(&material_name, AI_MATKEY_NAME);

	aiScene scene;
	scene.mFlags		= AI_SCENE_FLAGS_NON_VERBOSE_FORMAT;
	scene.mRootNode		= &scene_root;
	//scene.mCameras		= &scene_cameras[0];
	//scene.mNumCameras	= scene_cameras.size();
	scene.mMeshes		= &scene_meshes[0];
	scene.mNumMeshes	= scene_meshes.size();
	scene.mMaterials	= &scene_material;
	scene.mNumMaterials	= 1;
	
	// Export
	Assimp::Exporter e; const aiExportFormatDesc *export_description = e.GetExportFormatDescription(0); // 0 index means 'collada'

	std::string export_path	 = path;							if (export_path.back() == '/' || export_path.back() == '\\') { export_path += "scene_mesh"; }
	size_t		file_ext_pos = export_path.find_last_of('.');	if (file_ext_pos == export_path.length() - 1)				 { export_path.pop_back(); file_ext_pos = std::string::npos; }
	
	std::string format_ext = export_description->fileExtension;
	if (file_ext_pos == std::string::npos || export_path.substr(file_ext_pos + 1) != format_ext) export_path += format_ext;

	e.Export(&scene, export_description->id, export_path); // Currently does not export camera instances
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
		
		stringstream cam_name; cam_name << "camera_view" << i;
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
	gp3.setSearchRadius(2.525);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	_mesh.clear(); _mesh.resize(1);
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(_mesh[0]);
}