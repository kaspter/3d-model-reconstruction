
#include "DataExport.h"

#include <fstream>

#include <pcl/PolygonMesh.h>

#include <assimp/scene.h>
#include <assimp/camera.h>
#include <assimp/mesh.h>

#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>


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

namespace DataExport
{
	bool Collada(const MeshBuilder &builder, const std::string &path)
	{
		const std::vector<pcl::PolygonMesh> &_mesh = builder.get();
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

			std::stringstream ss; ss << "object_segment" << i;
			dst_mesh->mName = ss.str();

			const pcl::PolygonMesh *src_mesh_ptr	= &_mesh[i];
			dst_mesh->mNumVertices					= src_mesh_ptr->cloud.width * src_mesh_ptr->cloud.height;
			dst_mesh->mVertices						= new aiVector3D[dst_mesh->mNumVertices];
			dst_mesh->mNormals						= new aiVector3D[dst_mesh->mNumVertices];
			for (uint32_t r = 0, idx = 0; r < src_mesh_ptr->cloud.height; ++r)
			{
				const unsigned char *row = &src_mesh_ptr->cloud.data[r * src_mesh_ptr->cloud.row_step];
				for (uint32_t c = 0; c < src_mesh_ptr->cloud.width; ++c)
				{
					const float *data = reinterpret_cast<const float*>(row + c * src_mesh_ptr->cloud.point_step);

					new (&dst_mesh->mVertices[idx]) aiVector3D(data[0], data[1], data[2]);
					new (&dst_mesh->mNormals [idx]) aiVector3D(data[4], data[5], data[6]);

					++idx;
				}
			}

			dst_mesh->mNumFaces			= src_mesh_ptr->polygons.size();
			dst_mesh->mFaces			= new aiFace[dst_mesh->mNumFaces];
			for (size_t p = 0; p < dst_mesh->mNumFaces; ++p)
			{
				const std::vector<size_t> &vertices = src_mesh_ptr->polygons[p].vertices;

				aiFace *face		= new (&dst_mesh->mFaces[p]) aiFace();
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

		return e.Export(&scene, export_description->id, export_path) == AI_SUCCESS; // Currently does not export camera instances
	}
}