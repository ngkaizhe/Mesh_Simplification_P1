#pragma once
#include "Common.h"
#include "Shader.h"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;

class MyMesh : public TriMesh
{
public:
	MyMesh();
	~MyMesh();

	void ClearMesh();
};

class GLMesh
{
public:
	GLMesh();
	~GLMesh();

	bool Init(std::string fileName);
	void Render();

	MyMesh mesh;

	GLuint vao;
	GLuint ebo;
	GLuint vboVertices, vboNormal;

	GLuint lineVAO;

	void LoadToShader();

private:
	bool LoadModel(std::string fileName);
};

