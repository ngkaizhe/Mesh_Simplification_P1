#pragma once

#include "Common.h"
#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Sparse>
#include <map>
#include <algorithm>
#include<Eigen/Sparse>
#include "Shader.h"
#include <vector>

enum class SimplificationMode {
	V1,
	V2,
	Middle,
	SmallestError,
};

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

private:

	bool LoadModel(std::string fileName);
	void LoadToShader();
};

class MeshObject
{
public:
	struct VertexCost {
		double cost;
		OpenMesh::VertexHandle vh;
	};

	MeshObject();
	~MeshObject();

	bool Init(std::string fileName);
	void Render(Shader shader);
	void RenderPoint(Shader shader);

	int GetVerticesNumber();
	int GetEdgesNumber();
	int GetFacesNumber();

	// decrease the vertex number
	void SimplifyMesh(SimplificationMode mode);

private:
	// some added properties
	OpenMesh::VPropHandleT<std::vector<VertexCost>> validVertices;
	OpenMesh::VPropHandleT<glm::mat4> quadricMat;

	GLMesh model;

	// helper function
	// get error quadratic matrix
	glm::mat4 GetErrorQuadricMatrix(OpenMesh::VertexHandle vh);
	// recalculate the cost of vPair for the vertex handle
	void RecalculateCost(OpenMesh::VertexHandle baseVh, OpenMesh::VertexHandle updatedVH= OpenMesh::VertexHandle());
	// check whether the vertex pair is an concave edge
	bool CheckConcave(OpenMesh::VertexHandle baseVH, OpenMesh::VertexHandle sideVH);
	// get the cost of sideVH and baseVH
	double GetCost(OpenMesh::VertexHandle baseVH, OpenMesh::VertexHandle sideVH);
};

