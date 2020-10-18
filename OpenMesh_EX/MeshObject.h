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

	void ReLoadToShader();

private:

	bool LoadModel(std::string fileName);
	void LoadToShader();
};

// some extra enum and struct
enum class SimplificationMode {
	V1,
	V2,
	Middle,
	SmallestError,
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

	// debug
	void DebugRender(Shader shader);

	int GetVerticesNumber();
	int GetEdgesNumber();
	int GetFacesNumber();

	// decrease the vertex number
	void SimplifyMesh(SimplificationMode mode, int vertices_left);

private:
	// some added properties
	OpenMesh::VPropHandleT<glm::mat4> quadricMat;
	OpenMesh::EPropHandleT<double> cost;

	// heap with minimum cost in front
	std::vector<OpenMesh::EdgeHandle> heap;

	GLMesh model;

	void InitVerticesQuadratic();
	// helper function
	// get error quadratic matrix
	glm::mat4 GetErrorQuadricMatrix(OpenMesh::VertexHandle vh);
	// check whether the edge is an concave edge
	bool CheckConcave(OpenMesh::EdgeHandle eh);
	// set cost of edge handle with property
	void SetCost(MyMesh::EdgeHandle eh);
	// rearrange the heap
	void RearrangeHeap();

	// debug purpose
	void RecalculateCollapseVerticesToRender();
	std::vector<MyMesh::VertexHandle> CollapseVerticesToRender;
	bool CollapseRecalculated;
};

