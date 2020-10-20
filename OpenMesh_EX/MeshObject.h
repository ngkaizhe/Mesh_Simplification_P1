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
#include <fstream>

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

	void LoadToShader();

private:
	bool LoadModel(std::string fileName);
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
	struct EdgeInfo {
		int _idx;
		double cost;
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

	void SetRate(int rate);

private:
	// some added properties
	OpenMesh::VPropHandleT<glm::mat4> quadricMat;
	OpenMesh::EPropHandleT<double> cost;

	// heap with minimum cost in front
	std::vector<EdgeInfo> heap;

	// the original model
	GLMesh model;
	// the model to be save in all rate
	std::vector<GLMesh> models;
	// the current model to be render
	GLMesh* modelToRender;
	// mesh simplify rate(id)
	int currentIDToRender;

	// file ofstream
	std::ofstream fileToWrite;

	// decrease the vertex number
	void SimplifyMesh(SimplificationMode mode, int vertices_left, int simplifyRate);

	// helper function for init
	// init all simplification rate models
	void InitModels();
	// init vertices quadratic property
	void InitVerticesQuadratic();
	// check whether the edge is an concave edge
	bool CheckOk(OpenMesh::EdgeHandle eh);
	// set cost of edge handle with property
	void SetCost(MyMesh::EdgeHandle eh);
	// rearrange the heap
	void RearrangeHeap();

	// debug purpose
	void RecalculateCollapseVerticesToRender();
	std::vector<MyMesh::VertexHandle> CollapseVerticesToRender;
	bool CollapseRecalculated;
};

