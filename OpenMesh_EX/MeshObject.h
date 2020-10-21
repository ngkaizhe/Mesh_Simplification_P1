#pragma once

#include "Common.h"
#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Sparse>
#include <map>
#include <vector>
#include <algorithm>
#include "Shader.h"
#include <fstream>
#include <math.h>

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
		EdgeInfo() {
			_idx = -1;
			_cost = -1;
		}

		EdgeInfo(int idx, double cost) {
			_idx = idx;
			_cost = cost;
		}

		int _idx;
		double _cost;

		bool operator<(const EdgeInfo& rhs) const {
			// if id is same, they are same edge info
			if (_idx == rhs._idx) return false;

			// if the cost is same, the lower id will be in front
			if (_cost == rhs._cost) return _idx < rhs._idx;

			// else just consider the cost value
			return _cost < rhs._cost;
		}
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


#pragma region Skeleton Extraction
	/*  Skeleton Extraction*/
	void Parameterization();

	MyMesh::Point& GetLaplacianOperator(MyMesh& mesh, MyMesh::VertexIter& v_it, MyMesh::Point vi);

	double calcAreaOfThreePoints(MyMesh::Point& a, MyMesh::Point& b, MyMesh::Point& c);

	double GetOneRingArea(MyMesh& mesh, MyMesh::VertexIter& v_it, OpenMesh::FPropHandleT<double>& areaArr, OpenMesh::FPropHandleT<int>& timeId, int it);
	// decrease the vertex number 
#pragma endregion

private:
	// some added properties
	OpenMesh::VPropHandleT<glm::mat4> quadricMat;
	OpenMesh::EPropHandleT<double> cost;

	// heap with minimum cost in front
	std::set<EdgeInfo> heap;

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

	int GetUndeletedFacesNumber();

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

