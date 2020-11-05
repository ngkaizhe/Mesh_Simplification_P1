#pragma once

#include "Common.h"
#include "Shader.h"
#include"SkeletonMesh.h"
#include"GLMesh.h"

#include <string>
#include <Eigen/Sparse>
#include <map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <math.h>

// current mode in
enum class Mode {
	// quadratic error metric
	QEM,
	// skeleton extraction
	SE,
	// shape, sampling cost mesh simplification
	SSM,
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
	void Render(Shader shader, Mode mode);

	// debug
	void DebugRender(Shader shader);
	void PrintSkeletonMeshInfo();

	void SetRate(int rate);

	// the current model to be render
	GLMesh* modelToRender;
	// decrease the vertex number
	
	void SimplifyMeshQEMOnce(SimplificationMode mode);
	void SimplifyMeshSSMOnce();

#pragma region Get Mesh Info
	int GetVerticesNumber();
	int GetEdgesNumber();
	int GetFacesNumber();
	int GetModelsSize();
#pragma endregion

#pragma region Skeleton Extraction
	void InitSE(int id);

	/*  Skeleton Extraction*/
	void Parameterization();

	MyMesh::Point& GetLaplacianOperator(MyMesh& mesh, MyMesh::VertexIter& v_it, MyMesh::Point vi);

	double calcAreaOfThreePoints(MyMesh::Point& a, MyMesh::Point& b, MyMesh::Point& c);

	double GetOneRingArea(MyMesh& mesh, MyMesh::VertexIter& v_it);
	// decrease the vertex number 
#pragma endregion

private:
	// self timer
	MyTimer tGlobal;

	// the original model
	GLMesh model;
	// the model to be save in all rate
	std::vector<GLMesh> models;
	// mesh simplify rate(id)
	int currentIDToRender;

#pragma region Mesh Simplification QEM
	// some added properties
	OpenMesh::VPropHandleT<glm::mat4> quadricMat;
	OpenMesh::EPropHandleT<double> cost;

	// heap with minimum cost in front
	std::set<EdgeInfo> heap;

	// init heap and quadratic
	// init all simplification rate models
	void InitQEM();

	// helper function for lazy deletion
	int GetUndeletedFacesNumber();
	// helper function for init
	// decrease the vertex number
	void SimplifyMeshQEM(SimplificationMode mode, int vertices_left, int simplifyRate);
	// init vertices quadratic property
	void InitVerticesQuadratic();
	// check whether the edge is an concave edge
	bool CheckOk(OpenMesh::EdgeHandle eh);
	// set cost of edge handle with property
	void SetCost(MyMesh::EdgeHandle eh);
	// rearrange the heap
	void RearrangeHeap();

#pragma endregion

#pragma region Mesh Simplification SSM
	// some added properties
	// optimization purpose: total distance of base vertex and its adjadent vertices
	OpenMesh::VPropHandleT<double> totalDistance;
	// halfedge cost
	OpenMesh::HPropHandleT<double> heCost;

	int lowestCostHalfEdgeID;

	void InitSSM();
	float F(MyMesh::HalfedgeHandle heh, bool isInit);

	// the ssm part of skeleton mesh
	SkeletonMesh skeletonMesh;
#pragma endregion

	// debug purpose
	void QEMRecalculateCollapseVerticesToRender();
	void SSMRecalculateCollapseVerticesToRender();
	std::vector<MyMesh::VertexHandle> CollapseVerticesToRender;
	bool CollapseRecalculated;
};

