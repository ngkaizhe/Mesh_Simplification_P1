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

	// setting the current model/skeleton mesh to be used
	void SetQEMRate(int rate);
	void SetSSMRate(int rate);

	// the current model to be rendered
	GLMesh* modelToRender;
	// the current SkeletonMesh to be rendered
	SkeletonMesh* skeletonMeshToRender;

#pragma region Get Mesh Info
	int GetVerticesNumber();
	int GetEdgesNumber();
	int GetFacesNumber();
	int GetModelsSize();
	int GetSkeletonMeshsSize();
#pragma endregion

#pragma region Skeleton Extraction
	void InitSE(int id);

	void ResetModel(int id = 0);

	/*  Skeleton Extraction*/
	void Parameterization();

	MyMesh::Point& GetLaplacianOperator(MyMesh& mesh, MyMesh::VertexIter& v_it, MyMesh::Point vi);

	double calcAreaOfThreePoints(MyMesh::Point& a, MyMesh::Point& b, MyMesh::Point& c);

	double GetOneRingArea(MyMesh& mesh, MyMesh::VertexIter& v_it);

	GLMesh SEModel;
	// decrease the vertex number 
#pragma endregion

private:
	// self timer
	MyTimer tGlobal;

	// the original model
	GLMesh model;

#pragma region Skeleton Extraction
	MyMesh mesh;
	double initPara = 0;
	double power = 7.0;
	double SL = 2.5;
	double W_L = 0;
	double W_H = 1.0;
	double totalArea = 0;
	int iterNum = 0;
	std::vector<double>ori_OneRing;

	OpenMesh::HPropHandleT<double> weight;
	OpenMesh::VPropHandleT<double> or_area;
	OpenMesh::VPropHandleT<int> matrixIndex;
	OpenMesh::VPropHandleT<int> outAreaIndex;

#pragma endregion



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
	void SimplifyMeshQEM(int vertices_left, int simplifyRate);
	// init vertices quadratic property
	void InitVerticesQuadratic();
	// check whether the edge is an concave edge
	bool CheckOk(OpenMesh::EdgeHandle eh);
	// set cost of edge handle with property
	void SetCost(MyMesh::EdgeHandle eh);
	// rearrange the heap
	void RearrangeHeap();

	// the model to be save in all rate
	std::vector<GLMesh> models;
	// mesh simplify rate(id)
	int currentQEMIDToRender;

#pragma endregion

#pragma region Mesh Simplification SSM
	// called the skeleton mesh initSSM
	void InitSSM();

	// the ssm part of skeleton mesh
	SkeletonMesh skeletonMesh;

	// the 100 skeletonMeshs
	std::vector<SkeletonMesh> skeletonMeshs;
	// mesh simplify rate(id)
	int currentSSMIDToRender;
#pragma endregion

	// debug purpose
	void QEMRecalculateCollapseVerticesToRender();
	void SSMRecalculateCollapseVerticesToRender();
	std::vector<MyMesh::VertexHandle> CollapseVerticesToRender;
	bool CollapseRecalculated;

	// debug used
	void SimplifyMeshQEMOnce();
	void SimplifyMeshSSMOnce();
};

