#pragma once
#include"GLMesh.h"

class Edge;

class Vertex {
public:
	Vertex() {
		_point = glm::vec3(0.0f);
		_idx = -1;
		_isDeleted = false;

		quadraticMat = glm::mat4(0.0f);
		totalDistanceOfOneRing = 0.0f;
	}

	Vertex(MyMesh::Point point, int idx) {
		_point = glm::vec3(point[0], point[1], point[2]);
		_idx = idx;
		_isDeleted = false;

		quadraticMat = glm::mat4(0.0f);
		totalDistanceOfOneRing = 0.0f;
	}

	bool operator==(const Vertex& v) {
		return _idx == v._idx;
	}

	glm::vec3 _point;
	std::vector<Vertex*> oneRingVertices;
	std::vector<Edge*> oneRingEdges;

	// property
	glm::mat4 quadraticMat;
	double totalDistanceOfOneRing;
	
	// var for debug
	int _idx;
	bool _isDeleted;
};

class Edge {
public:
	Edge() {
		_idx = -1;
		_isDeleted = false;
	}

	Edge(Vertex* i, Vertex* j, int idx) {
		_i = i;
		_j = j;

		_idx = idx;
		_isDeleted = false;
	}

	bool operator==(const Edge& e) {
		return _idx == e._idx;
	}

	Vertex* _i;
	Vertex* _j;

	// we save the 2 cost to represent the half edge fucntion
	double costIToJ;
	double costJToI;

	// var for debug
	int _idx;
	bool _isDeleted;
};

class SkeletonMesh
{
public:
	SkeletonMesh();
	SkeletonMesh(GLMesh mesh, std::map<int, bool>& _canCollapse);

	// render the line (currently)
	void Render(Shader shader);

	// init the SSM methd
	void InitSSM(GLMesh model);

	// simplify our mesh with ssm algorithm
	void SimplifyMeshSSMOnce();
	void SimplifyMeshSSM(int edgesToLeft);

	// print debug info
	void PrintSkeletonMeshInfo();

	// get the undeleted edges and vertices number
	int GetUndeletedEdgesNumber();
	int GetUndeletedVerticesNumber();

	// helper function to reload the shader
	void LoadToShader();

private:
	// we dont sort our vector, so our pointer wont get useless
	// when we delete some vertices / edges, we will just mark the boolean of isDeleted to true
	std::vector<Vertex> vertices;
	std::vector<Edge> edges;

	std::map<int, bool> _sk_canCollapse;
	// the edge id to get collapse next
	int edgeIDToBeCollapsed = -1;

	// helper function for finding the lowest cost to collapse
	void CalculateEdgeIDToBeCollapsed();
	//helper function for calculating shape cost and sampling cost
	float F(Vertex* i, Vertex* j);
	// helper function that is similiar for open mesh
	// get the edge from the given 2 vertices
	Edge* GetEdge(Vertex* i, Vertex* j);
	// collapse the given halfEdge
	void CollapseEdgeIToJ(Edge* edge);
	void CollapseEdgeJToI(Edge* edge);
	void CollapseEdge(Edge* edge, Vertex* i, Vertex* j);

	// this value check whether we init our sampling cost the first time or not
	bool isSamplingCostInit;

	// vao for rendering the line
	GLuint lineVAO;
};

