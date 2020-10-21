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
	//void LoadToShader();
};

class MeshObject
{
public:
	MeshObject();
	~MeshObject();

	bool Init(std::string fileName);
	void Render(Shader shader);

	int GetVerticesNumber();
	int GetEdgesNumber();
	int GetFacesNumber();


	/*  Skeleton Extraction*/
	void Parameterization();

	MyMesh::Point& GetLaplacianOperator(MyMesh& mesh, MyMesh::VertexIter& v_it, MyMesh::Point vi);

	double calcAreaOfThreePoints(MyMesh::Point& a, MyMesh::Point& b, MyMesh::Point& c);
		
	double GetOneRingArea(MyMesh& mesh, MyMesh::VertexIter& v_it, OpenMesh::FPropHandleT<double>& areaArr, OpenMesh::FPropHandleT<int>& timeId, int it);
	// decrease the vertex number 

private:
	GLMesh model;
	bool flag = false;
};

