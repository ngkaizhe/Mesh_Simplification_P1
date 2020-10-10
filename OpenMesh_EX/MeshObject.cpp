#include "MeshObject.h"


using namespace Eigen;

#define Quad
//#define Harmonic

struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

extern int faceSize;

#pragma region MyMesh

MyMesh::MyMesh()
{
	request_face_normals();
	request_vertex_normals();
	request_vertex_status();
	request_face_status();
	request_edge_status();
}

MyMesh::~MyMesh()
{

}

void MyMesh::ClearMesh()
{
	if (!faces_empty())
	{
		for (MyMesh::FaceIter f_it = faces_begin(); f_it != faces_end(); ++f_it)
		{
			delete_face(*f_it, true);
		}

		garbage_collection();
	}
}

#pragma endregion

#pragma region GLMesh

GLMesh::GLMesh()
{

}

GLMesh::~GLMesh()
{

}

bool GLMesh::Init(std::string fileName)
{
	if (LoadModel(fileName))
	{
		LoadToShader();
		return true;
	}
	return false;
}

void GLMesh::Render()
{
	glBindVertexArray(vao);
	glDrawElements(GL_TRIANGLES, mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}


bool GLMesh::LoadModel(std::string fileName)
{
	OpenMesh::IO::Options ropt;
	if (OpenMesh::IO::read_mesh(mesh, fileName, ropt))
	{
		if (!ropt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_vertex_normals())
		{
			mesh.request_face_normals();
			mesh.update_normals();
			mesh.release_face_normals();
		}

		return true;
	}

	return false;
}

void GLMesh::LoadToShader()
{
	std::vector<MyMesh::Point> vertices;
	vertices.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));

		MyMesh::Point p = mesh.point(*v_it);
	}

	std::vector<MyMesh::Normal> normals;
	normals.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		normals.push_back(mesh.normal(*v_it));
	}

	std::vector<unsigned int> indices;
	indices.reserve(mesh.n_faces() * 3);
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
	{
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
		{
			indices.push_back(fv_it->idx());
		}
	}

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vboVertices);
	glBindBuffer(GL_ARRAY_BUFFER, vboVertices);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &vboNormal);
	glBindBuffer(GL_ARRAY_BUFFER, vboNormal);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * normals.size(), &normals[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), &indices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

#pragma endregion

MeshObject::MeshObject()
{
	// add properties
	model.mesh.add_property(this->validVertices);
	model.mesh.add_property(this->quadricMat);
}

MeshObject::~MeshObject()
{
}

bool MeshObject::Init(std::string fileName)
{
	bool retV = model.Init(fileName);

	// initial the error quadric matrix
	for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); v_it++) {
		model.mesh.property(this->quadricMat, *v_it) = GetErrorQuadricMatrix(*v_it);
	}

	// initial vPairs and its cost
	for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); v_it++) {
		// the valid vertices array to be binded to the property
		std::vector<VertexCost> vertexCosts;

		// get the current vertex information
		MyMesh::Point vPF = model.mesh.point(*v_it);
		glm::vec4 v = glm::vec4(vPF[0], vPF[1], vPF[2], 1);
		// find the valid pairs by considering the (v1, v2) which is an edge
		for (MyMesh::VVIter vv_it = model.mesh.vv_begin(*v_it); vv_it.is_valid(); vv_it++) {
			VertexCost vertexCost;
			vertexCost.vhPtr = &(*vv_it);
			glm::mat4 Q = model.mesh.property(this->quadricMat, *v_it);

			// Cost/error value = (v(T) * M * v)
			// as glm doesn't provide vector * matrix from the left
			// so v(T) * M = (M(T) * v)(T) --> 1X4 vector
			// 1X4 vector * 4X1 vector = dot product
			vertexCost.cost = glm::dot(glm::transpose(Q) * v, v);

			// save to the array
			vertexCosts.push_back(vertexCost);
		}

		// sort the vertex cost array
		// the minimum will be the front
		std::sort(vertexCosts.begin(), vertexCosts.end(), 
			[](VertexCost vc1, VertexCost vc2) {
				return vc1.cost < vc2.cost;
			}
		);
	}

	return retV;
}

void MeshObject::Render(Shader shader)
{
	shader.use();

	glBindVertexArray(model.vao);
	glDrawElements(GL_TRIANGLES, model.mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

int MeshObject::GetVerticesNumber() {
	return model.mesh.n_vertices();
}

int MeshObject::GetEdgesNumber() {
	return model.mesh.n_edges();
}

int MeshObject::GetFacesNumber() {
	return model.mesh.n_faces();
}

void MeshObject::SimplifyMesh(SimplificationMode mode)
{
	
}

glm::mat4 MeshObject::GetErrorQuadricMatrix(OpenMesh::VertexHandle vh)
{
	// the summation fundamental error quadric
	glm::mat4 Kps = glm::mat4(0.0);
	for (MyMesh::VFIter vf_it = model.mesh.vf_begin(vh); vf_it != model.mesh.vf_end(vh); vf_it++) {
		// we need the plane equation of the face, 
		// get the plane normal
		// get the normal of face in the point form
		MyMesh::Point nPF = model.mesh.normal(*vf_it);
		// convert the normal into glm form and normalized it
		glm::vec3 n = glm::vec3(nPF[0], nPF[1], nPF[2]);
		// n = <a, b, c> which a^2 + b^2 + c^2 = 1
		n = glm::normalize(n);

		// get the point which is on the plane
		// point = (x0, y0, z0)
		MyMesh::Point vPF = model.mesh.point(vh);
		// plane equation -> ax + by + cz - (ax0 + by0 + cz0) = 0
		glm::vec4 p = glm::vec4(n[0], n[1], n[2], -(n[0] * vPF[0] + n[1] * vPF[1] + n[2] * vPF[2]));

		// get the fundamental error quadric
		glm::mat4 Kp = glm::outerProduct(p, p);
		Kps += Kp;
	}

	// return the error quadric
	return Kps;
}
