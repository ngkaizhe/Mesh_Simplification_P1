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
			mesh.request_vertex_status();;
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
	model.mesh.add_property(this->quadricMat);
	model.mesh.add_property(this->cost);
}

MeshObject::~MeshObject()
{
}

bool MeshObject::Init(std::string fileName)
{
	bool retV = model.Init(fileName);

	// initial the quadric matrix for each vertex
	for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); v_it++) {
		model.mesh.property(this->quadricMat, *v_it) = GetErrorQuadricMatrix(*v_it);
	}

	// initial vPairs and its cost
	for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); v_it++) {
		// the valid vertices array to be binded to the property
		std::vector<VertexCost> vertexCosts;

		// find the valid pairs by considering the (v1, v2) which is an edge
		for (MyMesh::VVIter vv_it = model.mesh.vv_begin(*v_it); vv_it.is_valid(); vv_it++) {
			// get the quadratic matrix of baseVH
			glm::mat4 Q = model.mesh.property(this->quadricMat, *v_it);
			// get the current sideVH information
			MyMesh::Point vPF = model.mesh.point(*vv_it);
			glm::vec4 v = glm::vec4(vPF[0], vPF[1], vPF[2], 1);

			VertexCost vertexCost;
			vertexCost.vh = *vv_it;

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

		// set the property values
		model.mesh.property(this->validVertices, *v_it) = vertexCosts;
	}

	// initial the cost of all edge handle
	for (MyMesh::EdgeIter e_it = model.mesh.edges_begin(); e_it != model.mesh.edges_end(); e_it++) {
		this->SetCost(*e_it, SimplificationMode::SmallestError);
		this->heap.push_back(*e_it);
	}

	// sort the heap
	this->RearrangeHeap();

	return retV;
}

void MeshObject::Render(Shader shader)
{
	shader.use();

	glBindVertexArray(model.vao);
	glDrawElements(GL_TRIANGLES, model.mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void MeshObject::RenderPoint(Shader shader) {
	shader.use();

	glBindVertexArray(model.vao);
	glDrawElements(GL_POINTS, model.mesh.n_vertices(), GL_UNSIGNED_INT, 0);
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
	// we use the skipping iterators, as we will ignore the vertices that were deleted
	for (MyMesh::VertexIter v_it = model.mesh.vertices_sbegin(); v_it != model.mesh.vertices_end(); v_it++) {
		// check whether the v_it already marked as delete then we will just skipped
		if (model.mesh.status(*v_it).deleted()) continue;

		// Find which vertex to be collapsed
		MyMesh::VertexHandle baseVH = *v_it;
		MyMesh::VertexHandle sideVH;

		std::vector<VertexCost> validVertices = model.mesh.property(this->validVertices, baseVH);
		// start to choose the sideVH
		for (int i = 0; i < validVertices.size();) {
			sideVH = validVertices[i].vh;

			// if the current chosen sideVH is not deleted
			if (sideVH.is_valid()) {
				// Check whether the edge whether is concave or not
				if (!this->CheckConcave(baseVH, sideVH) && !this->CheckConcave(sideVH, baseVH)) {
					break;
				}
				i++;
			}
			// else we will just pop off the deleted vertex
			else {
				// this part shouldn't be called
				validVertices.erase(validVertices.begin() + i);
			}
		}
		// case consider for we couldn't find any sideVH to be collapsed
		if (sideVH.is_valid()) {
			// we will change the value of baseVH
			glm::vec4 newV;
			// Q = Q1 + Q2;
			glm::mat4 newQ = model.mesh.property(this->quadricMat, baseVH) + model.mesh.property(this->quadricMat, sideVH);

			// choose the different mode
			if (mode == SimplificationMode::SmallestError) {
				// set the differential matrix
				glm::mat4 differentialMat = newQ;
				// set the last row to (0, 0, 0, 1)
				glm::row(differentialMat, 3, glm::vec4(0, 0, 0, 1));

				// if the differential matrix is invertible
				if (glm::determinant(differentialMat) != 0) {
					newV = glm::inverse(differentialMat) * glm::vec4(0, 0, 0, 1);
				}
				// else we just use the middle point
				else {
					MyMesh::Point p1 = model.mesh.point(baseVH);
					MyMesh::Point p2 = model.mesh.point(sideVH);
					MyMesh::Point newP = (p1 + p2) / 2.0f;
					newV = glm::vec4(newP[0], newP[1], newP[2], 1);
				}
			}
			// V = (V1 + V2) / 2
			else if (mode == SimplificationMode::Middle) {
				MyMesh::Point p1 = model.mesh.point(baseVH);
				MyMesh::Point p2 = model.mesh.point(sideVH);
				MyMesh::Point newP = (p1 + p2) / 2.0f;
				newV = glm::vec4(newP[0], newP[1], newP[2], 1);
			}
			// V = V1 
			else if (mode == SimplificationMode::V1) {
				MyMesh::Point p1 = model.mesh.point(baseVH);
				newV = glm::vec4(p1[0], p1[1], p1[2], 1);
			}
			// V = V2
			else if (mode == SimplificationMode::V2) {
				MyMesh::Point p1 = model.mesh.point(sideVH);
				newV = glm::vec4(p1[0], p1[1], p1[2], 1);
			}

			// till now we have a newV and newQ
			// assign the new value to baseVH
			model.mesh.property(this->quadricMat, baseVH) = newQ;
			model.mesh.set_point(baseVH, MyMesh::Point(newV[0], newV[1], newV[2]));

			// the processes below is to collapse the sideVH
			// pop off the sideVH from the baseVH's validVertices or
			// inform all vertices inside the baseVH's validVertices to update its cost value
			std::vector<VertexCost> baseVHValidVertices = model.mesh.property(this->validVertices, baseVH);
			for (int i = 0; i < baseVHValidVertices.size(); i++) {
				// pop off the sideVH from the baseVH's validVertices
				if (baseVHValidVertices[i].vh == sideVH) {
					baseVHValidVertices.erase(baseVHValidVertices.begin() + i);
				}

				// inform all vertices inside the baseVH's validVertices to update its cost value
				else {
					this->RecalculateCost(baseVHValidVertices[i].vh, baseVH);
					i++;
				}
			}
			model.mesh.property(this->validVertices, baseVH) = baseVHValidVertices;

			// inform all vertices inside the sideVH's validVertices to update its vhPtr and cost value
			std::vector<VertexCost> sideVHValidVertices = model.mesh.property(this->validVertices, sideVH);
			for (int i = 0; i < sideVHValidVertices.size(); i++) {
				MyMesh::VertexHandle sideVHVH = sideVHValidVertices[i].vh;
				std::vector<VertexCost> sideVHVHValidVertices = model.mesh.property(this->validVertices, sideVHVH);

				// go through the vertex cost inside sideVHVH
				for (int i = 0; i < sideVHVHValidVertices.size(); i++) {
					if (sideVHVHValidVertices[i].vh == sideVH){
						// update its vhPtr to baseVH
						sideVHVHValidVertices[i].vh = baseVH;
						// update its cost value
						this->RecalculateCost(sideVHVH, baseVH);
						break;
					}		
				}

				// assign back the vector
				model.mesh.property(this->validVertices, sideVHVH) = sideVHVHValidVertices;
			}

			// merge the sideVH's validVertices to baseVH's validVertices
			std::vector<VertexCost> newValidVertices;
			newValidVertices.reserve(baseVHValidVertices.size() + sideVHValidVertices.size());
			newValidVertices.insert(newValidVertices.end(), baseVHValidVertices.begin(), baseVHValidVertices.end());
			newValidVertices.insert(newValidVertices.end(), sideVHValidVertices.begin(), sideVHValidVertices.end());
			model.mesh.property(this->validVertices, baseVH) = newValidVertices;

			// update the baseVH's validVertices' cost value
			RecalculateCost(baseVH);

			// remove the sideVH from the mesh
			model.mesh.delete_vertex(sideVH, false);
		}
	}

	// straight up called garbage collection
	model.mesh.garbage_collection();
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

// recalculate all valid vertices cost of the current vertex handle
// or recalculate the current vh cost of the valid vertices of the vertex handle
void MeshObject::RecalculateCost(OpenMesh::VertexHandle parentVH, OpenMesh::VertexHandle childVH)
{
	// the valid vertices array to be binded to the property
	std::vector<VertexCost> vertexCosts = model.mesh.property(this->validVertices, parentVH);

	// get the current vertex information
	MyMesh::Point vPF = model.mesh.point(parentVH);
	glm::vec4 v = glm::vec4(vPF[0], vPF[1], vPF[2], 1);
	glm::mat4 Q = model.mesh.property(this->quadricMat, parentVH);

	// 2 conditions: either we recalculate all vertex handle, or we only calculate specific vertex handle
	// recalculate all vertex handle
	if (!childVH.is_valid()) {
		// recalculate all cost from the vertexCosts
		for (int i = 0; i < vertexCosts.size(); i++) {
			// Cost/error value = (v(T) * M * v)
			// as glm doesn't provide vector * matrix from the left
			// so v(T) * M = (M(T) * v)(T) --> 1X4 vector
			// 1X4 vector * 4X1 vector = dot product
			vertexCosts[i].cost = glm::dot(glm::transpose(Q) * v, v);
		}
	} 
	// only calculate specific vertex handle
	else {
		for (int i = 0; i < vertexCosts.size(); i++) {
			if (childVH == vertexCosts[i].vh) {
				// Cost/error value = (v(T) * M * v)
				// as glm doesn't provide vector * matrix from the left
				// so v(T) * M = (M(T) * v)(T) --> 1X4 vector
				// 1X4 vector * 4X1 vector = dot product
				vertexCosts[i].cost = glm::dot(glm::transpose(Q) * v, v);
				break;
			}
			
		}
	}

	// sort the vertex cost array
	// the minimum will be the front
	std::sort(vertexCosts.begin(), vertexCosts.end(),
		[](VertexCost vc1, VertexCost vc2) {
			return vc1.cost < vc2.cost;
		}
	);

	// assigned back the vertexCost to parentVH
	model.mesh.property(this->validVertices, parentVH) = vertexCosts;
}

bool MeshObject::CheckConcave(OpenMesh::VertexHandle baseVH, OpenMesh::VertexHandle sideVH)
{
	return false;
}

void MeshObject::SetCost(MyMesh::EdgeHandle eh, SimplificationMode mode)
{
	double cost;
	// find the halfEdge handle from the edge handle
	MyMesh::HalfedgeHandle heh = model.mesh.halfedge_handle(eh, 0);
	// get both vertices of the half edge
	MyMesh::VertexHandle vh1 = model.mesh.to_vertex_handle(heh);
	MyMesh::VertexHandle vh2 = model.mesh.from_vertex_handle(heh);
	// find the Q(Q1 + Q2)
	glm::mat4 newQ = model.mesh.property(this->quadricMat, vh1) + model.mesh.property(this->quadricMat, vh2);
	glm::vec4 newV;

	// find the collapse result(vertex position)
	// choose the different mode
	if (mode == SimplificationMode::SmallestError) {
		// set the differential matrix
		glm::mat4 differentialMat = newQ;
		// set the last row to (0, 0, 0, 1)
		glm::row(differentialMat, 3, glm::vec4(0, 0, 0, 1));

		// if the differential matrix is invertible
		if (glm::determinant(differentialMat) != 0) {
			newV = glm::inverse(differentialMat) * glm::vec4(0, 0, 0, 1);
		}
		// else we just use the middle point
		else {
			MyMesh::Point p1 = model.mesh.point(vh1);
			MyMesh::Point p2 = model.mesh.point(vh2);
			MyMesh::Point newP = (p1 + p2) / 2.0f;
			newV = glm::vec4(newP[0], newP[1], newP[2], 1);
		}
	}
	// V = (V1 + V2) / 2
	else if (mode == SimplificationMode::Middle) {
		MyMesh::Point p1 = model.mesh.point(vh1);
		MyMesh::Point p2 = model.mesh.point(vh2);
		MyMesh::Point newP = (p1 + p2) / 2.0f;
		newV = glm::vec4(newP[0], newP[1], newP[2], 1);
	}
	// V = V1 
	else if (mode == SimplificationMode::V1) {
		MyMesh::Point p1 = model.mesh.point(vh1);
		newV = glm::vec4(p1[0], p1[1], p1[2], 1);
	}
	// V = V2
	else if (mode == SimplificationMode::V2) {
		MyMesh::Point p1 = model.mesh.point(vh2);
		newV = glm::vec4(p1[0], p1[1], p1[2], 1);
	}

	// calculate the cost
	// Cost/error value = (v(T) * M * v)
	// as glm doesn't provide vector * matrix from the left
	// so v(T) * M = (M(T) * v)(T) --> 1X4 vector
	// 1X4 vector * 4X1 vector = dot product
	cost = glm::dot(glm::transpose(newQ) * newV, newV);
	
	// set the cost of the edge
	model.mesh.property(this->cost, eh) = cost;
}

void MeshObject::RearrangeHeap()
{
	MyMesh::EdgeHandle tempEh;
	int min;

	// selection sort
	// sort the heap cost array
	// the minimum will be the front
	for (int i = 0; i < heap.size() - 1; i++) {
		min = i;
		for (int j = i + 1; j < heap.size(); j++) {
			// exchange the index
			if (heap[j] < heap[min]) {
				min = j;
			}
		}

		// exchange the heap[i] and heap[min]
		tempEh = heap[i];
		heap[i] = heap[min];
		heap[min] = tempEh;
	}
}






