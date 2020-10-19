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

	this->InitVerticesQuadratic();

	// initial the cost of all edge handle
	for (MyMesh::EdgeIter e_it = model.mesh.edges_begin(); e_it != model.mesh.edges_end(); e_it++) {
		this->SetCost(*e_it);
	}

	// sort the heap
	this->RearrangeHeap();
	std::cout << "Init finish arrange heap\n";

	CollapseRecalculated = false;

	// start to initial the models
	this->InitModels();
	this->currentIDToRender = -1;
	this->SetRate(0);

	return retV;
}

void MeshObject::InitModels() {
	models.reserve(100);

	int lowestPercentage = 20;
	int lowestEdgeNumber = lowestPercentage / 100.0 * this->GetEdgesNumber();
	int highestEdgeNumber = this->GetEdgesNumber();
	int edgeDiff = highestEdgeNumber - lowestEdgeNumber;

	for (int i = 0; i < 100; i++) {
		// save the initial state first
		models.push_back(model);

		// for each rate we wish to decrease the original model
		this->SimplifyMesh(SimplificationMode::SmallestError, this->GetEdgesNumber() - (edgeDiff / 100));

		std::cout << "Model Simplification Rate " << i << "% Done\n";
	}
}

void MeshObject::InitVerticesQuadratic() {
	// initial the quadric matrix for each vertex to 0.0
	for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); v_it++) {
		model.mesh.property(this->quadricMat, *v_it) = glm::mat4(0.0);
	}

	// initial the quadric matrix for each vertices
	for (MyMesh::FaceIter f_it = model.mesh.faces_begin(); f_it != model.mesh.faces_end(); f_it++) {
		MyMesh::FaceHandle fh = *f_it;
		// run through each vertices the face have
		glm::mat4 Kp = glm::mat4(0.0);
		bool KpCalculated = false;
		for (MyMesh::FVCWIter fv_it = model.mesh.fv_cwbegin(fh); fv_it != model.mesh.fv_cwend(fh); fv_it++) {
			MyMesh::VertexHandle vh = *fv_it;

			// the Kp of same planes should be same
			if (!KpCalculated) {
				KpCalculated = true;
				// we need the plane equation of the face, 
				// get the plane normal
				// get the normal of face in the point form
				MyMesh::Point nPF = model.mesh.normal(fh);
				// convert the normal into glm form and normalized it
				glm::vec3 n = glm::vec3(nPF[0], nPF[1], nPF[2]);
				// n = <a, b, c> which a^2 + b^2 + c^2 = 1
				n = glm::normalize(n);

				// get the point which is on the plane(that is the center point)
				// point = (x0, y0, z0)
				MyMesh::Point vPF = model.mesh.point(vh);
				// plane equation -> ax + by + cz - (ax0 + by0 + cz0) = 0
				glm::vec4 p = glm::vec4(n[0], n[1], n[2], -(n[0] * vPF[0] + n[1] * vPF[1] + n[2] * vPF[2]));

				// get the fundamental error quadric
				Kp = glm::outerProduct(p, p);
			}

			// update each vertices' quadratic mat property
			model.mesh.property(this->quadricMat, vh) += Kp;
		}
	}
}

void MeshObject::Render(Shader shader)
{
	shader.use();

	// render the current rate to use
	glBindVertexArray(this->modelToRender->vao);
	glDrawElements(GL_TRIANGLES, this->modelToRender->mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void MeshObject::RenderPoint(Shader shader) {
	shader.use();

	glBindVertexArray(model.vao);
	glDrawElements(GL_POINTS, model.mesh.n_vertices(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

// debug used
void MeshObject::DebugRender(Shader shader) {
	if(!CollapseRecalculated)	this->RecalculateCollapseVerticesToRender();
	shader.use();

	std::vector<MyMesh::Point> vertices;
	vertices.reserve(CollapseVerticesToRender.size() * 3);
	for (int i = 0; i < CollapseVerticesToRender.size(); i++) {
		vertices.push_back(model.mesh.point(CollapseVerticesToRender[i]));
	}

	GLuint tempEbo;
	glGenBuffers(1, &tempEbo);
	glBindBuffer(GL_ARRAY_BUFFER, tempEbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	
	glDrawArrays(GL_POINTS, 0, vertices.size());
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// debug used
void MeshObject::RecalculateCollapseVerticesToRender() {
	CollapseVerticesToRender.clear();

	// for each edge collapse
	// might let our edge decreased by 3
	// get the edge handle from the first element of heap
	MyMesh::HalfedgeHandle heh;
	for (int i = 0; i < heap.size(); i++) {
		// we dont collapse the concave edge
		MyMesh::EdgeHandle eh = model.mesh.edge_handle(heap[i]._idx);
		if (!this->CheckConcave(eh)) {
			heh = model.mesh.halfedge_handle(eh, 0);
			break;
		}
	}

	// get the 2 vertice handles of the edge handle
	MyMesh::VertexHandle vh1 = model.mesh.to_vertex_handle(heh);

	for (MyMesh::VVCWIter vv_it = model.mesh.vv_cwbegin(vh1); vv_it != model.mesh.vv_cwend(vh1); vv_it++) {
		CollapseVerticesToRender.push_back(*vv_it);
	}

	CollapseRecalculated = true;
}

void MeshObject::SetRate(int rate) {
	// if the rate is different to our current rate change the pointer then
	if (rate != this->currentIDToRender) {
		this->currentIDToRender = rate;
		this->modelToRender = &models[this->currentIDToRender];
		this->modelToRender->LoadToShader();
	}
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

void MeshObject::SimplifyMesh(SimplificationMode mode, int edgesLeft)
{
	int heapID = 0;
	// recheck whether the we reached the total edges number should be
	while (model.mesh.n_edges() > edgesLeft) {
		std::cout << "\n\n============\n";
		std::cout << "Current Edge left = " << model.mesh.n_edges() << '\n';
		std::cout << "Target Edge left = " << edgesLeft << '\n';
		std::cout << "\n============\n";
		// for each edge collapse
		// might let our edge decreased by 3
		// get the edge handle from the first element of heap
		MyMesh::HalfedgeHandle heh;
		// as we update our heap only after the while loop finished
		for (; heapID < heap.size(); heapID++) {
			// we dont collapse the concave edge
			// we might need to check whether heap[i] out of bound or not
			if (heap[heapID]._idx < model.mesh.n_edges()) {
				MyMesh::EdgeHandle eh = model.mesh.edge_handle(heap[heapID]._idx);
				if (!this->CheckConcave(eh)) {
					heh = model.mesh.halfedge_handle(eh, 0);
					break;
				}
			}
			
		}

		// get the 2 vertice handles of the edge handle
		MyMesh::VertexHandle vh1 = model.mesh.to_vertex_handle(heh);
		MyMesh::VertexHandle vh2 = model.mesh.from_vertex_handle(heh);

		// find the newQ
		glm::mat4 newQ = model.mesh.property(this->quadricMat, vh1) + model.mesh.property(this->quadricMat, vh2);
		// the final vertex position to be
		glm::vec4 newV;

		// choose the different mode
		if (mode == SimplificationMode::SmallestError) {
			MyMesh::Point p1 = model.mesh.point(vh1);
			MyMesh::Point p2 = model.mesh.point(vh2);
			// set the differential matrix
			glm::mat4 differentialMat = newQ;
			// set the last row to (0, 0, 0, 1)
			differentialMat = glm::row(differentialMat, 3, glm::vec4(0, 0, 0, 1));

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

		// collapse the halfedge (vh2 -> vh1)
		model.mesh.collapse(heh);
		// set the new vertex point
		model.mesh.set_point(vh1, MyMesh::Point(newV[0], newV[1], newV[2]));
		// set the vh1 quadratic matrix
		model.mesh.property(this->quadricMat, vh1) = newQ;

		// recalculate the edge's cost around the vh1
		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh1); ve_it != model.mesh.ve_cwend(vh1); ve_it++) {
			MyMesh::EdgeHandle eh = *ve_it;
			this->SetCost(eh);
		}

		// straight up called garbage collection
		// to delete the edge and vertex we collapsed before
		model.mesh.garbage_collection();

		// need to recalculate the collapsed vertices again
		CollapseRecalculated = false;
	}

	// we rearrange our heap after each rate
	this->RearrangeHeap();

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

		// get the point which is on the plane(that is the center point)
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

bool MeshObject::CheckConcave(OpenMesh::EdgeHandle eh)
{
	return false;
}

void MeshObject::SetCost(MyMesh::EdgeHandle eh)
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
	// we will use the smallest error to define our error value
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

	// we recreate the heap
	heap.clear();
	heap.reserve(model.mesh.n_edges());

	// repush our edge handle into heap
	for (MyMesh::EdgeIter e_it = model.mesh.edges_begin(); e_it != model.mesh.edges_end(); e_it++) {
		MyMesh::EdgeHandle eh = *e_it;
		MeshObject::EdgeInfo edgeInfo;
		edgeInfo._idx = e_it->idx();
		edgeInfo.cost = model.mesh.property(this->cost, *e_it);

		// sort in the process of pushing
		heap.push_back(edgeInfo);
	}

	std::sort(heap.begin(), heap.end(), [](MeshObject::EdgeInfo ei1, MeshObject::EdgeInfo ei2) {
		return ei1.cost < ei2.cost;
	});
}