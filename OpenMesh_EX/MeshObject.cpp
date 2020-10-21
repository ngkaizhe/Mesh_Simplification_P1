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

	// init our file ofstream
	fileToWrite = std::ofstream("C:/Users/ngkaizhe/Desktop/OpenMesh_EX/Assets/Temp/normalDeletionBear.txt");

	// init modelToRender
	this->modelToRender = &model;

	// start to initial the models
	this->InitModels();
	this->currentIDToRender = -1;
	this->SetRate(0);

	fileToWrite.close();

	return retV;
}

void MeshObject::InitModels() {
	models.reserve(101);

	int lowestPercentage = 1;
	int lowestFaceNumber = lowestPercentage / 100.0 * this->model.mesh.n_faces();
	int highestFaceNumber = this->model.mesh.n_faces();
	int faceDiff = highestFaceNumber - lowestFaceNumber;

	for (int i = 0; i < 100; i++) {
		// save the initial state first
		models.push_back(model);

		// for each rate we wish to decrease the original model
		std::cout << "Model Simplification Rate " << i << "% Start\n";
		this->SimplifyMesh(SimplificationMode::SmallestError, this->model.mesh.n_faces() - (faceDiff / 100), i);
		std::cout << "Model Simplification Rate " << i << "% Done\n\n";
	}

	// save the final state
	models.push_back(model);
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
	for (std::set<EdgeInfo>::iterator s_it = heap.begin(); s_it != heap.end(); s_it++) {
		// we dont collapse the concave edge
		MyMesh::EdgeHandle eh = model.mesh.edge_handle(s_it->_idx);
		if (!this->CheckOk(eh)) {
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
	return modelToRender->mesh.n_vertices();
}

int MeshObject::GetEdgesNumber() {
	return modelToRender->mesh.n_edges();
}

int MeshObject::GetFacesNumber() {
	return modelToRender->mesh.n_faces();
}

void MeshObject::SimplifyMesh(SimplificationMode mode, int faceLeft, int simplifiedRate)
{
	fileToWrite << "Simplified Rate => " << simplifiedRate << "\n";

	std::set<EdgeInfo>::iterator s_it;
	// recheck whether the we reached the total edges number should be
	//while(model.mesh.n_edges() > edgesLeft) {
	while (this->GetUndeletedFacesNumber() > faceLeft) {
		/*std::cout << "\n\n============\n";
		std::cout << "Current Edge left = " << model.mesh.n_edges() << '\n';
		std::cout << "Target Edge left = " << edgesLeft << '\n';
		std::cout << "============\n";*/

		// for each edge collapse
		// might let our edge decreased by 3
		// get the edge handle from the first element of heap
		MyMesh::HalfedgeHandle heh;
		// as we update our heap only after the while loop finished
		for (s_it = heap.begin(); s_it != heap.end(); s_it++) {
			MyMesh::EdgeHandle eh = model.mesh.edge_handle(s_it->_idx);
			// as we dont do garbage collection every loop, 
			// so we need to check whether the first position of heap can be used or not
			if (this->CheckOk(eh)) {
				heh = model.mesh.halfedge_handle(eh, 0);
				break;
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

		// save the changed vertex id
		std::vector<EdgeInfo> costChangedVerticesID;

		// collapse the halfedge (vh2 -> vh1)
		model.mesh.collapse(heh);
		// set the new vertex point
		model.mesh.set_point(vh1, MyMesh::Point(newV[0], newV[1], newV[2]));
		// set the vh1 quadratic matrix
		model.mesh.property(this->quadricMat, vh1) = newQ;

		// recalculate the edge's cost around the vh1
		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh1); ve_it != model.mesh.ve_cwend(vh1); ve_it++) {
			MyMesh::EdgeHandle eh = *ve_it;

			costChangedVerticesID.push_back(EdgeInfo(eh.idx(), model.mesh.property(this->cost, eh)));

			this->SetCost(eh);
		}

		// need to recalculate the collapsed vertices again
		CollapseRecalculated = false;

		// we use lazy deletion
		// for not resorting our heap each time we deleted 1 edge
		// change the heap's value which has cost has changed
		// erase the selected id first
		for (std::vector<EdgeInfo>::iterator v_it = costChangedVerticesID.begin(); v_it != costChangedVerticesID.end(); v_it++) {
			heap.erase(*v_it);
		}

		// insert back the selected id
		for (std::vector<EdgeInfo>::iterator v_it = costChangedVerticesID.begin(); v_it != costChangedVerticesID.end(); v_it++) {
			EdgeInfo ei = *v_it;
			ei._cost = model.mesh.property(this->cost, model.mesh.edge_handle(ei._idx));

			heap.insert(ei);
		}
	}
	// straight up called garbage collection
	// to delete the edge and vertex we collapsed before
	model.mesh.garbage_collection();

	// we rearrange our heap after each rate
	this->RearrangeHeap();

	fileToWrite << "\n";
}

int MeshObject::GetUndeletedFacesNumber() {
	int i = 0;
	for (MyMesh::FaceIter f_it = model.mesh.faces_sbegin(); f_it != model.mesh.faces_end(); f_it++) {
		i++;
	}
	return i;
}

bool MeshObject::CheckOk(OpenMesh::EdgeHandle eh)
{
	if (model.mesh.status(eh).deleted()) {
		return false;
	}

	else {
		MyMesh::HalfedgeHandle heh = model.mesh.halfedge_handle(eh, 0);
		return model.mesh.is_collapse_ok(heh) && !model.mesh.is_boundary(eh);
	}
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

	// calculate the cost
	// Cost/error value = (v(T) * M * v)
	// as glm doesn't provide vector * matrix from the left
	// so v(T) * M = (M(T) * v)(T) --> 1X4 vector
	// 1X4 vector * 4X1 vector = dot product
	cost = glm::dot(glm::transpose(newQ) * newV, newV);

	float x = newV.x;
	float y = newV.y;
	float z = newV.z;
	double cost2 = (newQ[0][0] * x * x) + (2 * newQ[0][1] * x * y) + (2 * newQ[0][2] * x * z) + (2 * newQ[0][3] * x) + (newQ[1][1] * y * y) +
		(2 * newQ[1][2] * y * z) + (2 * newQ[1][3] * y) + (newQ[2][2] * z * z) + (2 * newQ[2][3] * z) + (newQ[3][3]);
	
	// set the cost of the edge
	model.mesh.property(this->cost, eh) = cost;
}

void MeshObject::RearrangeHeap()
{
	MyMesh::EdgeHandle tempEh;
	int min;

	// we recreate the heap
	heap.clear();

	// repush our edge handle into heap
	for (MyMesh::EdgeIter e_it = model.mesh.edges_begin(); e_it != model.mesh.edges_end(); e_it++) {
		MyMesh::EdgeHandle eh = *e_it;
		MeshObject::EdgeInfo edgeInfo;
		edgeInfo._idx = e_it->idx();
		edgeInfo._cost = model.mesh.property(this->cost, *e_it);

		// sort in the process of pushing
		heap.insert(edgeInfo);
	}
}