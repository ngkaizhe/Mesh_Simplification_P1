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
		std::cout << "Load Model\n";
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

		//MyMesh::Point p = mesh.point(*v_it);
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
	tGlobal.Start();
	bool retV = model.Init(fileName);

	this->InitVerticesQuadratic();

	// initial the cost of all edge handle
	for (MyMesh::EdgeIter e_it = model.mesh.edges_begin(); e_it != model.mesh.edges_end(); e_it++) {
		this->SetCost(*e_it);
	}

	// sort the heap
	this->RearrangeHeap();
	std::cout << "Init finish arrange heap\n";
	tGlobal.Flag();

	CollapseRecalculated = false;

	// init our file ofstream
	// fileToWrite = std::ofstream("C:/Users/ngkaizhe/Desktop/OpenMesh_EX/Assets/Temp/normalDeletionBear.txt");

	// init modelToRender
	this->modelToRender = &model;

	// start to initial the models
	this->InitModels();
	this->currentIDToRender = -1;
	this->SetRate(0);

	// fileToWrite.close();

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
		GLMesh tModel = model;
		models.push_back(tModel);

		MyTimer tTemp;
		// for each rate we wish to decrease the original model
		tTemp.Start("Model Simplification Rate " + std::to_string(i) + "% Start");
		this->SimplifyMesh(SimplificationMode::SmallestError, this->model.mesh.n_faces() - (faceDiff / 100), i);
		tTemp.Flag("Model Simplification Rate " + std::to_string(i) + "% Done");
	}

	tGlobal.Flag("Simplify all model finish!");

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
	// fileToWrite << "Simplified Rate => " << simplifiedRate << "\n";

	std::set<EdgeInfo>::iterator s_it;
	// recheck whether the we reached the total edges number should be
	//while(model.mesh.n_edges() > edgesLeft) {
	while (this->GetUndeletedFacesNumber() > faceLeft) {

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

		MyMesh::EdgeHandle eh = model.mesh.edge_handle(s_it->_idx);
		// fileToWrite << "Edge handle with Id => " << eh.idx() << " has been chosen to collapse!\n";

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

	// fileToWrite << "\n";
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

double MeshObject::calcAreaOfThreePoints(MyMesh::Point& a, MyMesh::Point& b, MyMesh::Point& c) {
	double lenA = sqrt(pow(b[0] - c[0], 2) + pow(b[1] - c[1], 2) + pow(b[2] - c[2], 2));
	double lenB = sqrt(pow(a[0] - c[0], 2) + pow(a[1] - c[1], 2) + pow(a[2] - c[2], 2));
	double lenC = sqrt(pow(b[0] - a[0], 2) + pow(b[1] - a[1], 2) + pow(b[2] - a[2], 2));
	double S = (lenA + lenB + lenC)/2.0;
	double Area = sqrt(S * (S-lenA) * (S-lenB) * (S-lenC));
	//std::cout << "Area : " << Area << std::endl;
	return Area;
}

MyMesh::Point& MeshObject::GetLaplacianOperator(MyMesh& mesh, MyMesh::VertexIter& v_it, MyMesh::Point vi) {
	MyMesh::Point result = MyMesh::Point(0, 0, 0);
	int count = 0;
	for (MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
	{
		result += mesh.point(*vv_it);
		count++;
	}
	return vi - result / count;
}


double MeshObject::GetOneRingArea(MyMesh& mesh, MyMesh::VertexIter& v_it, OpenMesh::FPropHandleT<double>& areaArr, OpenMesh::FPropHandleT<int>& timeId, int it) {
	double area = 0;
	for (MyMesh::VertexFaceIter vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
	{
		MyMesh::FaceVertexIter  fv_it = mesh.fv_iter(*vf_it);
		MyMesh::Point& P = mesh.point(*fv_it);  ++fv_it;
		MyMesh::Point& Q = mesh.point(*fv_it);  ++fv_it;
		MyMesh::Point& R = mesh.point(*fv_it);
		double a = calcAreaOfThreePoints(P, Q, R);
		//if (mesh.property(areaArr, *vf_it) == 0 || mesh.property(timeId, *vf_it) != it) {
		mesh.property(areaArr, *vf_it) = a;
			//mesh.property(timeId, *vf_it) = it;
		//}
		
		area += a;
	}
	return area;
}

void MeshObject::Parameterization()
{
	std::cout << "Parameterization" << std::endl;
	int iterNum = 1;
	float SL = 0.9f;
	double W_L = 0;
	double totalArea = 0;
	/*
	MyMesh::Point P = MyMesh::Point(0, 0, 0);
	MyMesh::Point Q = MyMesh::Point(0, 0, 1);
	MyMesh::Point R = MyMesh::Point(1, 0, 0);

	std::cout << "Area : " << calcAreaOfThreePoints(P, Q, R) << std::endl;
	*/

	OpenMesh::HPropHandleT<double> weight;
	OpenMesh::FPropHandleT<double> area;
	OpenMesh::FPropHandleT<int> timeId;
	OpenMesh::VPropHandleT<double> or_area;
	OpenMesh::VPropHandleT<int> matrixIndex;

	MyMesh mesh = model.mesh;

	mesh.add_property(area, "FaceArea");
	mesh.add_property(timeId, "TimeId");
	mesh.add_property(or_area, "oneRingArea");
	mesh.add_property(weight, "halfedgeWeight");
	mesh.add_property(matrixIndex, "row");
	int count = 0;

	for (int it = 0; it < iterNum; it++) {
		std::cout << "Iter index : " << it << std::endl;

		//std::cout << "V count : " << count << std::endl;
		//mesh.request_vertex_texcoords2D();
		std::cout << "Start calculate weight!" << std::endl;
		//calculate weight
		MyMesh::HalfedgeHandle heh;
		for (MyMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
		{
			if (!mesh.is_boundary(*e_it))
			{
				GLdouble angle1, angle2, w;
				MyMesh::HalfedgeHandle _heh = mesh.halfedge_handle(*e_it, 0);
				MyMesh::Point pfrom = mesh.point(mesh.from_vertex_handle(_heh));
				MyMesh::Point pto = mesh.point(mesh.to_vertex_handle(_heh));
				MyMesh::Point po = mesh.point(mesh.opposite_vh(_heh));
				MyMesh::Point poo = mesh.point(mesh.opposite_he_opposite_vh(_heh));

				// weight cot(po -> form , po -> to) + cot(poo -> from, poo -> to)
				OpenMesh::Vec3d v1 = (OpenMesh::Vec3d)(po - pfrom);
				v1.normalize();
				OpenMesh::Vec3d v2 = (OpenMesh::Vec3d)(po - pto);
				v2.normalize();

				angle1 = std::acos(OpenMesh::dot(v1, v2));

				v1 = (OpenMesh::Vec3d)(poo - pfrom);
				v1.normalize();
				v2 = (OpenMesh::Vec3d)(poo - pto);
				v2.normalize();

				angle2 = std::acos(OpenMesh::dot(v1, v2));

				w = (1.0 / std::tan(angle1)) + (1.0 / std::tan(angle2));

				mesh.property(weight, _heh) = w;
				mesh.property(weight, mesh.opposite_halfedge_handle(_heh)) = w;
			}
			else
			{
				MyMesh::HalfedgeHandle _heh = mesh.halfedge_handle(*e_it, 0);
				mesh.property(weight, _heh) = 0;
				mesh.property(weight, mesh.opposite_halfedge_handle(_heh)) =0;
				/*
				//find select mesh boundary 
				if (!heh.is_valid())
				{
					heh = mesh.halfedge_handle(*e_it, 1);
				}*/
			}
		}
		std::cout << "Calculate weight finish!\n" << std::endl;

		std::cout << "Start calculate matrix size!" << std::endl;

		int fn = 0;
		// calculate matrix size
		if (it == 0) {
			for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
			{
				//mesh.property(matrixIndex, *v_it) = count++;
				if (mesh.is_boundary(*v_it))
				{
					mesh.property(matrixIndex, *v_it) = -1;
				}
				else
				{
					double onering_area = GetOneRingArea(mesh, v_it, area, timeId, it);
					mesh.property(or_area, *v_it) = onering_area;
					mesh.property(matrixIndex, *v_it) = count++;
				}
			}

			for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
				
				MyMesh::FaceVertexIter  fv_it = mesh.fv_iter(*f_it);
				MyMesh::Point& P = mesh.point(*fv_it);  ++fv_it;
				MyMesh::Point& Q = mesh.point(*fv_it);  ++fv_it;
				MyMesh::Point& R = mesh.point(*fv_it);
				//std::cout << "Face" <<fn << " (" << P << ")\n";
				//std::cout << "Face" << fn << " (" << Q << ")\n";
				//std::cout << "Face" << fn << " (" << Q << ")\n\n";

				totalArea += calcAreaOfThreePoints(P, Q, R);
				fn++;
			}
		}
		//std::cout << "total Area : " << totalArea << std::endl;
		//std::cout << "Fn : " << fn << std::endl;
		if (it == 0)
			W_L =  7000.0f*sqrt(totalArea/fn);
		else
			W_L = SL * W_L;
		std::cout << "W_L : " << W_L << std::endl;

		//std::cout << "W_L : " << W_L << std::endl;
	//	std::cout << "Matrix size is " << count << std::endl;
		std::cout << "Calculate matrix size finish!\n" << std::endl;

		std::cout << "Start fill matrix!" << std::endl;

		Eigen::SparseMatrix<double> A(2*count, count);

		//Eigen::MatrixXd A(2 * count, count);// = Eigen::MatrixXd(2 * count, count);
		Eigen::VectorXd BX(2*count);
		Eigen::VectorXd BY(2*count);
		Eigen::VectorXd BZ(2*count);
		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> linearSolver;
		//Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>> linearSolver;
		//Eigen::SparseQR<Eigen::SparseMatrix<double> > linearSolver;
		BX.setZero();
		BY.setZero();
		BZ.setZero();

		double W_H = 1.0;
		// fiil matrix
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			if (!mesh.is_boundary(*v_it))
			{
				//
				if (it == 0) {
					/*mesh.property(or_area, *v_it) = onering_area;
					W_L = 0.001 * sqrt(onering_area);
					W_H = 1.0;*/
				}
				else   //wH^t+1 = wH^t * sqrt(A^0 / A^t)
				{
					double onering_area = GetOneRingArea(mesh, v_it, area, timeId, it);
					W_H = sqrt(mesh.property(or_area, *v_it) / onering_area);
				}
				int i = mesh.property(matrixIndex, *v_it);
				double totalWeight = 0;
				
				for (MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
				{
					MyMesh::HalfedgeHandle _heh = mesh.find_halfedge(*v_it, *vv_it);
					double w = mesh.property(weight, _heh);
					if (mesh.is_boundary(*vv_it))
					{
						//
					}
					else
					{
						int j = mesh.property(matrixIndex, *vv_it);
						A.insert(i, j) = w * W_L;
						//A(i, j) = w * W_L;
						totalWeight += w;
					}
				}
				//www -= totalWeight;
				//std::cout << "www : " << www << std::endl;
				//A(i, i) = -totalWeight * W_L;
				A.insert(i, i) = -totalWeight * W_L;
				//std::cout << "W_H : " << W_H << std::endl;
				//A(count + i, i) = W_H;

				A.insert(count+i, i) = W_H;

				MyMesh::Point p = mesh.point(*v_it);
			
				BX[count+i] = W_H * p[0];
				BY[count + i] = W_H * p[1];
				BZ[count + i] = W_H * p[2];
			}
		}

		std::cout << "Fill matrix finish!\n" << std::endl;

		std::cout << "Start solve linear system!" << std::endl;
	//	M.compute(A.transpose()* A);
		//Eigen::VectorXd X = (A.transpose() * A).ldlt().solve(A.transpose()* BX);
		//Eigen::VectorXd Y = (A.transpose() * A).ldlt().solve(A.transpose() * BY);
		//Eigen::VectorXd Z = (A.transpose() * A).ldlt().solve(A.transpose() * BZ);
		BX = A.transpose() * BX;
		BY = A.transpose() * BY;
		BZ = A.transpose() * BZ;
		A = A.transpose() * A;

		A.makeCompressed();
		linearSolver.compute(A);
		Eigen::VectorXd X = linearSolver.solve(BX);
		Eigen::VectorXd Y = linearSolver.solve(BY);
		Eigen::VectorXd Z = linearSolver.solve(BZ);
		//ldlt.compute(A);
		//Eigen::VectorXd X = (A.transpose() * A).ldlt().solve(A.transpose() * BX);
		//Eigen::VectorXd Y = (A.transpose() * A).ldlt().solve(A.transpose() * BY);
		//Eigen::VectorXd Z = (A.transpose() * A).ldlt().solve(A.transpose() * BZ);

		//A.makeCompressed();
		//std::cout << "A : " << A << std::endl;
		//linearSolver.compute(A);
		//std::cout << A << "\n";
		//Eigen::VectorXd X = linearSolver.solve(BX);
		//Eigen::VectorXd Y = linearSolver.solve(BY);
		//Eigen::VectorXd Z = linearSolver.solve(BZ);
		
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			if (!mesh.is_boundary(*v_it)) {
				int i = mesh.property(matrixIndex, *v_it);
				mesh.set_point(*v_it, MyMesh::Point(X[i], Y[i], Z[i]));
			}
		}//
		//if (it == iterNum - 1) {
		for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); ++v_it)
		{
			int i = mesh.property(matrixIndex, *v_it);
				//std::cout << "Point [" << i <<"] = " << MyMesh::Point(X[i], Y[i], Z[i]) << std::endl;

			if (i >= count || i < 0) {
				std::cout << "Index out of range!! " << i << std::endl;
				continue;
			}
				//model.mesh.set_point(*v_it, MyMesh::Point(mX[0], mY[0], mZ[0]));
			model.mesh.set_point(*v_it, MyMesh::Point(X[i], Y[i], Z[i]));
		}//
		//}
		
		std::cout << "Solve linear system finish11!\n" << std::endl;

		// solve linear system
	}
	model.LoadToShader();
}