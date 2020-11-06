#include "MeshObject.h"

using namespace Eigen;

#define Quad
//#define Harmonic


MeshObject::MeshObject()
{
	// add properties
	model.mesh.add_property(this->quadricMat);
	model.mesh.add_property(this->cost);
}

MeshObject::~MeshObject()
{
	// remove properties
	model.mesh.remove_property(this->quadricMat);
	model.mesh.remove_property(this->cost);
}

bool MeshObject::Init(std::string fileName)
{
	tGlobal.Start();
	bool retV = model.Init(fileName);

	// init modelToRender
	this->modelToRender = &model;

	// init models
	models.clear();
	models.reserve(101);

	// start to initial the qem model
	// this->InitQEM();
	// start to initial the parameterization
	// this->InitSE();
	// start to initial the ssm model
	this->InitSSM();

	CollapseRecalculated = false;

	return retV;
}

void MeshObject::Render(Shader shader, Mode mode)
{
	if (mode == Mode::QEM || mode == Mode::SE) {
		// render face part
		shader.setUniform3fv("color", glm::vec3(1.0f, 1.0f, 0.0f));
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		shader.use();
		// render the current rate to use
		glBindVertexArray(this->modelToRender->vao);
		glDrawElements(GL_TRIANGLES, this->modelToRender->mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);


		// render line / wireframe part
		shader.setUniform3fv("color", glm::vec3(0.0f, 0.0f, 0.0f));
		shader.use();
		glBindVertexArray(this->modelToRender->lineVAO);
		glDrawArrays(GL_LINES, 0, this->modelToRender->mesh.n_edges() * 2);
		glBindVertexArray(0);
	}
	
	else if (mode == Mode::SSM) {
		// render line but with our skeleton mesh
		shader.setUniform3fv("color", glm::vec3(0.0f, 0.0f, 0.0f));
		skeletonMeshToRender->Render(shader);
	}
}

// debug used
void MeshObject::DebugRender(Shader shader) {
	if(!CollapseRecalculated)	this->SSMRecalculateCollapseVerticesToRender();
	shader.use();

	std::vector<MyMesh::Point> vertices;
	vertices.reserve(CollapseVerticesToRender.size());
	for (int i = 0; i < CollapseVerticesToRender.size(); i++) {
		vertices.push_back(model.mesh.point(CollapseVerticesToRender[i]));
	}

	if (vertices.size() != 0) {
		GLuint tempEbo;
		glGenBuffers(1, &tempEbo);
		glBindBuffer(GL_ARRAY_BUFFER, tempEbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		glDrawArrays(GL_POINTS, 0, vertices.size());
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	
}

void MeshObject::PrintSkeletonMeshInfo() {
	this->skeletonMeshToRender->PrintSkeletonMeshInfo();
}

// debug used
void MeshObject::QEMRecalculateCollapseVerticesToRender() {
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

void MeshObject::SSMRecalculateCollapseVerticesToRender() {
	// do nothing as we have skelenton for ssm
}

// for ui to used to render
void MeshObject::SetQEMRate(int rate) {
	// if the rate is different to our current rate change the pointer then
	if (rate != this->currentQEMIDToRender) {
		this->currentQEMIDToRender = rate;
		this->modelToRender = &models[this->currentQEMIDToRender];
	}
}

void MeshObject::SetSSMRate(int rate) {
	// if the rate is different to our current rate change the pointer then
	if (rate != this->currentSSMIDToRender) {
		this->currentSSMIDToRender = rate;
		this->skeletonMeshToRender = &skeletonMeshs[this->currentSSMIDToRender];
	}
}

#pragma region Get Mesh Info
int MeshObject::GetVerticesNumber() {
	return modelToRender->mesh.n_vertices();
}

int MeshObject::GetEdgesNumber() {
	return modelToRender->mesh.n_edges();
}

int MeshObject::GetFacesNumber() {
	return modelToRender->mesh.n_faces();
}

int MeshObject::GetModelsSize() {
	return this->models.size();
}

int MeshObject::GetSkeletonMeshsSize() {
	return this->skeletonMeshs.size();
}

#pragma endregion

#pragma region Mesh Simplification QEM
void MeshObject::InitQEM() {
	// init properties
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

	// init modelToRender
	this->modelToRender = &model;

	// start to initial the models
	models.clear();
	models.reserve(101);

	// save the initial state first
	GLMesh tModel = model;
	tModel.mesh.garbage_collection();
	tModel.LoadToShader();
	models.push_back(tModel);

	int lowestPercentage = 1;
	int lowestFaceNumber = lowestPercentage / 100.0 * this->model.mesh.n_faces();
	int highestFaceNumber = this->model.mesh.n_faces();
	int faceDiff = highestFaceNumber - lowestFaceNumber;

	for (int i = 0; i < 100; i++) {
		// MyTimer tTemp;
		// for each rate we wish to decrease the original model
		// tTemp.Start("Model Simplification Rate " + std::to_string(i) + "% Start");
		std::cout << "Model Simplification QEM Rate " + std::to_string(i) + "% Start\n";
		this->SimplifyMeshQEM(this->GetUndeletedFacesNumber() - (faceDiff / 100), i);
		//tTemp.Flag("Model Simplification Rate " + std::to_string(i) + "% Done");
		std::cout << "Model Simplification QEM Rate " + std::to_string(i) + "% Done\n\n";

		// save the final state
		GLMesh tModel = model;
		tModel.mesh.garbage_collection();
		tModel.LoadToShader();
		models.push_back(tModel);
	}

	tGlobal.Flag("Simplify all model finish!");

	this->currentQEMIDToRender = -1;
	this->SetQEMRate(0);
	model.mesh.garbage_collection();
}

void MeshObject::InitVerticesQuadratic() {
	// initial the quadric matrix for each vertex to 0.0
	for (MyMesh::VertexIter v_it = model.mesh.vertices_sbegin(); v_it != model.mesh.vertices_end(); v_it++) {
		model.mesh.property(this->quadricMat, *v_it) = glm::mat4(0.0);
	}

	// initial the quadric matrix for each vertices
	for (MyMesh::FaceIter f_it = model.mesh.faces_sbegin(); f_it != model.mesh.faces_end(); f_it++) {
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

void MeshObject::SimplifyMeshQEMOnce() {
	model.mesh.garbage_collection();
	RearrangeHeap();
	modelToRender = &model;

	std::set<EdgeInfo>::iterator s_it;
	bool once = true;
	// recheck whether the we reached the total edges number should be
	while (once) {
		once = false;
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

		// find the newV
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


		// save the edge id around vh1 and vh2
		std::vector<EdgeInfo> edgesAroundVh1;
		std::vector<EdgeInfo> edgesAroundVh2;

		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh1); ve_it != model.mesh.ve_cwend(vh1); ve_it++) {
			EdgeInfo ei;
			ei._idx = ve_it->idx();
			ei._cost = model.mesh.property(this->cost, *ve_it);

			edgesAroundVh1.push_back(ei);
		}

		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh2); ve_it != model.mesh.ve_cwend(vh2); ve_it++) {
			EdgeInfo ei;
			ei._idx = ve_it->idx();
			ei._cost = model.mesh.property(this->cost, *ve_it);

			edgesAroundVh2.push_back(ei);
		}

		MyMesh::EdgeHandle eh = model.mesh.edge_handle(s_it->_idx);
		// fileToWrite << "Edge handle with Id => " << eh.idx() << " has been chosen to collapse!\n";

		// collapse the halfedge (vh2 -> vh1)
		model.mesh.collapse(heh);
		// set the new vertex point
		model.mesh.set_point(vh1, MyMesh::Point(newV[0], newV[1], newV[2]));
		// set the vh1 quadratic matrix
		model.mesh.property(this->quadricMat, vh1) = newQ;

		// recalculate the edge's cost around the result vh1
		std::vector<EdgeInfo> edgesAroundResultVh1;
		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh1); ve_it != model.mesh.ve_cwend(vh1); ve_it++) {
			MyMesh::EdgeHandle eh = *ve_it;

			EdgeInfo ei;
			ei._idx = eh.idx();
			ei._cost = model.mesh.property(this->cost, eh);
			edgesAroundResultVh1.push_back(ei);

			this->SetCost(eh);
		}

		// need to recalculate the collapsed vertices again
		CollapseRecalculated = false;


		if (false) {
			// we use lazy deletion
		// for not resorting our heap each time we deleted 1 edge
		// change the heap's value which has cost has changed
		// erase the selected id first
			for (std::vector<EdgeInfo>::iterator v_it = edgesAroundVh1.begin(); v_it != edgesAroundVh1.end(); v_it++) {
				heap.erase(*v_it);
			}
			for (std::vector<EdgeInfo>::iterator v_it = edgesAroundVh2.begin(); v_it != edgesAroundVh2.end(); v_it++) {
				heap.erase(*v_it);
			}

			// insert back the changed cost value edge id
			for (std::vector<EdgeInfo>::iterator v_it = edgesAroundResultVh1.begin(); v_it != edgesAroundResultVh1.end(); v_it++) {
				EdgeInfo ei = *v_it;
				ei._cost = model.mesh.property(this->cost, model.mesh.edge_handle(ei._idx));

				heap.insert(ei);
			}
		}
	}

	modelToRender->mesh.garbage_collection();
	modelToRender->LoadToShader();

}

void MeshObject::SimplifyMeshQEM(int faceLeft, int simplifiedRate)
{
	std::set<EdgeInfo>::iterator s_it;
	// recheck whether the we reached the total edges number should be
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


		// find the newV
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


		// save the edge id around vh1 and vh2
		std::vector<EdgeInfo> edgesAroundVh1;
		std::vector<EdgeInfo> edgesAroundVh2;

		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh1); ve_it != model.mesh.ve_cwend(vh1); ve_it++) {
			EdgeInfo ei;
			ei._idx = ve_it->idx();
			ei._cost = model.mesh.property(this->cost, *ve_it);

			edgesAroundVh1.push_back(ei);
		}

		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh2); ve_it != model.mesh.ve_cwend(vh2); ve_it++) {
			EdgeInfo ei;
			ei._idx = ve_it->idx();
			ei._cost = model.mesh.property(this->cost, *ve_it);

			edgesAroundVh2.push_back(ei);
		}

		MyMesh::EdgeHandle eh = model.mesh.edge_handle(s_it->_idx);
		// fileToWrite << "Edge handle with Id => " << eh.idx() << " has been chosen to collapse!\n";

		// collapse the halfedge (vh2 -> vh1)
		model.mesh.collapse(heh);
		// set the new vertex point
		model.mesh.set_point(vh1, MyMesh::Point(newV[0], newV[1], newV[2]));
		// set the vh1 quadratic matrix
		model.mesh.property(this->quadricMat, vh1) = newQ;

		// recalculate the edge's cost around the result vh1
		std::vector<EdgeInfo> edgesAroundResultVh1;
		for (MyMesh::VertexEdgeCWIter ve_it = model.mesh.ve_cwbegin(vh1); ve_it != model.mesh.ve_cwend(vh1); ve_it++) {
			MyMesh::EdgeHandle eh = *ve_it;

			EdgeInfo ei;
			ei._idx = eh.idx();
			ei._cost = model.mesh.property(this->cost, eh);
			edgesAroundResultVh1.push_back(ei);

			this->SetCost(eh);
		}

		// need to recalculate the collapsed vertices again
		CollapseRecalculated = false;

		bool useLazyDeletion = true;
		if (useLazyDeletion) {
			// we use lazy deletion
			// for not resorting our heap each time we deleted 1 edge
			// change the heap's value which has cost has changed
			// erase the selected id first
			for (std::vector<EdgeInfo>::iterator v_it = edgesAroundVh1.begin(); v_it != edgesAroundVh1.end(); v_it++) {
				heap.erase(*v_it);
			}
			for (std::vector<EdgeInfo>::iterator v_it = edgesAroundVh2.begin(); v_it != edgesAroundVh2.end(); v_it++) {
				heap.erase(*v_it);
			}

			// insert back the changed cost value edge id
			for (std::vector<EdgeInfo>::iterator v_it = edgesAroundResultVh1.begin(); v_it != edgesAroundResultVh1.end(); v_it++) {
				EdgeInfo ei = *v_it;
				ei._cost = model.mesh.property(this->cost, model.mesh.edge_handle(ei._idx));

				heap.insert(ei);
			}
		}
	}

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

	//float x = newV.x;
	//float y = newV.y;
	//float z = newV.z;
	//double cost2 = (newQ[0][0] * x * x) + (2 * newQ[0][1] * x * y) + (2 * newQ[0][2] * x * z) + (2 * newQ[0][3] * x) + (newQ[1][1] * y * y) +
	//	(2 * newQ[1][2] * y * z) + (2 * newQ[1][3] * y) + (newQ[2][2] * z * z) + (2 * newQ[2][3] * z) + (newQ[3][3]);
	
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

#pragma endregion

#pragma region Skeleton Extraction
void MeshObject::InitSE(int id) {
	if (models.size() > id) {
		this->model = models[id];
	}

	Parameterization();
	this->model.mesh.garbage_collection();
	this->model.LoadToShader();

	this->modelToRender = &model;
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

double MeshObject::GetOneRingArea(MyMesh& mesh, MyMesh::VertexIter& v_it) {
	double area = 0;
	for (MyMesh::VertexFaceIter vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
	{
		MyMesh::FaceVertexIter  fv_it = mesh.fv_iter(*vf_it);
		MyMesh::Point& P = mesh.point(*fv_it);  ++fv_it;
		MyMesh::Point& Q = mesh.point(*fv_it);  ++fv_it;
		MyMesh::Point& R = mesh.point(*fv_it);
		double a = calcAreaOfThreePoints(P, Q, R);
		area += a;
	}
	if (!isfinite(area))
		return 0.00000001;
	return area;
}

void MeshObject::Parameterization()
{
	std::cout << "Parameterization" << std::endl;
	/* Best Case
	iterNum = 2
	power = 7.0f
	SL = 2.5f
	*/
	int iterNum = 2;
	double power = 7.0f;
	float SL = 2.5f;
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

		double onering_area = GetOneRingArea(mesh, mesh.vertices_begin());
		if (!isfinite(onering_area))
			break;
		std::cout << "onering_area : " << onering_area << std::endl;
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
					double onering_area = GetOneRingArea(mesh, v_it);
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
			W_L =  power * sqrt(totalArea/fn);
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
					double onering_area = GetOneRingArea(mesh, v_it);
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
}

#pragma endregion

#pragma region Mesh Simplification SSM
void MeshObject::InitSSM() {
	model.mesh.garbage_collection();
	skeletonMesh = SkeletonMesh(model);

	skeletonMeshs.clear();
	skeletonMeshs.reserve(101);

	SkeletonMesh tempSkeletonMesh = skeletonMesh;
	tempSkeletonMesh.LoadToShader();
	skeletonMeshs.push_back(tempSkeletonMesh);

	// the lowest total edge numbers our aim to be
	int lowestTotalEdgeNumber = 10;
	int currentTotalEdgeNumber = skeletonMesh.GetUndeletedEdgesNumber();
	int edgeToDecreaseForEachLoop = (currentTotalEdgeNumber - lowestTotalEdgeNumber) / 100;

	for (int i = 0; i < 100; i++) {
		// MyTimer tTemp;
		// tTemp.Start("Model Simplification Rate " + std::to_string(i) + "% Start");
		std::cout << "Skeleton Mesh Simplification SSM Rate " + std::to_string(i) + "% Start\n";
		this->skeletonMesh.SimplifyMeshSSM(this->skeletonMesh.GetUndeletedEdgesNumber() - edgeToDecreaseForEachLoop);
		//tTemp.Flag("Model Simplification Rate " + std::to_string(i) + "% Done");
		std::cout << "Skeleton Mesh Simplification SSM Rate " + std::to_string(i) + "% Done\n\n";

		// save the final state
		SkeletonMesh tempSkeletonMesh = skeletonMesh;
		tempSkeletonMesh.LoadToShader();
		skeletonMeshs.push_back(tempSkeletonMesh);
	}
	
	// Set the current id for skeleton mesh to rendered
	this->currentSSMIDToRender = -1;
	this->SetSSMRate(0);
}

// simplify the mesh with SSM algorithm
void MeshObject::SimplifyMeshSSMOnce() {
	skeletonMesh.SimplifyMeshSSMOnce();
}
#pragma endregion