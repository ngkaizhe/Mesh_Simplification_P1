#include "SkeletonMesh.h"

SkeletonMesh::SkeletonMesh() {
	// do nothing
}

SkeletonMesh::SkeletonMesh(GLMesh model) {
	// reserve the size first so our vector wont change its container size
	this->vertices.reserve(model.mesh.n_vertices());
	this->edges.reserve(model.mesh.n_edges());

	isSamplingCostInit = true;
	// init ssm value
	this->InitSSM(model);
	isSamplingCostInit = false;

	// reupdate the lowest edge to collapse
	this->CalculateEdgeIDToBeCollapsed();
	// reload the shader
	this->LoadToShader();
}

// helper function to reload the shader
void SkeletonMesh::LoadToShader() {
	std::vector<glm::vec3> verticesToRender;
	verticesToRender.reserve(this->GetUndeletedEdgesNumber() * 2);

	for (int k = 0; k < this->edges.size(); k++) {
		if (!this->edges[k]._isDeleted) {
			verticesToRender.push_back(this->edges[k]._i->_point);
			verticesToRender.push_back(this->edges[k]._j->_point);
		}
	}

	glGenVertexArrays(1, &lineVAO);
	glBindVertexArray(lineVAO);

	GLuint lineVBO;
	glGenBuffers(1, &lineVBO);
	glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * verticesToRender.size(), &verticesToRender[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

// render the line (currently)
void SkeletonMesh::Render(Shader shader) {
	// line render
	shader.use();

	glBindVertexArray(lineVAO);
	glDrawArrays(GL_LINES, 0, this->edges.size() * 2);
	glBindVertexArray(0);

	bool debugPointRendered = false;

	if (debugPointRendered) {
		// debug point render
		shader.setUniform3fv("color", glm::vec3(0.0f, 0.0f, 1.0f));

		if (this->edgeIDToBeCollapsed != -1) {
			std::vector<glm::vec3> verticesToRender;
			verticesToRender.reserve(2);
			verticesToRender.push_back(this->edges[this->edgeIDToBeCollapsed]._i->_point);
			verticesToRender.push_back(this->edges[this->edgeIDToBeCollapsed]._j->_point);

			if (verticesToRender.size() != 0) {
				GLuint tempVbo;
				glGenBuffers(1, &tempVbo);
				glBindBuffer(GL_ARRAY_BUFFER, tempVbo);
				glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * verticesToRender.size(), &verticesToRender[0], GL_STATIC_DRAW);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
				glEnableVertexAttribArray(0);

				glDrawArrays(GL_POINTS, 0, verticesToRender.size());
				glBindBuffer(GL_ARRAY_BUFFER, 0);
			}
		}
	}
}

// init the SSM methd
void SkeletonMesh::InitSSM(GLMesh model) {
	// init the vertices
	// we init the vertices from the starting idx
	for (int k = 0; k < model.mesh.n_vertices(); k++) {
		MyMesh::VertexHandle vh = model.mesh.vertex_handle(k);
		Vertex vertex;

		// init _point
		// init _idx
		// init _isDeleted
		vertex = Vertex(model.mesh.point(vh), vh.idx());

		// for this our vertices index will be exactly same as the original mesh vertices idx
		this->vertices.push_back(vertex);
	}

	// init the edges
	// we init the edges from the starting idx
	for (int k = 0; k < model.mesh.n_edges(); k++) {
		MyMesh::EdgeHandle eh = model.mesh.edge_handle(k);
		MyMesh::HalfedgeHandle heh = model.mesh.halfedge_handle(eh, 0);
		MyMesh::VertexHandle vhi = model.mesh.from_vertex_handle(heh);
		MyMesh::VertexHandle vhj = model.mesh.to_vertex_handle(heh);

		Edge edge;

		// init _i
		// init _j
		// init _idx
		// init _isDeleted
		edge = Edge(&this->vertices[vhi.idx()], &this->vertices[vhj.idx()], eh.idx());

		// pushback first
		this->edges.push_back(edge);
		Edge* edgePtr = &this->edges[this->edges.size() - 1];

		// set the oneRingVertices of i and j
		edgePtr->_i->oneRingVertices.push_back(edgePtr->_j);
		edgePtr->_j->oneRingVertices.push_back(edgePtr->_i);

		// set the oneRingEdges of i and j
		edgePtr->_i->oneRingEdges.push_back(edgePtr);
		edgePtr->_j->oneRingEdges.push_back(edgePtr);
	}

	// loop through all edges
	// vertices part
	// init the quadratic matrix
	// init the totalDistanceOfOneRing
	for (int k = 0; k < this->edges.size(); k++) {
		glm::vec3 i = this->edges[k]._i->_point;
		glm::vec3 j = this->edges[k]._j->_point;

		// a is the normalized edge vector of edge(i, j)
		glm::vec3 a = glm::normalize(j - i);
		glm::vec3 b = glm::cross(a, i);

		// glm is column major
		glm::mat4x3 K = glm::mat4x3(0.0f);
		K[0] = glm::vec3(0, a.z, -a.y);
		K[1] = glm::vec3(-a.z, 0, a.x);
		K[2] = glm::vec3(a.y, -a.x, 0);
		K[3] = glm::vec3(-b.x, -b.y, -b.z);

		glm::mat4 Q = glm::transpose(K) * K;

		this->edges[k]._i->quadraticMat += Q;
		this->edges[k]._j->quadraticMat += Q;

		// optimization purpose, we add the total distance of the vertex
		this->edges[k]._i->totalDistanceOfOneRing += glm::distance(i, j);
		this->edges[k]._j->totalDistanceOfOneRing += glm::distance(i, j);

	}

	// edges part
	// init the costIToJ
	// init the costJToI
	for (int k = 0; k < this->edges.size(); k++) {
		this->edges[k].costIToJ = this->F(this->edges[k]._i, this->edges[k]._j);
		this->edges[k].costJToI = this->F(this->edges[k]._j, this->edges[k]._i);
	}
}

// simplify our mesh with ssm algorithm
void SkeletonMesh::SimplifyMeshSSMOnce() {
	Edge* edgeToCollapse = &this->edges[this->edgeIDToBeCollapsed];
	Vertex* i;
	Vertex* j;

	// determine either collapse i->j
	// or j->i
	if(edgeToCollapse->costIToJ < edgeToCollapse->costJToI){
		i = edgeToCollapse->_i;
		j = edgeToCollapse->_j;
	}
	else {
		i = edgeToCollapse->_j;
		j = edgeToCollapse->_i;
	}

	// update the quadratic matrix
	j->quadraticMat += i->quadraticMat;
	// collapse edge
	this->CollapseEdge(edgeToCollapse, i, j);

	// reupdate all cost of the halfedge around the collapsed vertex
	for (int k = 0; k < j->oneRingEdges.size(); k++) {
		j->oneRingEdges[k]->costIToJ = this->F(j->oneRingEdges[k]->_i, j->oneRingEdges[k]->_j);
		j->oneRingEdges[k]->costJToI = this->F(j->oneRingEdges[k]->_j, j->oneRingEdges[k]->_i);
	}
	
	// reupdate the lowest edge to collapse
	this->CalculateEdgeIDToBeCollapsed();
	// reload the shader
	this->LoadToShader();
}

void SkeletonMesh::SimplifyMeshSSM(int edgesToLeft) {
	while (this->GetUndeletedEdgesNumber() > edgesToLeft) {
		Edge* edgeToCollapse = &this->edges[this->edgeIDToBeCollapsed];
		Vertex* i;
		Vertex* j;

		// determine either collapse i->j
		// or j->i
		if (edgeToCollapse->costIToJ < edgeToCollapse->costJToI) {
			i = edgeToCollapse->_i;
			j = edgeToCollapse->_j;
		}
		else {
			i = edgeToCollapse->_j;
			j = edgeToCollapse->_i;
		}

		// update the quadratic matrix
		j->quadraticMat += i->quadraticMat;
		// collapse edge
		this->CollapseEdge(edgeToCollapse, i, j);

		// reupdate all cost of the halfedge around the collapsed vertex
		for (int k = 0; k < j->oneRingEdges.size(); k++) {
			j->oneRingEdges[k]->costIToJ = this->F(j->oneRingEdges[k]->_i, j->oneRingEdges[k]->_j);
			j->oneRingEdges[k]->costJToI = this->F(j->oneRingEdges[k]->_j, j->oneRingEdges[k]->_i);
		}

		// reupdate the lowest edge to collapse
		this->CalculateEdgeIDToBeCollapsed();
	}
}

// helper function for finding the lowest cost to collapse
void SkeletonMesh::CalculateEdgeIDToBeCollapsed() {
	double currentLowestCost;
	bool isFirst = true;

	for (int i = 0; i < edges.size(); i++) {
		// we dont consider the edge that was deleted
		if (edges[i]._isDeleted) continue;

		double minCost = std::min(edges[i].costIToJ, edges[i].costJToI);
		if (isFirst) {
			this->edgeIDToBeCollapsed = i;
			isFirst = false;
			currentLowestCost = minCost;
		}
		else if (minCost < currentLowestCost) {
			this->edgeIDToBeCollapsed = i;
			currentLowestCost = minCost;
		}
	}
}

//helper function for calculating shape cost and sampling cost
// the cost are the halfedge(i->j)
float SkeletonMesh::F(Vertex* i, Vertex* j) {
	// init shape cost
	// shapeCost = Fi(vj) + Fj(vj)
	glm::vec4 vi = glm::vec4(i->_point, 1.0f);
	glm::mat4 Qi = i->quadraticMat;

	glm::vec4 vj = glm::vec4(j->_point, 1.0f);
	glm::mat4 Qj = j->quadraticMat;

	// Fivj = (vj(T) * Qi * vj)
	// as glm doesn't provide vector * matrix from the left
	// so v(T) * Qi = (Qi * vj(T))(T) --> 1X4 vector
	// 1X4 vector * 4X1 vector = dot product
	float Fivj = glm::dot(glm::transpose(Qi) * vj, vj);
	float Fjvj = glm::dot(glm::transpose(Qj) * vj, vj);
	float shapeCost = Fivj + Fjvj;

	// init sampling cost
	float samplingCost = 0;
	if (isSamplingCostInit) {
		samplingCost = glm::distance(vi, vj) * i->totalDistanceOfOneRing;
	}

	else {
		// we recalculate the sampling cost from the one ring of vertices
		for (int index = 0; index < i->oneRingVertices.size(); index++) {
			glm::vec4 vTemp = glm::vec4(i->oneRingVertices[index]->_point, 1.0f);
			samplingCost += glm::distance(vi, vTemp);
		}
		samplingCost *= glm::distance(vi, vj);
	}

	// return the result
	return shapeCost + 0.1 * samplingCost;
}

// helper function that is similiar for open mesh
// get the edge from the given 2 vertices
Edge* SkeletonMesh::GetEdge(Vertex* i, Vertex* j) {
	const std::vector<Edge*> iOneRingEdges = i->oneRingEdges;
	const std::vector<Edge*> jOneRingEdges = j->oneRingEdges;

	// find the same edges in the one ring of vertices
	// which means the edges have both i and j as its end point
	for (int i = 0; i < iOneRingEdges.size(); i++) {
		for (int j = 0; j < jOneRingEdges.size(); j++) {
			// exception consideration
			if (iOneRingEdges[i]->_isDeleted || jOneRingEdges[j]->_isDeleted) throw "In void SkeletonMesh::GetEdge()\nEdge that was already deleted shouldn't be found!\n";

			if (*jOneRingEdges[j] == *iOneRingEdges[i]) {
				return jOneRingEdges[j];
			}
		}
	}
}

// collapse the given halfEdge
void SkeletonMesh::CollapseEdgeIToJ(Edge* edge){
	this->CollapseEdge(edge, edge->_i, edge->_j);
}

void SkeletonMesh::CollapseEdgeJToI(Edge* edge) {
	this->CollapseEdge(edge, edge->_j, edge->_i);
}

void SkeletonMesh::CollapseEdge(Edge* edge, Vertex* i, Vertex* j) {
	// find the common adjadent vertices ij
	// we will save the common adjadent vertices idxs
	std::vector<int> commonAdjadentVerticesIJ_Idxs;
	for (int k = 0; k < i->oneRingVertices.size(); k++) {
		Vertex* v1 = i->oneRingVertices[k];
		for (int l = 0; l < j->oneRingVertices.size(); l++) {
			Vertex* v2 = j->oneRingVertices[l];
			
			// exception consideration
			// this shouldn't get ran
			if (v1->_isDeleted || v2->_isDeleted) {
				throw "In void SkeletonMesh::CollapseEdge()\nVertex that was already deleted shouldn't be found!\n";
			}

			// common adjadent vertices found
			if (*v1 == *v2) {
				commonAdjadentVerticesIJ_Idxs.push_back(v1->_idx);
			}
		}
	}

	// as we are collapsing the halfedge of i->j
	// update the information between j and the i's one ring edges

	// i's one ring edges should be update
	// so the edge informations are all from i(that should be updated)
	for (int k = 0; k < i->oneRingEdges.size(); k++) {
		Edge* edge = i->oneRingEdges[k];
		// edge exception consideration
		if (edge->_isDeleted) throw "In void SkeletonMesh::CollapseEdge()\nEdge that was already deleted shouldn't be found!\n";

		// we first defined the from_vertex and to_vertex
		// the from_vertex should be i
		Vertex* from_vertex;
		Vertex* to_vertex;
		if (*(edge->_i) == *i) {
			from_vertex = edge->_i;
			to_vertex = edge->_j;
		}
		else {
			from_vertex = edge->_j;
			to_vertex = edge->_i;
		}


		// there are 2 type of edges 
		// case 1: edge of (i, j) // this should only be 1
		// case 2: edges of (common adjadent vertices ij, i) // this should only be 2
		if (*to_vertex == *j || std::find(commonAdjadentVerticesIJ_Idxs.begin(), commonAdjadentVerticesIJ_Idxs.end(), to_vertex->_idx) != commonAdjadentVerticesIJ_Idxs.end()) {
			// delete the edge
			edge->_isDeleted = true;
		}

		// case 3: other edges to be update
		else {
			// update the edge side that is i
			if (*(edge->_i) == *i) {
				edge->_i = j;
			}
			else {
				edge->_j = j;
			}

			// update the edge to j's one ring edges
			j->oneRingEdges.push_back(edge);
		}
	}

	// j's one ring edges should be update
	// the case 1 from the previous loop
	// there should have 1 edge from j's one ring edges that have been deleted
	for (int k = 0; k < j->oneRingEdges.size(); ) {
		if (j->oneRingEdges[k]->_isDeleted) {
			j->oneRingEdges.erase(j->oneRingEdges.begin() + k);
			break;
		} 
		else k++;
	}

	// i's one ring vertices should be update
	for (int k = 0; k < i->oneRingVertices.size(); k++) {
		Vertex* v = i->oneRingVertices[k];

		// find whether the v is the adjadent of j
		// if it is save the index
		bool isJAdjadent = false;
		int adjIndex = -1;
		for (int l = 0; l < j->oneRingVertices.size(); l++) {
			if (*(j->oneRingVertices[l]) == *v) {
				isJAdjadent = true;
				adjIndex = l;
				break;
			}
		}


		// there are also 3 types of vertices
		// j
		if (*v == *j) {
			// do nothing
		}

		// common adjadent vertices of i and j
		else if (isJAdjadent) {
			// delete i from the v's one ring vertices
			for (int l = 0; l < v->oneRingVertices.size(); l++) {
				if (*v->oneRingVertices[l] == *i) {
					v->oneRingVertices.erase(v->oneRingVertices.begin() + l);
					break;
				}
			}
			// delete the edge(i, v) from the v's one ring edges
			for (int l = 0; l < v->oneRingEdges.size(); l++) {
				Edge* e = v->oneRingEdges[l];
				if ((*e->_i == *i) && (*e->_j == *v) ||
					(*e->_j == *i) && (*e->_i == *v)) {
					v->oneRingEdges.erase(v->oneRingEdges.begin() + l);
					break;
				}
			}
		}

		// the other vertices
		// update the vertices one ring info
		else {
			// update the v to have j as one ring (i becomes j)
			for (int l = 0; l < v->oneRingVertices.size(); l++) {
				if (*v->oneRingVertices[l] == *i) {
					v->oneRingVertices[l] = j;
					break;
				}
			}

			// update the j's one ring vertices to have v
			j->oneRingVertices.push_back(v);
		}
	}

	// delete i from the j's one ring vertices
	for (int l = 0; l < j->oneRingVertices.size(); l++) {
		if (*(j->oneRingVertices[l]) == *i) {
			j->oneRingVertices.erase(j->oneRingVertices.begin() + l);
		}
	}
	
	// set delete for the i vertex
	i->_isDeleted = true;

	// check for whether there have unconsidered conditions
	for (int k = 0; k < i->oneRingVertices.size(); k++) {
		Vertex* v = i->oneRingVertices[k];

		// the one ring of v shouldn't have i inside
		for (int l = 0; l < v->oneRingVertices.size(); l++) {
			if (*(v->oneRingVertices[l]) == *i) {
				throw "V shouldn't have i inside!\n";
			}
		}
	}

	for (int k = 0; k < i->oneRingEdges.size(); k++) {
		Edge* e = i->oneRingEdges[k];

		// the undeleted edge shouldn't have i as its vertices
		// the e shouldn't have i as its vertices
		if (!e->_isDeleted) {
			if (*(e->_i) == *i || *(e->_j) == *i) {
				throw "E shouldn't have i as its vertices!\n";
			}
		}
	}
}

void SkeletonMesh::PrintSkeletonMeshInfo() {
	std::cout << "\n";
	std::cout << "Total Vertices => " << GetUndeletedVerticesNumber() << '\n';
	std::cout << "Total Edges => " << GetUndeletedEdgesNumber() << '\n';
	std::cout << "\n";
}

int SkeletonMesh::GetUndeletedVerticesNumber() {
	int totalVerticesNumber = 0;
	for (int k = 0; k < this->vertices.size(); k++) {
		if (!this->vertices[k]._isDeleted) totalVerticesNumber++;
	}

	return totalVerticesNumber;
}

int SkeletonMesh::GetUndeletedEdgesNumber() {
	int totalEdgesNumber = 0;
	for (int k = 0; k < this->edges.size(); k++) {
		if (!this->edges[k]._isDeleted) totalEdgesNumber++;
	}

	return totalEdgesNumber;
}