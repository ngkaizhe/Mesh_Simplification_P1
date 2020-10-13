#include "MeshObject.h"


using namespace Eigen;

#define Quad
//#define Harmonic

struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

extern int faceSize;

#pragma region MyMesh

MyMesh::MyMesh()
{
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
}

MeshObject::~MeshObject()
{
}

bool MeshObject::Init(std::string fileName)
{
	bool retV = model.Init(fileName);
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

void MeshObject::Parameterization()
{
	std::cout << "Parameterization" << std::endl;

	OpenMesh::HPropHandleT<double> weight;
	OpenMesh::VPropHandleT<int> matrixIndex;

	MyMesh mesh = model.mesh;

	mesh.add_property(weight, "halfedgeWeight");
	mesh.add_property(matrixIndex, "row");
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
			//find select mesh boundary 
			if (!heh.is_valid())
			{
				heh = mesh.halfedge_handle(*e_it, 1);
			}
		}
	}
	std::cout << "Calculate weight finish!\n" << std::endl;

	std::cout << "Start calculate matrix size!" << std::endl;

	// calculate matrix size
	int count = 0;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		//mesh.property(matrixIndex, *v_it) = count++;
		if (mesh.is_boundary(*v_it))
		{
			mesh.property(matrixIndex, *v_it) = -1;
		}
		else
		{
			mesh.property(matrixIndex, *v_it) = count++;
		}
	}
	std::cout << "Matrix size is " << count << std::endl;
	std::cout << "Calculate matrix size finish!\n" << std::endl;

	std::cout << "Start fill matrix!" << std::endl;

	Eigen::SparseMatrix<double> A(count, count);
	Eigen::VectorXd BX(count);
	Eigen::VectorXd BY(count);
	Eigen::VectorXd BZ(count);
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > linearSolver;

	BX.setZero();
	BY.setZero();
	BZ.setZero();

	// fiil matrix
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		if (!mesh.is_boundary(*v_it))
		{
			int i = mesh.property(matrixIndex, *v_it);
			double totalWeight = 0;
			MyMesh::Point p = mesh.point(*v_it);
			BX[i] = p[0];
			BY[i] = p[1];
			BZ[i] = p[2];

			for (MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
			{
				MyMesh::HalfedgeHandle _heh = mesh.find_halfedge(*v_it, *vv_it);
				double w = mesh.property(weight, _heh);
				totalWeight += w;
			}
			A.insert(i, i) = 1;
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
					A.insert(i, j) = -w / totalWeight;
				}
			}
		}
	}

	A.makeCompressed();
	std::cout << "Fill matrix finish!\n" << std::endl;

	std::cout << "Start solve linear system!" << std::endl;

	// solve linear system
	linearSolver.compute(A);

	Eigen::VectorXd X = linearSolver.solve(BX);
	Eigen::VectorXd Y = linearSolver.solve(BY);
	Eigen::VectorXd Z = linearSolver.solve(BZ);
	std::cout << "Solve linear system finish!\n" << std::endl;

	// set texcoord
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		if (!mesh.is_boundary(*v_it))
		{
			int i = mesh.property(matrixIndex, *v_it);
			mesh.set_point(*v_it, MyMesh::Point(X[i], Y[i], Z[i]));
		}
	}

	//model.mesh.request_vertex_texcoords2D();
	for (MyMesh::VertexIter v_it = model.mesh.vertices_begin(); v_it != model.mesh.vertices_end(); ++v_it)
	{
		int i = mesh.property(matrixIndex, *v_it);
		if (i >= count || i < 0) {
			std::cout << "Index out of range!! " << i << std::endl;
			continue;
		}
			
		//model.mesh.set_texcoord2D(*v_it, MyMesh::TexCoord2D(-1, -1));
		//MyMesh::Point oriP = model.mesh.point(*v_it);
		//oriP = oriP + (MyMesh::Point(X[i], Y[i], Z[i]) - oriP) * 0.01f;
		//model.mesh.set_point(*v_it, oriP);
		model.mesh.set_point(*v_it, MyMesh::Point(X[i], Y[i], Z[i]));
	}//
	model.LoadToShader();

}