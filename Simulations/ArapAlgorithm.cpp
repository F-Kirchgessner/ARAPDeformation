#include "ArapAlgorithm.h"
#include <set>



void ArapAlgorithm::addMesh(
	DirectX::GeometricPrimitive* _mesh,
	std::map<uint16_t, std::vector<uint16_t>* >* _neighbours)
{	
	mesh = _mesh;
	VertexCollection points = _mesh->GetVertexList();
	p_amount = points.size();
	p_data.resize(p_amount);
	p.resize(3, p_amount);
	p_prime.resize(3, p_amount);
	neighbours.resize(p_amount);
	for (size_t i = 0; i < p_amount; i++)
	{
		p.col(i) << points[i].position.x, points[i].position.y, points[i].position.z;
		//cout << p.col(i) << endl;
		p_prime.col(i) << points[i].position.x, points[i].position.y, points[i].position.z;
		p_data[i] = Eigen::Vector3f(points[i].position.x, points[i].position.y, points[i].position.z);
		neighbours[i] = _neighbours->at(i);
	}

	R = vector<Eigen::Matrix3f>(p_amount, Eigen::Matrix3f::Identity());
}

void ArapAlgorithm::init() {
	calculate_w();
	calculate_laplacian();
	calculate_b();
}


void ArapAlgorithm::setHandle(uint16_t v, float x, float y, float z) {
	handles.insert_or_assign(v, Eigen::Vector3f(x, y, z));
}

void ArapAlgorithm::updateMesh() {
	for (size_t i = 0; i < p_amount; i++)
	{	
		XMFLOAT3 point(p_prime.col(i).x(), p_prime.col(i).y(), p_prime.col(i).z());
		mesh->SetVertex(i, point);
	}
}




void ArapAlgorithm::iteration_step() {
	calculate_p_prime();
	calculate_R();
	calculate_b();
}

float ArapAlgorithm::calculate_wij(uint16_t i, uint16_t j) {
	//return 1;
	//Find shared vertices
	std::vector<uint16_t>* v_i = neighbours[i];
	std::vector<uint16_t>* v_j = neighbours[j];
	std::vector<uint16_t> shared;
	set_intersection(v_i->begin(), v_i->end(), v_j->begin(), v_j->end(), std::back_inserter(shared));
	if (shared.size() > 2) { //Check if false neighbours are existing(common partner in different face)
		IndexCollection fs = mesh->GetIndexList();
		shared.clear();
		for (size_t k = 0; k < fs.size(); k += 3)
		{
			std::set<uint16_t> face = { fs[k], fs[k + 1], fs[k + 2] };
			face.erase(i);
			face.erase(j);
			if (face.size() == 1) {
				shared.push_back(*face.begin());
			}
		}
		assert(shared.size() < 3);
	}

	//Calculate the individual wij using the weight formula
	Eigen::Vector3f vi = p_data[i];
	Eigen::Vector3f vj = p_data[j];
	Eigen::Vector3f va = p_data[shared[0]];
	


	float e1 = (vi - vj).norm();
	float e2 = (vi - va).norm();
	float e3 = (va - vj).norm();
	float alpha_cos = fabs((e3*e3 + e2*e2 - e1*e1) / (2 * e3*e2)); //Take abs to eliminate bad negative weighting

	float beta_cos = 0;
	if (shared.size() > 1) { //Some edges only have one shared vertex
		Eigen::Vector3f vb = p_data[shared[1]];
		float e4 = (vi - vb).norm();
		float e5 = (vb - vj).norm();
		beta_cos = fabs((e4*e4 + e5*e5 - e1*e1) / (2 * e4*e5));
	}

	return ((alpha_cos / sqrt(1 - alpha_cos*alpha_cos)) + (beta_cos / sqrt(1 - beta_cos*beta_cos))) / 2;
}

void ArapAlgorithm::calculate_w() {
	std::vector<Eigen::Triplet<float>> wij;
	for (size_t i = 0; i < p_amount; i++)
	{
		std::vector<uint16_t>* v = neighbours[i];
		for (size_t index_j = 0; index_j < v->size(); index_j++)
		{
			uint16_t j = v->at(index_j);
			float value = calculate_wij(i,j);
			wij.push_back(Eigen::Triplet<float>(i, j, value));
			wij.push_back(Eigen::Triplet<float>(i + p_amount, j + p_amount, value));
			wij.push_back(Eigen::Triplet<float>(i + 2* p_amount, j +2* p_amount, value));
		}
	}
	w.resize(3 * p_amount, 3 * p_amount);
	w.setFromTriplets(wij.begin(), wij.end());
}

void ArapAlgorithm::calculate_laplacian() {
	//Should probably work now
	L.resize(p_amount, p_amount);
	for (size_t i = 0; i < p_amount; i++)
	{	
		if (handles.count(i) < 1) { //If not a Handle
			for (size_t index_j = 0; index_j < neighbours[i]->size(); index_j++)
			{
				uint16_t j = neighbours[i]->at(index_j);
				float wij = w.coeff(i, j);
				L.coeffRef(i, i) += wij;
				L.coeffRef(i, j) = -wij;
			}
		}
		else {
			L.coeffRef(i, i) = 1;
		}
	}


	L.makeCompressed();
	cholesky.analyzePattern(L);
	cholesky.factorize(L);
	assert(cholesky.info() == Eigen::Success);
}


void ArapAlgorithm::calculate_b() {
	b.resize(p_amount, 3);
	b.fill(0);
	for (size_t i = 0; i < p_amount; i++)
	{
		bool isHandle = handles.count(i) == 1;
		if (isHandle) {
			//Handles get set to their fixed position
			b.row(i) = handles[i];
		}
		else {
			//Right hand side of formula (8)
			for (size_t index_j = 0; index_j < neighbours[i]->size(); index_j++)
			{
				uint16_t j = neighbours[i]->at(index_j);
				b.row(i) += ((w.coeffRef(i, j)/2)*(R[i]+R[j])*(p.col(i)-p.col(j))).transpose();
			}
		}
	}
}

float ArapAlgorithm::calculate_p_prime() {
	Eigen::VectorXf b_vectorized(Eigen::VectorXf::Map(b.data(), b.cols()*b.rows()));
	p_prime = cholesky.solve(b).transpose();
	return (p_prime - p).norm() / p.norm();
}

void ArapAlgorithm::calculate_R() {
	Eigen::MatrixXf Di;
	Eigen::MatrixXf Pi;
	Eigen::MatrixXf Pi_prime;
	Eigen::MatrixXf Si;
	for (size_t i = 0; i < p_amount; i++)
	{
		size_t n = neighbours[i]->size();
		Di = Eigen::MatrixXf::Zero(n, n);
		Pi_prime.resize(3, n);
		Pi.resize(3, n);
		for (size_t index_j = 0; index_j < n; index_j++)
		{	
			uint16_t j = neighbours[i]->at(index_j);
			Di(index_j, index_j) = w.coeffRef(i, j);
			Pi.col(index_j) = p.col(i) - p.col(j);
			Pi_prime.col(index_j) = p_prime.col(i) - p_prime.col(j);
		}
		Si = Pi*Di*Pi_prime.transpose();
		//assert(Si.determinant() > 0);
		Eigen::JacobiSVD<Eigen::MatrixXf, Eigen::FullPivHouseholderQRPreconditioner> svd(Si, Eigen::ComputeFullU | Eigen::ComputeFullV);

		/*if (svd.matrixU().determinant() < 0)
			std::cout << "U determinant is negative!" << std::endl << svd.matrixU() << std::endl;
		else
			std::cout <<"U----------------------------------------------------------------------";

		if (svd.matrixV().determinant() < 0)
			std::cout << "V determinant is negative!" << std::endl << svd.matrixV() << std::endl;*/
		R[i] = svd.matrixV()*svd.matrixU().transpose();
		/*if (R[i].determinant() < 0)
			std::cout << "determinant is negative!" << std::endl;*/
	}

} 