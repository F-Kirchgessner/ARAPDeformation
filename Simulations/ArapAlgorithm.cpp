#include "ArapAlgorithm.h"



ArapAlgorithm::ArapAlgorithm(DirectX::GeometricPrimitive * _mesh)
{
	mesh = _mesh;
	VertexCollection points = _mesh->GetVertexList();
	p_amount = points.size();
	p.resize(3, p_amount);
	p_prime.resize(p_amount, 3);
	neighbours.resize(p_amount);
	for (size_t i = 0; i < p_amount; i++)
	{
		p.col(i) << points[i].position.x, points[i].position.y, points[i].position.z;
		p_prime.row(i) << points[i].position.x, points[i].position.y, points[i].position.z;
		neighbours[i] = new std::vector<uint16_t>();
		neighbours[i]->reserve(7);
	}
	auto faces = _mesh->GetIndexList();
	for (size_t idx = 0; idx < faces.size(); idx += 3) {
		auto a = faces[idx];
		auto b = faces[idx + 1];
		auto c = faces[idx + 2];
		neighbours[a]->push_back(b);
		neighbours[a]->push_back(c);
		neighbours[b]->push_back(a);
		neighbours[b]->push_back(c);
		neighbours[c]->push_back(a);
		neighbours[c]->push_back(b);
	}
	cout << "Neighbours done";
	for (auto adj_list : neighbours) {
		for (auto k : *adj_list) {
		}
		sort(adj_list->begin(), adj_list->end());
		adj_list->erase(unique(adj_list->begin(), adj_list->end()), adj_list->end());
	}
	R = vector<Eigen::Matrix3f>(p_amount, Eigen::Matrix3f::Identity());
	cout << "Object creation done!";
}

void ArapAlgorithm::init() {
	calculate_w();
	calculate_laplacian();
	calculate_b();
	calculate_Si_part();
}


void ArapAlgorithm::setHandle(uint16_t v, float x, float y, float z) {
	handles.insert_or_assign(v, Eigen::Vector3f(x, y, z));
}

void ArapAlgorithm::updateMesh() {
	for (size_t i = 0; i < p_amount; i++)
	{	
		XMFLOAT3 point(p_prime.row(i).x(), p_prime.row(i).y(), p_prime.row(i).z());
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
	Eigen::Vector3f vi = p.col(i);
	Eigen::Vector3f vj = p.col(j);
	Eigen::Vector3f va = p.col(shared[0]);
	


	float e1 = (vi - vj).norm();
	float e2 = (vi - va).norm();
	float e3 = (va - vj).norm();
	float alpha_cos = fabs((e3*e3 + e2*e2 - e1*e1) / (2 * e3*e2)); //Take abs to eliminate bad negative weighting

	float beta_cos = 0;
	if (shared.size() > 1) { //Some edges only have one shared vertex
		Eigen::Vector3f vb = p.col(shared[1]);
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
		if (handles.count(i) > 0) {
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
	p_prime = cholesky.solve(b);
	//return (p_prime - p).norm() / p.norm();
	return 0; // Performance :)
}



void ArapAlgorithm::calculate_R() {
	Eigen::MatrixXf Pi_prime;
	Eigen::Matrix<float, 3, 3> Si;
	for (size_t i = 0; i < p_amount; i++)
	{
		size_t n = neighbours[i]->size();
		Pi_prime.resize(n, 3);
		for (size_t index_j = 0; index_j < n; index_j++)
		{	
			uint16_t j = neighbours[i]->at(index_j);
			Pi_prime.row(index_j) = p_prime.row(i) - p_prime.row(j);
		}
		Si = Si_part[i]*Pi_prime;
		Eigen::JacobiSVD<Eigen::MatrixXf, Eigen::NoQRPreconditioner> svd(Si, Eigen::ComputeFullU | Eigen::ComputeFullV);
		R[i] = svd.matrixV()*svd.matrixU().transpose();
	}
}

void ArapAlgorithm::calculate_Si_part()
{
	Eigen::MatrixXf Di;
	Eigen::MatrixXf Pi;
	Si_part.resize(p_amount);
	for (size_t i = 0; i < p_amount; i++)
	{
		size_t n = neighbours[i]->size();
		Di = Eigen::MatrixXf::Zero(n, n);
		Pi.resize(3, n);
		for (size_t index_j = 0; index_j < n; index_j++)
		{
			uint16_t j = neighbours[i]->at(index_j);
			Di(index_j, index_j) = w.coeffRef(i, j);
			Pi.col(index_j) = p.col(i) - p.col(j);
		}
		Si_part[i] = Pi*Di;
	}
}
