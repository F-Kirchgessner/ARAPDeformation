
#include "Simulator.h"
#include "RigidbodySystem.h"
#include "../DirectXTK/Src/Geometry.h"
#include <Eigen\Eigen>
#include <Eigen\Sparse>
#include <Eigen\SVD>
#include <vector>
#include <map>
#include <math.h>
#include <set>


class ArapAlgorithm {
public:
	ArapAlgorithm() {};
	~ArapAlgorithm() {};
	void addMesh(DirectX::GeometricPrimitive* mesh, std::map<uint16_t, std::vector<uint16_t>* >* neighbours);
	void setHandle(uint16_t v, float x, float y, float z);
	void iteration_step();
	void updateMesh();
	void init();

private:
	float calculate_wij(uint16_t i, uint16_t j);
	void calculate_w();
	void calculate_laplacian();
	void calculate_b();
	float calculate_p_prime();
	void calculate_R();


	uint16_t p_amount;
	vector<Eigen::Vector3f> p_data;
	Eigen::Matrix3Xf p;
	Eigen::Matrix3Xf p_prime;
	Eigen::SparseMatrix<float> w;
	Eigen::SparseMatrix<float> L;
	Eigen::MatrixX3f b;
	std::vector<Eigen::Matrix3f> R;

	std::vector< std::vector<uint16_t>* > neighbours;
	std::map<uint16_t, Eigen::Vector3f> handles;
	//std::set<std::set<uint16_t, Eigen::Vector3f>> SetHandles;
	//std::set<Eigen::Vector3f> HandleVertices;
	//std::set<Eigen::Vector3f> HandlePosition;

	Eigen::SparseLU<Eigen::SparseMatrix<float>> cholesky;

	GeometricPrimitive* mesh;
};