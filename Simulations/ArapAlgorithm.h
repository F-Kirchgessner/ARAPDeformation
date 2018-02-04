
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
	ArapAlgorithm(DirectX::GeometricPrimitive* _mesh);
	~ArapAlgorithm() {};
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
	void calculate_Si_part();


	uint16_t p_amount;
	Eigen::Matrix3Xf p;
	Eigen::MatrixX3f p_prime;
	Eigen::SparseMatrix<float> w;
	Eigen::SparseMatrix<float> L;
	Eigen::MatrixX3f b;
	std::vector<Eigen::Matrix3f> R;
	std::vector<Eigen::MatrixXf> Si_part;

	std::vector< std::vector<uint16_t>* > neighbours;
	std::map<uint16_t, Eigen::Vector3f> handles;

	Eigen::SparseLU<Eigen::SparseMatrix<float>> cholesky;

	GeometricPrimitive* mesh;
};