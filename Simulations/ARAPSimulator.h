#ifndef ARAPSIMULATOR_h
#define ARAPSIMULATOR_h

#include "Simulator.h"

#include "../DirectXTK/Src/Geometry.h"
#include <map>
#include <vector>

class ARAPSimulator:public Simulator{
public:
	// Construtors
	ARAPSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void findNeighbours(std::map<uint16_t, vector<uint16_t>* >* neighborList);
	void insertVertexNeighbors(std::map<uint16_t, vector<uint16_t>* >* neighborList, uint16_t vertex, uint16_t neighb1, uint16_t neighb2);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawSomeRandomObjects();
	void drawTriangle();
	void drawMesh();
	void drawMesh(Vec3 pos, Vec3 rot, Vec3 scale);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	int   m_iNumSpheres;
	float m_fSphereSize;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	std::unique_ptr<GeometricPrimitive> m_pMesh;
	std::map<uint16_t, vector<uint16_t>* > vertexNeighbours;
};

#endif