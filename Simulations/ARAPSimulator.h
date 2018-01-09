#ifndef ARAPSIMULATOR_h
#define ARAPSIMULATOR_h

#include "Simulator.h"
#include "DeformableModel.h"

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
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawSomeRandomObjects();
	void drawMovableTeapot();
	void drawTriangle();

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
};

#endif