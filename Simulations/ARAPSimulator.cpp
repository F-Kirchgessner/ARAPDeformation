#include "ARAPSimulator.h"
#include "MassSpringSystemSimulator.h"

ARAPSimulator::ARAPSimulator()
{
	m_iTestCase = 1;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_iNumSpheres    = 100;
	m_fSphereSize    = 0.05f;

	m_RigidbodySystem = new RigidbodySystem();
}

const char * ARAPSimulator::getTestCasesStr(){
	return "Basic Mesh,Physics Objects";
}

void ARAPSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

		m_pMesh.reset();
		// TODO Release vertexNeighbours
}

void ARAPSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		//TwAddVarRW(DUC->g_pTweakBar, "Elasticity", TW_TYPE_FLOAT, &m_elasticity, "step=0.1 min=0.0");
		//TwAddVarRW(DUC->g_pTweakBar, "Timefactor", TW_TYPE_FLOAT, &m_timeFactor, "step=0.1 min=1.0");
		break;
	default:break;
	}
}

void ARAPSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: {
		cout << "Draw model!\n";
		
		m_pMesh = GeometricPrimitive::CreateMesh("../Butterfly.obj", DUC->g_pd3dImmediateContext, 0.05f, false);
		//findNeighbours(&vertexNeighbours);

		// Test move vertex
		alg = new ArapAlgorithm(m_pMesh.get());
		handleHelper(353, 0, 1, 0);
		handleHelper(65, 0, 0, 0);
		handleHelper(165, 0, 1, 0);
		handleHelper(258, 0, 0, 0);

		alg->init();
		cout << "Init done";
		// Update vertex buffer do display changes
		m_pMesh->UpdateBuffer(DUC->g_pd3dImmediateContext);

		cout << "Init done";
		break;
	}
	case 1:
		cout << "Physics Objects!\n";
		m_RigidbodySystem->reset();

		m_pMesh = GeometricPrimitive::CreateMesh("../Sign.obj", DUC->g_pd3dImmediateContext, 0.0045f, false);
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void ARAPSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void ARAPSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		alg->iteration_step();
		alg->updateMesh();
		m_pMesh->UpdateBuffer(DUC->g_pd3dImmediateContext);
		handleHelper(353, timeStep, -timeStep, timeStep);
		handleHelper(165,-timeStep, -timeStep, -timeStep);
		break;

	case 1:
		m_RigidbodySystem->simulateTimestep(timeStep);

	default:
		break;
	}
}

void ARAPSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch(m_iTestCase)
	{
	case 0: drawMesh();break;
	case 1: 
		m_RigidbodySystem->drawObjects(pd3dImmediateContext, DUC);
		drawMesh(m_RigidbodySystem->m_rigidbodies[3].m_position, m_RigidbodySystem->m_rigidbodies[3].orientation, Vec3(1, 1, 1));
		break;
	}
}

void ARAPSimulator::drawMesh()
{
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
	drawMesh(m_vfMovableObjectPos, m_vfRotate, Vec3(0.5, 0.5, 0.5));
}

void ARAPSimulator::drawMesh(Vec3 pos, Vec3 rot, Vec3 scale)
{
	XMVECTOR posXM = pos.toDirectXVector();
	XMVECTOR rotXM = rot.toDirectXVector();
	XMVECTOR scaleXM = scale.toDirectXVector();

	// Setup position/normal effect (per object variables)
	XMMATRIX s = XMMatrixScaling(XMVectorGetX(scaleXM), XMVectorGetY(scaleXM), XMVectorGetZ(scaleXM));
	XMMATRIX t = XMMatrixTranslation(XMVectorGetX(posXM), XMVectorGetY(posXM), XMVectorGetZ(posXM));
	XMMATRIX r = XMMatrixRotationRollPitchYaw(XMVectorGetX(rotXM), XMVectorGetX(rotXM), XMVectorGetX(rotXM));

	DUC->g_pEffectPositionNormal->SetWorld(r * s * t * DUC->g_camera.GetWorldMatrix());
	m_pMesh->Draw(DUC->g_pEffectPositionNormal, DUC->g_pInputLayoutPositionNormal);
}

void ARAPSimulator::drawMesh(Vec3 pos, Quat rot, Vec3 scale)
{
	XMVECTOR posXM = pos.toDirectXVector();
	XMVECTOR scaleXM = scale.toDirectXVector();

	// Setup position/normal effect (per object variables)
	XMMATRIX s = XMMatrixScaling(XMVectorGetX(scaleXM), XMVectorGetY(scaleXM), XMVectorGetZ(scaleXM));
	XMMATRIX t = XMMatrixTranslation(XMVectorGetX(posXM), XMVectorGetY(posXM), XMVectorGetZ(posXM));
	XMMATRIX r = XMMatrixRotationQuaternion(rot.toDirectXQuat());

	DUC->g_pEffectPositionNormal->SetWorld(r * s * t * DUC->g_camera.GetWorldMatrix());
	m_pMesh->Draw(DUC->g_pEffectPositionNormal, DUC->g_pInputLayoutPositionNormal);
}

void ARAPSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void ARAPSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

//This is just a helper function for Testing purposes!
void ARAPSimulator::handleHelper(int i, float x, float y, float z) {
	auto pos = m_pMesh->GetVertexList()[i].position;
	alg->setHandle(i, pos.x + x, pos.y + y, pos.z + z);
	m_pMesh->SetVertex(i, { pos.x + x, pos.y + y, pos.z + z });
};