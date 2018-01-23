#include "MassSpringSystemSimulator.h"

	MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_iIntegrator = 0;
	m_fInputScale = 0.001f;
	m_fGravityAccel = 9.81f;
}


const char * MassSpringSystemSimulator::getTestCasesStr() {
	return "Basic Test, Complex Test";
}


void MassSpringSystemSimulator::initTestScene() {
	switch (m_iTestCase)
	{
	case 0:
		addMassPoint(Vec3(0.0f, 0.0f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 0
		addMassPoint(Vec3(0.0f, 0.2f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 1
		addMassPoint(Vec3(0.2f, 0.2f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 2
		addMassPoint(Vec3(0.2f, 0.0f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 3
		addMassPoint(Vec3(0.0f, 0.0f, 0.3f), Vec3(0.0, 0.0, 0), false);	// 4
		addMassPoint(Vec3(0.0f, 0.2f, 0.3f), Vec3(0.0, 0.0, 0), false);	// 5
		addMassPoint(Vec3(0.2f, 0.2f, 0.3f), Vec3(0.0, 0.0, 0), false);	// 6
		addMassPoint(Vec3(0.2f, 0.0f, 0.3f), Vec3(0.0, 0.0, 0), false);	// 7
		addSpring(0, 1, 0.2f);
		addSpring(1, 2, 0.2f);
		addSpring(2, 3, 0.2f);
		addSpring(3, 0, 0.2f);
		addSpring(4, 5, 0.2f);
		addSpring(5, 6, 0.2f);
		addSpring(6, 7, 0.2f);
		addSpring(7, 4, 0.2f);
		addSpring(0, 4, 0.2f);
		addSpring(1, 5, 0.2f);
		addSpring(2, 6, 0.2f);
		addSpring(3, 7, 0.2f);

		addMassPoint(Vec3(0.0f, 0.5f, 0), Vec3(0.0, 0.0, 0), true);
		addMassPoint(Vec3(0.2f, 0.3f, 0), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(0.4f, 0.4f, 0), Vec3(0.0, 0.0, 0), false);
		addSpring(8, 9, 0.25f);
		addSpring(9, 10, 0.25f);

		addMassPoint(Vec3(-0.3f, 0.5f, -0.3f), Vec3(0.0, 0.0, 0), true);
		addMassPoint(Vec3(-0.3f, 0.5f, -0.5f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.5f, 0.5f, -0.5f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.5f, 0.5f, -0.3f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.3f, 0.5f, -0.3f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.3f, 0.5f, -0.5f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.5f, 0.5f, -0.5f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.5f, 0.5f, -0.3f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.3f, 0.5f, -0.3f), Vec3(0.0, 0.0, 0), false);
		addMassPoint(Vec3(-0.5f, 0.5f, -0.1f), Vec3(0.0, 0.0, 0), false);
		addSpring(11, 12, 0.05f);
		addSpring(12, 13, 0.05f);
		addSpring(13, 14, 0.05f);
		addSpring(14, 15, 0.05f);
		addSpring(15, 16, 0.05f);
		addSpring(16, 17, 0.05f);
		addSpring(17, 18, 0.05f);
		addSpring(18, 19, 0.05f);
		addSpring(19, 20, 0.05f);

		break;
	case 1:
		addMassPoint(Vec3(0.0f, 0.0f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 0
		addMassPoint(Vec3(0.2f, -0.15f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 1
		addMassPoint(Vec3(-0.2f, -0.15f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 2
		addMassPoint(Vec3(0.1f, -0.35f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 3
		addMassPoint(Vec3(-0.1f, -0.35f, 0.1f), Vec3(0.0, 0.0, 0), false);	// 4

		addMassPoint(Vec3(0.0f, -0.175f, 0.35f), Vec3(0.0, 0.0, 0), false);	// 5
		addMassPoint(Vec3(0.0f, -0.175f, 0.05f), Vec3(0.0, 0.0, 0), false);	// 6

		addMassPoint(Vec3(0.0f, -0.35f, 0.25f), Vec3(0.0, 0.0, 0), false);	// 7
		addMassPoint(Vec3(0.2f, -0.2f, 0.25f), Vec3(0.0, 0.0, 0), false);	// 8
		addMassPoint(Vec3(-0.2f, -0.2f, 0.25f), Vec3(0.0, 0.0, 0), false);	// 9
		addMassPoint(Vec3(0.1f, 0.0f, 0.25f), Vec3(0.0, 0.0, 0), false);	// 10
		addMassPoint(Vec3(-0.1f, 0.0f, 0.25f), Vec3(0.0, 0.0, 0), false);	// 11

		addMassPoint(Vec3(0.0f, -0.175f, 0.2f), Vec3(0.0, 0.0, 0), false);	// 12

		addSpring(0, 10, 0.2f);
		addSpring(0, 11, 0.2f);
		addSpring(1, 10, 0.2f);
		addSpring(1, 8, 0.2f);
		addSpring(3, 7, 0.2f);
		addSpring(3, 8, 0.2f);
		addSpring(4, 7, 0.2f);
		addSpring(4, 9, 0.2f);
		addSpring(2, 9, 0.2f);
		addSpring(2, 11, 0.2f);

		addSpring(0, 1, 0.2f);
		addSpring(1, 3, 0.2f);
		addSpring(3, 4, 0.2f);
		addSpring(4, 2, 0.2f);
		addSpring(2, 0, 0.2f);

		addSpring(7, 8, 0.2f);
		addSpring(7, 9, 0.2f);
		addSpring(9, 11, 0.2f);
		addSpring(10, 11, 0.2f);
		addSpring(10, 8, 0.2f);

		addSpring(6, 0, 0.2f);
		addSpring(6, 1, 0.2f);
		addSpring(6, 2, 0.2f);
		addSpring(6, 3, 0.2f);
		addSpring(6, 4, 0.2f);

		addSpring(5, 7, 0.2f);
		addSpring(5, 8, 0.2f);
		addSpring(5, 9, 0.2f);
		addSpring(5, 10, 0.2f);
		addSpring(5, 11, 0.2f);

		addSpring(12, 0, 0.2f);
		addSpring(12, 1, 0.2f);
		addSpring(12, 2, 0.2f);
		addSpring(12, 3, 0.2f);
		addSpring(12, 4, 0.2f);
		addSpring(12, 5, 0.2f);
		addSpring(12, 6, 0.2f);
		addSpring(12, 7, 0.2f);
		addSpring(12, 8, 0.2f);
		addSpring(12, 9, 0.2f);
		addSpring(12, 10, 0.2f);
		addSpring(12, 11, 0.2f);
		break;
	default:
		break;
	}
}


void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_fMass = m_fMassOld = 0.01f;
	m_fStiffness = m_fStiffnessOld = 25.0f;
	m_fDamping = m_fDampingOld = 0.01f;
	m_fGravityAccel = 9.81f;

	m_masspointList.clear();
	m_springList.clear();

	initTestScene();
}


void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	TwType TW_TYPE_INTEGRATORTYPE = TwDefineEnumFromString("IntegrationType", "Euler,Leapfrog,Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.01 min=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.1 min=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravityAccel, "step=0.01 min=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "InputForce", TW_TYPE_FLOAT, &m_fInputScale, "step=0.01 min=0.0001");

	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: TwAddVarRW(DUC->g_pTweakBar, "IntegrationType", TW_TYPE_INTEGRATORTYPE, &m_iIntegrator, ""); break;
	case 1: TwAddVarRW(DUC->g_pTweakBar, "IntegrationType", TW_TYPE_INTEGRATORTYPE, &m_iIntegrator, ""); break;
	default:break;
	}
}


void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		//ToDo: Clear point and spring list and add them again.
		cout << "Mass System!\n";
		reset();
		break;
	default:
		cout << "Complex Test!\n";
		reset();
		m_fStiffness = m_fStiffnessOld = 75.0f;
		break;
	}
}


void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
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
		inputWorld = inputWorld * m_fInputScale;

		// Apply to mass points
		for (auto& masspoint : m_masspointList) {
			masspoint.applyForce(inputWorld);
		}
	}
}


void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0: integrate(timeStep); break;
	case 1: integrate(timeStep); break;
	default: break;
	}
}


void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	// Draw mass points
	for (auto& masspoint : m_masspointList) {
		DUC->drawSphere(masspoint.position, Vec3(MASS_POINT_SIZE, MASS_POINT_SIZE, MASS_POINT_SIZE));
	}

	// Draw springs
	DUC->beginLine();
	for (auto& spring : m_springList) {
		float springForce = spring.force.squaredDistanceTo(Vec3(0, 0, 0));
		DUC->drawLine(spring.mass_point1->m_position, Vec3(0, 1 - springForce, springForce), spring.mass_point2->m_position, Vec3(0, 1 - springForce, springForce));
	}
	DUC->endLine();

	if (m_fMass != m_fMassOld) {
		for (auto& masspoint : m_masspointList) {
			masspoint.setMass(m_fMass);
		}
	}
	if (m_fDamping != m_fDampingOld) {
		for (auto& masspoint : m_masspointList) {
			masspoint.setDamping(m_fDamping);
		}
	}
	if (m_fStiffness != m_fStiffnessOld) {
		for (auto& spring : m_springList) {
			spring.setStiffness(m_fStiffness);
		}
	}

	m_fMassOld = m_fMass;
	m_fStiffnessOld = m_fStiffness;
	m_fDampingOld = m_fDamping;
}


void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}


void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}


void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}


void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}


void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}


int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	m_masspointList.push_back(Masspoint(position, velocity, isFixed, Vec3(0, 0, 0), m_fMass, m_fDamping));
	return m_masspointList.size() - 1;
}


void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring s(m_masspointList.at(masspoint1), m_masspointList.at(masspoint2), m_fStiffness, initialLength);
	m_springList.push_back(s);
}


int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_masspointList.size();
}


int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_springList.size();
}


Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return m_masspointList.at(index).getPosition();
}


Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return m_masspointList.at(index).getVelocity();
}


void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
}


void MassSpringSystemSimulator::integrate(float elapsedTime) {
	switch (m_iIntegrator) {
	//euler
	case 0:
		for (auto& spring : m_springList) {
			spring.computeElasticForces();
			spring.addToEndPoints();
		}

		for (auto &masspoint : m_masspointList) {
			masspoint.integrateVelocityEuler(elapsedTime);
			masspoint.integratePositionsEuler(elapsedTime);
		}

		for (auto& masspoint : m_masspointList) {
			masspoint.clearForce();
			masspoint.addGravity(m_fGravityAccel);
		}
		break;

	//leapfrog
	case 1:
		for (auto& spring : m_springList) {
			spring.computeElasticForces();
			spring.addToEndPoints();
		}

		if (init) {
			for (auto &masspoint : m_masspointList) {
				masspoint.initVelocity(elapsedTime / 2);
			}
		}
		init = false;

		for (auto &masspoint : m_masspointList) {
			masspoint.integrateVelocityLeapfrog(elapsedTime);
			masspoint.integratePositionsLeapfrog(elapsedTime);
		}

		for (auto& masspoint : m_masspointList) {
			masspoint.clearForce();
			masspoint.addGravity(m_fGravityAccel);
		}
		break;

	//midpoint
	case 2:
		std::vector<Vec3> PosTemp;
		std::vector<Vec3> VelTemp;
		std::vector<Vec3> oldPos;
		std::vector<Vec3> oldVel;
		Vec3 inputForce;
		if (m_masspointList.size() > 0)
			inputForce = m_masspointList[0].getForce();

		// Compute a(t)
		for (auto& spring : m_springList) {
			spring.computeElasticForces();
			spring.addToEndPoints();
		}

		// Compute xtmp at t+h/2 based on v(t)
		for (auto &massspoint : m_masspointList) {
			massspoint.integrateMidpointPosTemp(elapsedTime / 2, PosTemp);
		}

		// Compute vtmp at t+h/2 based on a(t)
		for (auto &massspoint : m_masspointList) {
			massspoint.integrateMidpointVelTemp(elapsedTime / 2, VelTemp);
		}

		for (unsigned int i = 0; i < m_masspointList.size(); i++) {
			m_masspointList[i].integrateSwitch(VelTemp, PosTemp, oldVel, oldPos, i);
		}

		// Compute a at t+h based on xtmp and vtmp
		for (auto& masspoint : m_masspointList) {
			masspoint.setForce(inputForce);
			masspoint.addGravity(m_fGravityAccel / 2);
		}

		for (auto& spring : m_springList) {
			spring.computeElasticForces();
			spring.addToEndPoints();
		}

		for (unsigned int i = 0; i < m_masspointList.size(); i++) {
			m_masspointList[i].integrateSwitchBack(oldVel, oldPos, i);
		}

		// Compute x at t+h
		// Compute v at t+h
		for (unsigned int i = 0; i < m_masspointList.size(); i++) {
			m_masspointList[i].computeX(elapsedTime, VelTemp, i);
			m_masspointList[i].computeY(elapsedTime, VelTemp, i);
		}

		for (auto& masspoint : m_masspointList) {
			masspoint.clearForce();
			masspoint.addGravity(m_fGravityAccel / 2);
		}
		break;
	}
}