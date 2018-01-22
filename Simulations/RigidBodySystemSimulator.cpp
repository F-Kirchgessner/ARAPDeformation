#include "RigidBodySystemSimulator.h"


// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 1;
	m_elasticity = 0.5;
	m_timeFactor = 10;
}

// Functions
const char * RigidBodySystemSimulator::getTestCasesStr() {
	return "Single box, Two boxes, Multiple boxes";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Elasticity", TW_TYPE_FLOAT, &m_elasticity, "step=0.1 min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Timefactor", TW_TYPE_FLOAT, &m_timeFactor, "step=0.1 min=1.0");
}

void RigidBodySystemSimulator::initTestScene()
{
	switch (m_iTestCase)
	{
	case 0:
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(0, 0, 0.125f), Vec3(2, 1, 1));
		break;
	case 1:
		addRigidBody(Vec3(-0.6f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		addRigidBody(Vec3(0.3f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(-0.25f, 0.0f, 0), Vec3(-5, 0.5, 0.5));
		applyForceOnBody(getNumberOfRigidBodies() - 2, Vec3(-0.25f, 0.0f, 0), Vec3(5, 0, 0));
		break;
	case 2:
		addRigidBody(Vec3(-0.6f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		addRigidBody(Vec3(0.3f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		addRigidBody(Vec3(-0.3f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(0.25f, 0.0f, 0.0f), Vec3(10, 1, 1));
		addRigidBody(Vec3(-0.3f, 1.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
		applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(0.0f, 0.1f, 0), Vec3(+1, -5.6, 0));
		break;
	}
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_rigidbodysystems.clear();

	initTestScene();

}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

	// Draw mass points
	for (auto& rigidbodySystem : m_rigidbodysystems) {
		DUC->setUpLighting(Vec3(rigidbodySystem.red, rigidbodySystem.green, rigidbodySystem.blue), 0.4*Vec3(1, 1, 1), 2000.0, Vec3(rigidbodySystem.red, rigidbodySystem.green, rigidbodySystem.blue));
		rigidbodySystem.Obj2WorldMatrix = rigidbodySystem.scaleMat * rigidbodySystem.rotMat * rigidbodySystem.transMat;
		DUC->drawRigidBody(rigidbodySystem.Obj2WorldMatrix);
	}

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	default:
		reset();
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {

}
void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	timeStep *= m_timeFactor;
	// update current setup for each frame
	switch (m_iTestCase)
	{
	default:
		if (DXUTIsKeyDown(VK_LBUTTON))
			pullTogether();
		checkForCollisions();
		for (auto& rigidbodySystem : m_rigidbodysystems) {
			rigidbodySystem.updateStep(timeStep);
		}
		break;
	}
}

void RigidBodySystemSimulator::checkForCollisions() {
	for (int a = 0; a < m_rigidbodysystems.size(); a++) {
		for (int b = a+1; b < m_rigidbodysystems.size(); b++) {
			RigidbodySystem &bodyA = m_rigidbodysystems[a];
			RigidbodySystem &bodyB = m_rigidbodysystems[b];
			Mat4 worldA = bodyA.scaleMat * bodyA.rotMat * bodyA.transMat;
			Mat4 worldB = bodyB.scaleMat * bodyB.rotMat * bodyB.transMat;
			CollisionInfo simpletest = checkCollisionSAT(worldA, worldB);
			if (simpletest.isValid) {
				//std::printf("collision detected at normal: %f, %f, %f\n", simpletest.normalWorld.x, simpletest.normalWorld.y, simpletest.normalWorld.z);
				//std::printf("collision point : %f, %f, %f\n", (simpletest.collisionPointWorld).x, (simpletest.collisionPointWorld).y, (simpletest.collisionPointWorld).z);
				collisionDetected(bodyA, bodyB, simpletest.collisionPointWorld, simpletest.normalWorld);
			}
		}
	}
}

void RigidBodySystemSimulator::collisionDetected(RigidbodySystem &bodyA, RigidbodySystem &bodyB, Vec3 collisionPointWorld, Vec3 normalWorld) {
	Vec3 collisionPointA = collisionPointWorld - bodyA.m_position;
	Vec3 collisionPointB = collisionPointWorld - bodyB.m_position;
	//------------------------------------------------------------------------------------------------
	// NUMERATOR
	Vec3 velA = bodyA.velocity + cross(bodyA.angluarvelocity, collisionPointA);
	Vec3 velB = bodyB.velocity + cross(bodyB.angluarvelocity, collisionPointB);

	Vec3 vrel = velA - velB;
	float c = m_elasticity;
	float numerator = -(1 + c)*dot(vrel, normalWorld);
	float colCase = dot(velA - velB, normalWorld);
	if (colCase > 0.0f) return;
	//-----------------------------------------------------------------------------------------------

	float massInvA = 1.0 / bodyA.mass;
	float massInvB = 1.0 / bodyB.mass;
	Mat4 interiaTensorInvA = bodyA.interiatensor.inverse();
	Mat4 interiaTensorInvB = bodyB.interiatensor.inverse();

	//DENOMINATOR
	Vec3 xaCrossN = GamePhysics::cross(collisionPointA, normalWorld);
	Vec3 iaInverseTimesXaCrossN = interiaTensorInvA.transformVector(xaCrossN);
	Vec3 leftPlus = GamePhysics::cross(iaInverseTimesXaCrossN, collisionPointA);

	Vec3 xbCrossN = GamePhysics::cross(collisionPointB, normalWorld);
	Vec3 ibInverseTimesXbCrossN = interiaTensorInvB.transformVector(xbCrossN);
	Vec3 rightPlus = GamePhysics::cross(ibInverseTimesXbCrossN, collisionPointB);

	float denominator = massInvA+massInvB+ GamePhysics::dot((leftPlus + rightPlus),normalWorld);

	// Update velocity and angular momentum
	Vec3 result = (numerator / denominator) * normalWorld;
	bodyA.velocity += result * massInvA;
	bodyB.velocity -= result * massInvB;
	bodyA.angularMomentum += GamePhysics::cross(collisionPointA, result);
	bodyB.angularMomentum -= GamePhysics::cross(collisionPointB, result);

}

void RigidBodySystemSimulator::onClick(int x, int y) {

}
void RigidBodySystemSimulator::onMouse(int x, int y) {

}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_rigidbodysystems.size();
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return m_rigidbodysystems.at(i).m_position;
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return m_rigidbodysystems.at(i).velocity;
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return m_rigidbodysystems.at(i).angluarvelocity;
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	m_rigidbodysystems.at(i).applyForce(loc, force);
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, float mass) {
	RigidbodySystem rig(size,position,mass);
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(0.0, 1.0);
	rig.red = dis(gen);
	rig.green = dis(gen);
	rig.blue = dis(gen);
	//create; copy; delete; because of inner function, maybe emplace_back?
	m_rigidbodysystems.push_back(rig);
}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	// ToDo use Quaterion for orientation;
	
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	m_rigidbodysystems.at(i).velocity = velocity;
}

void RigidBodySystemSimulator::pullTogether() {
	for (int i = 0; i < m_rigidbodysystems.size() - 1; ++i)
	{
		Vec3 vel = m_rigidbodysystems[i + 1].m_position - m_rigidbodysystems[i].m_position;
		m_rigidbodysystems[i].velocity = vel * 0.1f;
		m_rigidbodysystems[i + 1].velocity = vel * -0.1f;
	}
}