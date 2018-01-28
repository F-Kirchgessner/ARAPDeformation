#include "RigidbodySystem.h"


RigidbodySystem::RigidbodySystem()
{
	m_elasticity = 1.0f;
	m_timeFactor = 1.0f;
	m_fStiffness = 6.0f;
	m_fDampingVel = 0.1f;
	m_fDampingRot = 1.0f;
	m_fGravity = 9.81f;
	m_iIntegrator = EULER;
}


RigidbodySystem::~RigidbodySystem()
{
}


void RigidbodySystem::initTestScene()
{
	addRigidBody(Vec3(-0.6f, 1.5f, 0.0f), Vec3(0.1f, 0.1f, 0.1f), 0.0f, true, true);
	addRigidBody(Vec3(0.6f, 1.5f, 0.0f), Vec3(0.1f, 0.1f, 0.1f), 0.0f, true, true);

	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.7f, 0.5f, 0.1f), 1.0f, false, false);
	signIndex = getNumberOfRigidBodies() - 1;

	addRigidBody(Vec3(0.0f, 0.0f, -0.5f), Vec3(0.2f, 0.2f, 0.2f), 0.1f, false, true);
	applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(0, 0, -0.1f), Vec3(0, 0, 500.0f));

	addSpring(0, signIndex, Vec3(0, 0, 0), Vec3(-0.3f, 0.25f, 0), 0.4f);
	addSpring(1, signIndex, Vec3(0, 0, 0), Vec3(0.3f, 0.25f, 0), 0.4f);
}


void RigidbodySystem::reset() {
	m_rigidbodies.clear();
	m_springList.clear();

	initTestScene();
}


void RigidbodySystem::drawObjects(ID3D11DeviceContext* pd3dImmediateContext, DrawingUtilitiesClass* DUC) {

	// Draw rigid bodies
	for (auto& rigidbodySystem : m_rigidbodies) {
		if (rigidbodySystem.visible) {
			DUC->setUpLighting(Vec3(rigidbodySystem.red, rigidbodySystem.green, rigidbodySystem.blue), 0.4*Vec3(1, 1, 1), 2000.0, Vec3(rigidbodySystem.red, rigidbodySystem.green, rigidbodySystem.blue));
			rigidbodySystem.Obj2WorldMatrix = rigidbodySystem.scaleMat * rigidbodySystem.rotMat * rigidbodySystem.transMat;
			DUC->drawRigidBody(rigidbodySystem.Obj2WorldMatrix);
		}
	}

	// Draw springs
	DUC->beginLine();
	for (auto& spring : m_springList) {
		float springForce = spring.force.squaredDistanceTo(Vec3(0, 0, 0));
		Vec3 color = Vec3(0.4, 0.7, 0.2);
		DUC->drawLine(spring.mass_point1->m_position + spring.pos1, color, spring.mass_point2->m_position + spring.pos2, color);
	}
	DUC->endLine();
}


void RigidbodySystem::simulateTimestep(float timeStep) {
	timeStep *= m_timeFactor;

	integrate(timeStep);
	checkForCollisions();
	removeFallenBlocks();
	for (auto& rigidbodySystem : m_rigidbodies) {
		rigidbodySystem.updateStep(timeStep);
	}
}


void RigidbodySystem::checkForCollisions() {
	for (int a = 0; a < m_rigidbodies.size(); a++) {
		for (int b = a + 1; b < m_rigidbodies.size(); b++) {
			Rigidbody &bodyA = m_rigidbodies[a];
			Rigidbody &bodyB = m_rigidbodies[b];
			if (!bodyA.isFixed && !bodyB.isFixed) {
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
}


void RigidbodySystem::collisionDetected(Rigidbody &bodyA, Rigidbody &bodyB, Vec3 collisionPointWorld, Vec3 normalWorld) {
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

	float denominator = massInvA + massInvB + GamePhysics::dot((leftPlus + rightPlus), normalWorld);

	// Update velocity and angular momentum
	Vec3 result = (numerator / denominator) * normalWorld;
	bodyA.velocity += result * massInvA;
	bodyB.velocity -= result * massInvB;
	bodyA.angularMomentum += GamePhysics::cross(collisionPointA, result);
	bodyB.angularMomentum -= GamePhysics::cross(collisionPointB, result);

}


void RigidbodySystem::integrate(float elapsedTime) {
	switch (m_iIntegrator) {
		//euler
	case 0:
		for (auto& spring : m_springList) {
			spring.computeElasticForces();
			spring.addToEndPoints();
		}

		for (auto& masspoint : m_rigidbodies) {
			masspoint.addGravity(m_fGravity);
		}
	}
}


// ExtraFunctions
int RigidbodySystem::getNumberOfRigidBodies() {
	return m_rigidbodies.size();
}


Vec3 RigidbodySystem::getPositionOfRigidBody(int i) {
	return m_rigidbodies.at(i).m_position;
}


Vec3 RigidbodySystem::getLinearVelocityOfRigidBody(int i) {
	return m_rigidbodies.at(i).velocity;
}


Vec3 RigidbodySystem::getAngularVelocityOfRigidBody(int i) {
	return m_rigidbodies.at(i).angluarvelocity;
}


void RigidbodySystem::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	m_rigidbodies.at(i).applyForce(loc, force);
}


void RigidbodySystem::addRigidBody(Vec3 position, Vec3 size, float mass, bool isFixed, bool visible) {
	Rigidbody rig(size, position, mass, m_fDampingVel, m_fDampingRot, isFixed, visible);
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(0.0, 1.0);
	rig.red = dis(gen);
	rig.green = dis(gen);
	rig.blue = dis(gen);
	//create; copy; delete; because of inner function, maybe emplace_back?
	m_rigidbodies.push_back(rig);
}


void RigidbodySystem::addSpring(int masspoint1, int masspoint2, Vec3 pos1, Vec3 pos2, float initialLength) {
	Spring s(m_rigidbodies.at(masspoint1), m_rigidbodies.at(masspoint2), pos1, pos2, m_fStiffness, initialLength);
	m_springList.push_back(s);
}


void RigidbodySystem::setOrientationOf(int i, Quat orientation) {
	// ToDo use Quaterion for orientation;

}


void RigidbodySystem::setVelocityOf(int i, Vec3 velocity) {
	m_rigidbodies.at(i).velocity = velocity;
}

void RigidbodySystem::throwBlock() {
	if (!m_rigidbodies.empty()) {
		Vec3 newBody;
		newBody.x = m_rigidbodies[signIndex].m_position.x;
		newBody.y = m_rigidbodies[signIndex].m_position.y;
		newBody.Z = m_rigidbodies[signIndex].m_position.z - 0.5f;

		addRigidBody(newBody, Vec3(0.2f, 0.2f, 0.2f), 0.1f, false, true);
		applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(0, 0, -0.1f), Vec3(0, 0, 500.0f));
	}
}

void RigidbodySystem::removeFallenBlocks() {

	std::vector<int> indexlist;
	for (int i = 0; i < m_rigidbodies.size(); i++)
	{
		if (m_rigidbodies[i].m_position.y <= -2.0f && i != signIndex) {
			indexlist.push_back(i);
		}
	}
	for (int i = 0; i < indexlist.size(); i++)
	{
		m_rigidbodies.erase(m_rigidbodies.begin() + indexlist[i]);		
	}
}

/*
void RigidbodySystem::pullTogether() {
	for (int i = 0; i < m_rigidbodies.size() - 1; ++i)
	{
		Vec3 vel = m_rigidbodies[i + 1].m_position - m_rigidbodies[i].m_position;
		m_rigidbodies[i].velocity = vel * 0.1f;
		m_rigidbodies[i + 1].velocity = vel * -0.1f;
	}
}
*/