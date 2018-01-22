#include "RigidbodySystem.h"


RigidbodySystem::RigidbodySystem()
{
	m_elasticity = 0.5;
	m_timeFactor = 10;
	m_fStiffness = 25.0f;
	m_fDamping = 0.01f;
	m_fGravityAccel = 9.81f;
	m_iIntegrator = EULER;
}


RigidbodySystem::~RigidbodySystem()
{
}


void RigidbodySystem::initTestScene()
{
	addRigidBody(Vec3(-0.6f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
	addRigidBody(Vec3(0.3f, 0.0f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 2.0f);
	applyForceOnBody(getNumberOfRigidBodies() - 1, Vec3(-0.25f, 0.0f, 0), Vec3(-5, 0.5, 0.5));
	applyForceOnBody(getNumberOfRigidBodies() - 2, Vec3(-0.25f, 0.0f, 0), Vec3(5, 0, 0));

	addSpring(0, 1, 0.25f);

	/*
	addMassPoint(Vec3(0.0f, 0.5f, 0), Vec3(0.0, 0.0, 0), true);
	addMassPoint(Vec3(0.2f, 0.3f, 0), Vec3(0.0, 0.0, 0), false);
	addMassPoint(Vec3(0.4f, 0.4f, 0), Vec3(0.0, 0.0, 0), false);
	addSpring(9, 10, 0.25f);
	*/
}


void RigidbodySystem::reset() {
	m_rigidbodies.clear();
	m_springList.clear();

	initTestScene();
}


void RigidBodySystemSimulator::drawObjects(ID3D11DeviceContext* pd3dImmediateContext) {

	// Draw rigid bodies
	for (auto& rigidbodySystem : m_rigidbodysystems) {
		DUC->setUpLighting(Vec3(rigidbodySystem.red, rigidbodySystem.green, rigidbodySystem.blue), 0.4*Vec3(1, 1, 1), 2000.0, Vec3(rigidbodySystem.red, rigidbodySystem.green, rigidbodySystem.blue));
		rigidbodySystem.Obj2WorldMatrix = rigidbodySystem.scaleMat * rigidbodySystem.rotMat * rigidbodySystem.transMat;
		DUC->drawRigidBody(rigidbodySystem.Obj2WorldMatrix);
	}

	// Draw springs
	DUC->beginLine();
	for (auto& spring : m_springList) {
		float springForce = spring.force.squaredDistanceTo(Vec3(0, 0, 0));
		DUC->drawLine(spring.mass_point1->position, Vec3(0, 1 - springForce, springForce), spring.mass_point2->position, Vec3(0, 1 - springForce, springForce));
	}
	DUC->endLine();
}


void RigidbodySystem::simulateTimestep(float timeStep) {
	timeStep *= m_timeFactor;

	if (DXUTIsKeyDown(VK_LBUTTON))
		pullTogether();
	checkForCollisions();
	for (auto& rigidbodySystem : m_rigidbodysystems) {
		rigidbodySystem.updateStep(timeStep);
	}
	break;
}


void RigidbodySystem::checkForCollisions() {
	for (int a = 0; a < m_rigidbodies.size(); a++) {
		for (int b = a + 1; b < m_rigidbodies.size(); b++) {
			Rigidbody &bodyA = m_rigidbodies[a];
			Rigidbody &bodyB = m_rigidbodies[b];
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

/*
void RigidbodySystem::integrate(float elapsedTime) {
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
*/

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


void RigidbodySystem::addRigidBody(Vec3 position, Vec3 size, float mass) {
	Rigidbody rig(size, position, mass);
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(0.0, 1.0);
	rig.red = dis(gen);
	rig.green = dis(gen);
	rig.blue = dis(gen);
	//create; copy; delete; because of inner function, maybe emplace_back?
	m_rigidbodies.push_back(rig);
}


void RigidbodySystem::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring s(m_rigidbodies.at(masspoint1), m_rigidbodies.at(masspoint2), m_fStiffness, initialLength);
	m_springList.push_back(s);
}


void RigidbodySystem::setOrientationOf(int i, Quat orientation) {
	// ToDo use Quaterion for orientation;

}


void RigidbodySystem::setVelocityOf(int i, Vec3 velocity) {
	m_rigidbodies.at(i).velocity = velocity;
}

/*
void RigidbodySystem::pullTogether() {
	for (int i = 0; i < m_rigidbodysystems.size() - 1; ++i)
	{
		Vec3 vel = m_rigidbodysystems[i + 1].m_position - m_rigidbodysystems[i].m_position;
		m_rigidbodysystems[i].velocity = vel * 0.1f;
		m_rigidbodysystems[i + 1].velocity = vel * -0.1f;
	}
}
*/