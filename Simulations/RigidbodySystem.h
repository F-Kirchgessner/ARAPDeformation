#pragma once
#include "Simulator.h"
#include "global.h"
#include "Rigidbody.h"
#include "Spring.h"
#include "util\matrixbase.h"

#include <DirectXMath.h>
#include "collisionDetect.h"

#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2

class RigidbodySystem
{
public:
	RigidbodySystem();
	~RigidbodySystem();

	// Functions
	void reset();
	void simulateTimestep(float timeStep);
	void checkForCollisions();
	void collisionDetected(Rigidbody &bodyA, Rigidbody &bodyB, Vec3 collisionPointWorld, Vec3 normalWorld);
	void drawObjects(ID3D11DeviceContext* pd3dImmediateContext, DrawingUtilitiesClass* DUC);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	//void integrate(float elapsedTime);
	void addRigidBody(Vec3 position, Vec3 size, float mass);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	//void pullTogether();

	void initTestScene();

	std::vector<Rigidbody> m_rigidbodies;
	std::vector<Spring> m_springList;

	std::random_device rd;  //Will be used to obtain a seed for the random number engine


private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	float m_elasticity;
	float m_fGravity;
	float m_timeFactor;
	int m_iIntegrator;
	float m_fStiffness;
	float m_fDamping;
};