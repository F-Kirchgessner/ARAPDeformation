#pragma once
#include "Simulator.h"
#include "util\matrixbase.h"

#include <DirectXMath.h>
#include "collisionDetect.h"


class RigidbodySystem
{
public:
	RigidbodySystem();
	RigidbodySystem(Vec3 size, Vec3 position, float mass);
	~RigidbodySystem();

	// x_cm
	Vec3 m_position;
	// Translation Matrix
	Mat4 transMat;

	Vec3 size;
	//Scale Matrix
	Mat4 scaleMat;

	// For test purpose only! One axis, hence orientation is a scalar i.e. radian [0...2pi]! 
	// ToDo use quaterions!
	// r
	Quat orientation;
	//RotationMatrix;
	Mat4 rotMat;
	
	Mat4 Obj2WorldMatrix;

	//M
	float mass;
	//i
	Mat4 interiatensor;
	Mat4 interiatensorInv;
	//v_cm
	Vec3 velocity;
	//F
	Vec3 force;
	//w
	Vec3 angluarvelocity;
	//q
	Vec3 torque;
	//L
	Vec3 angularMomentum;

	// add Force and Torque, External Forces in the Simulation Algo.
	void applyForce(Vec3& loc, Vec3& f);
	void updateStep(float elapsedTime);
	void calculateInteriaTensor();
	void clearForce();
	void collisionDetected(Vec3 collisionPointWorld, Vec3 normal);

	float red;
	float green;
	float blue;
};

