#pragma once
#include "Simulator.h"
#include "RigidbodySystem.h"
#include "global.h"

class Spring
{
public:
	Spring();
	Spring(RigidbodySystem& point1, RigidbodySystem& point2, float stiffness, float initialLength);
	~Spring();

public:
	RigidbodySystem *mass_point1;
	RigidbodySystem *mass_point2;
	float stiffness;
	float initialLength;
	float currentLength;
	Vec3 force;

	float calcDirectedForce(float currentLength, float pos1, float pos2);
	void computeElasticForces();
	void addToEndPoints();

	void setPoint1(RigidbodySystem *point);
	void setPoint2(RigidbodySystem *point);
	void setStiffness(float stiff);
	void setInitalLength(float initlength);

	RigidbodySystem *getPoint1();
	RigidbodySystem *getPoint2();
	float getStiffness();
	float getInitialLength();
};

