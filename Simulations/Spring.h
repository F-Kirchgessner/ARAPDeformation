#pragma once
#include "Simulator.h"
#include "Rigidbody.h"
#include "global.h"

class Spring
{
public:
	Spring();
	Spring(Rigidbody& point1, Rigidbody& point2, Vec3& pos1, Vec3& pos2, float stiffness, float initialLength);
	~Spring();

public:
	Rigidbody *mass_point1;
	Rigidbody *mass_point2;
	float stiffness;
	float initialLength;
	float currentLength;
	Vec3 force;

	Vec3 posOrig1;
	Vec3 posOrig2;
	Vec3 pos1;
	Vec3 pos2;

	float calcDirectedForce(float currentLength, float pos1, float pos2);
	void computeElasticForces();
	void addToEndPoints();

	void setStiffness(float stiff);
	void setInitalLength(float initlength);

	float getStiffness();
	float getInitialLength();
};

