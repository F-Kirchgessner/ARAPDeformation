#pragma once
#include "Simulator.h"
#include "Masspoint.h"

class Spring
{
public:
	Spring();
	Spring(Masspoint& point1, Masspoint& point2, float stiffness, float initialLength);
	~Spring();

public:
	Masspoint *mass_point1;
	Masspoint *mass_point2;
	float stiffness;
	float initialLength;
	float currentLength;
	Vec3 force;

	float calcDirectedForce(float currentLength, float pos1, float pos2);
	void computeElasticForces();
	void addToEndPoints();

	void setPoint1(Masspoint *point);
	void setPoint2(Masspoint *point);
	void setStiffness(float stiff);
	void setInitalLength(float initlength);

	Masspoint *getPoint1();
	Masspoint *getPoint2();
	float getStiffness();
	float getInitialLength();
};

