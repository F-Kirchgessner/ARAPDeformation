#pragma once
#include "Simulator.h"

#include "global.h"

class Masspoint
{
public:
	Masspoint();
	Masspoint(Vec3 position, Vec3 velocity, bool isFixed, Vec3 force, float mass, float damping);
	~Masspoint();

public:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool isFixed;
	float mass;
	float damping;

	void clearForce();
	void addGravity(float gravityAccel);
	void integratePositionsLeapfrog(float elapsedTime);
	void integratePositionsEuler(float elapsedTime);
	void integrateMidpointPosTemp(float elapsedTime, vector<Vec3>& PosTemp);
	void integrateMidpointVelTemp(float elapsedTime, vector<Vec3>& VelTemp);
	void integrateSwitch(vector <Vec3>& VelTemp, vector <Vec3>& PosTemp, vector <Vec3>& oldVel, vector <Vec3>& oldPos, int index);
	void integrateSwitchBack(vector <Vec3>& oldVel, vector <Vec3>& oldPos, int index);
	void computeX(float elapsedTime,vector <Vec3>& VelTemp, int index);
	void computeY(float elapsedTime, vector <Vec3>& VelTemp, int index);

	void integrateVelocityLeapfrog(float elapsedTime);
	void integrateVelocityEuler(float elapsedTime);
	void applyForce(Vec3 force);
	void initVelocity(float halfElapsedTime);

	void setPosition(Vec3 position);
	void setVelocity(Vec3 velocity);
	void setIsFixed(bool isFixed);
	void setForce(Vec3 force);
	void setMass(float mass);
	void setDamping(float damping);

	Vec3 getPosition();
	Vec3 getVelocity();
	Vec3 getForce();
	bool getIsFixed();
	float getMass();
	float getDamping();
};