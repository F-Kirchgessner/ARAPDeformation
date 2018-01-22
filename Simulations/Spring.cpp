#include "Spring.h"

Spring::Spring(){
}


Spring::Spring(Rigidbody& point1, Rigidbody& point2, Vec3& pos1, Vec3& pos2, float stiffness, float initialLength){
	Vec3 force = Vec3(0, 0, 0);
	mass_point1 = &point1;
	mass_point2 = &point2;
	this->pos1 = pos1;
	this->pos2 = pos2;
	this->stiffness = stiffness;
	this->initialLength = initialLength;
}


Spring::~Spring(){
	mass_point1 = nullptr;
	mass_point2 = nullptr;
}


float Spring::calcDirectedForce(float currentLength, float pos1, float pos2) {
	return -stiffness * (currentLength - initialLength) * ((pos1-pos2) / currentLength);
}


void Spring::computeElasticForces() {

	Vec3 tmpPos1 = mass_point1->m_position - pos1;
	Vec3 tmpPos2 = mass_point2->m_position - pos2;


	float currentLength = sqrt(tmpPos1.squaredDistanceTo(tmpPos2));
	if (currentLength < SPRING_LENGTH_MIN)
		currentLength = SPRING_LENGTH_MIN;

	this->force.x = calcDirectedForce(currentLength, tmpPos1.x, tmpPos2.x);
	this->force.y = calcDirectedForce(currentLength, tmpPos1.y, tmpPos2.y);
	this->force.z = calcDirectedForce(currentLength, tmpPos1.z, tmpPos2.z);

	//std::cout << force.x << " | " << force.y << " | " << force.z << endl;
}

void Spring::addToEndPoints() {
	mass_point1->applyForce(pos1,force);
	mass_point2->applyForce(pos2,-force);
}

void Spring::setStiffness(float stiff) {
	stiffness = stiff;
}
void Spring::setInitalLength(float initlength) {
	initialLength = initlength;
}

float Spring::getStiffness() {
	return stiffness;
}

float Spring::getInitialLength() {
	return initialLength;
}