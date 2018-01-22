#include "Spring.h"

Spring::Spring(){
}


Spring::Spring(RigidbodySystem& point1, RigidbodySystem& point2, float stiffness, float initialLength){
	Vec3 force = Vec3(0, 0, 0);
	mass_point1 = &point1;
	mass_point2 = &point2;
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

	float currentLength = sqrt(mass_point1->m_position.squaredDistanceTo(mass_point2->m_position));
	if (currentLength < SPRING_LENGTH_MIN)
		currentLength = SPRING_LENGTH_MIN;

	this->force.x = calcDirectedForce(currentLength, mass_point1->m_position.x, mass_point2->m_position.x);
	this->force.y = calcDirectedForce(currentLength, mass_point1->m_position.y, mass_point2->m_position.y);
	this->force.z = calcDirectedForce(currentLength, mass_point1->m_position.z, mass_point2->m_position.z);

	//std::cout << force.x << " | " << force.y << " | " << force.z << endl;
}

void Spring::addToEndPoints() {
	mass_point1->applyForce(Vec3(0,0,0),force);
	mass_point2->applyForce(Vec3(0, 0, 0),-force);
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