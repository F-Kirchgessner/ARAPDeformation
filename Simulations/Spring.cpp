#include "Spring.h"

Spring::Spring(){
}


Spring::Spring(Masspoint& point1, Masspoint& point2, float stiffness, float initialLength){
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

	float currentLength = sqrt(mass_point1->position.squaredDistanceTo(mass_point2->position));
	if (currentLength < SPRING_LENGTH_MIN)
		currentLength = SPRING_LENGTH_MIN;

	this->force.x = calcDirectedForce(currentLength, mass_point1->position.x, mass_point2->position.x);
	this->force.y = calcDirectedForce(currentLength, mass_point1->position.y, mass_point2->position.y);
	this->force.z = calcDirectedForce(currentLength, mass_point1->position.z, mass_point2->position.z);

	//std::cout << force.x << " | " << force.y << " | " << force.z << endl;
}

void Spring::addToEndPoints() {
	mass_point1->applyForce(force);
	mass_point2->applyForce(-force);
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