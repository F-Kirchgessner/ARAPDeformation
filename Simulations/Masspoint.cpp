#include "Masspoint.h"

Masspoint::Masspoint() {
}

Masspoint::Masspoint(Vec3 position, Vec3 velocity, bool isFixed, Vec3 force, float mass, float damping) : position(position), velocity(velocity), isFixed(isFixed), force(force), mass(mass), damping(damping) {
}


Masspoint::~Masspoint() {
}

void Masspoint::clearForce() {
	force.x = 0;
	force.y = 0;
	force.z = 0;
}

void Masspoint::addGravity(float gravityAccel) {
	force.y -= mass * gravityAccel;
}

void Masspoint::integratePositionsLeapfrog(float elapsedTime) {
	if (!isFixed) {
		position.x += velocity.x * elapsedTime;
		position.y += velocity.y * elapsedTime;
		position.z += velocity.z * elapsedTime;

		if (position.y < GROUND_HEIGHT) {
			position.y = GROUND_HEIGHT;
			velocity.y = -velocity.y * GROUND_DAMPING;
		}

		if (position.x < -BOX_SIZE) {
			position.x = -BOX_SIZE;
			velocity.x = -velocity.x * GROUND_DAMPING;
		}
		else if (position.x > BOX_SIZE) {
			position.x = BOX_SIZE;
			velocity.x = -velocity.x * GROUND_DAMPING;
		}

		if (position.z < -BOX_SIZE) {
			position.z = -BOX_SIZE;
			velocity.z = -velocity.z * GROUND_DAMPING;
		}
		else if (position.z > BOX_SIZE) {
			position.z = BOX_SIZE;
			velocity.z = -velocity.z * GROUND_DAMPING;
		}
	}
}

void Masspoint::integratePositionsEuler(float elapsedTime) {
	if (!isFixed) {
		position.x += velocity.x * elapsedTime;
		position.y += velocity.y * elapsedTime;
		position.z += velocity.z * elapsedTime;

		if (position.y < GROUND_HEIGHT) {
			position.y = GROUND_HEIGHT;
			velocity.y = -velocity.y * GROUND_DAMPING;
		}

		if (position.x < -BOX_SIZE) {
			position.x = -BOX_SIZE;
			velocity.x = -velocity.x * GROUND_DAMPING;
		}
		else if (position.x > BOX_SIZE) {
			position.x = BOX_SIZE;
			velocity.x = -velocity.x * GROUND_DAMPING;
		}

		if (position.z < -BOX_SIZE) {
			position.z = -BOX_SIZE;
			velocity.z = -velocity.z * GROUND_DAMPING;
		}
		else if (position.z > BOX_SIZE) {
			position.z = BOX_SIZE;
			velocity.z = -velocity.z * GROUND_DAMPING;
		}
	}
}

void Masspoint::initVelocity(float halfElapsedTime) {
	velocity.x = velocity.x + (force.x / mass) * halfElapsedTime;
	velocity.y = velocity.y + (force.y / mass) * halfElapsedTime;
	velocity.z = velocity.z + (force.z / mass) * halfElapsedTime;

	if (velocity.x * velocity.x < VELOCITY_MIN_SQ)
		velocity.x = 0;
	if (velocity.y * velocity.y < VELOCITY_MIN_SQ)
		velocity.y = 0;
	if (velocity.z * velocity.z < VELOCITY_MIN_SQ)
		velocity.z = 0;
}

void Masspoint::integrateVelocityLeapfrog(float elapsedTime) {
	velocity.x += ((force.x - damping * velocity.x) / mass) * elapsedTime;
	velocity.y += ((force.y - damping * velocity.y) / mass) * elapsedTime;
	velocity.z += ((force.z - damping * velocity.z) / mass) * elapsedTime;

	if (velocity.x * velocity.x < VELOCITY_MIN_SQ)
		velocity.x = 0;
	if (velocity.y * velocity.y < VELOCITY_MIN_SQ)
		velocity.y = 0;
	if (velocity.z * velocity.z < VELOCITY_MIN_SQ)
		velocity.z = 0;
}

void Masspoint::integrateMidpointPosTemp(float elapsedTime, vector<Vec3>& PosTemp) {
	Vec3 tmp;

	tmp.x = position.x + elapsedTime * velocity.x;
	tmp.y = position.y + elapsedTime * velocity.y;
	tmp.z = position.z + elapsedTime * velocity.z;

	PosTemp.push_back(tmp);
}

void Masspoint::integrateMidpointVelTemp(float elapsedTime, vector<Vec3>& VelTemp) {
	Vec3 tmp;

	tmp.x = velocity.x + ((force.x - damping * velocity.x) / mass) * elapsedTime;
	tmp.y = velocity.y + ((force.y - damping * velocity.y) / mass) * elapsedTime;
	tmp.z = velocity.z + ((force.z - damping * velocity.z) / mass) * elapsedTime;

	VelTemp.push_back(tmp);
}

void Masspoint::integrateSwitch(vector <Vec3>& VelTemp, vector <Vec3>& PosTemp, vector <Vec3>& oldVel, vector <Vec3>& oldPos, int index) {
	oldPos.push_back(position);
	oldVel.push_back(velocity);

	position.x = PosTemp[index].x;
	position.y = PosTemp[index].y;
	position.z = PosTemp[index].z;

	velocity.x = VelTemp[index].x;
	velocity.y = VelTemp[index].y;
	velocity.z = VelTemp[index].z;

}

void Masspoint::integrateSwitchBack(vector <Vec3>& oldVel, vector <Vec3>& oldPos, int index) {
	position.x = oldPos[index].x;
	position.y = oldPos[index].y;
	position.z = oldPos[index].z;

	velocity.x = oldVel[index].x;
	velocity.y = oldVel[index].y;
	velocity.z = oldVel[index].z;

}

void Masspoint::computeX(float elapsedTime, vector <Vec3>& VelTemp, int index) {
	if (!isFixed) {
		position.x += elapsedTime * VelTemp[index].x;
		position.y += elapsedTime * VelTemp[index].y;
		position.z += elapsedTime * VelTemp[index].z;

		if (position.y < GROUND_HEIGHT) {
			position.y = GROUND_HEIGHT;
			velocity.y = -velocity.y * GROUND_DAMPING;
		}

		if (position.x < -BOX_SIZE) {
			position.x = -BOX_SIZE;
			velocity.x = -velocity.x * GROUND_DAMPING;
		}
		else if (position.x > BOX_SIZE) {
			position.x = BOX_SIZE;
			velocity.x = -velocity.x * GROUND_DAMPING;
		}

		if (position.z < -BOX_SIZE) {
			position.z = -BOX_SIZE;
			velocity.z = -velocity.z * GROUND_DAMPING;
		}
		else if (position.z > BOX_SIZE) {
			position.z = BOX_SIZE;
			velocity.z = -velocity.z * GROUND_DAMPING;
		}
	}
}

void Masspoint::computeY(float elapsedTime, vector <Vec3>& VelTemp, int index) {
	velocity.x += ((force.x - damping * VelTemp[index].x) / mass) * elapsedTime;
	velocity.y += ((force.y - damping * VelTemp[index].y) / mass) * elapsedTime;
	velocity.z += ((force.z - damping * VelTemp[index].z) / mass) * elapsedTime;

	if (velocity.x * velocity.x < VELOCITY_MIN_SQ)
		velocity.x = 0;
	if (velocity.y * velocity.y < VELOCITY_MIN_SQ)
		velocity.y = 0;
	if (velocity.z * velocity.z < VELOCITY_MIN_SQ)
		velocity.z = 0;

}

void Masspoint::integrateVelocityEuler(float elapsedTime) {
	velocity.x += ((force.x - damping * velocity.x) / mass) * elapsedTime;
	velocity.y += ((force.y - damping * velocity.y) / mass) * elapsedTime;
	velocity.z += ((force.z - damping * velocity.z) / mass) * elapsedTime;

	if (velocity.x * velocity.x < VELOCITY_MIN_SQ)
		velocity.x = 0;
	if (velocity.y * velocity.y < VELOCITY_MIN_SQ)
		velocity.y = 0;
	if (velocity.z * velocity.z < VELOCITY_MIN_SQ)
		velocity.z = 0;
}

void Masspoint::applyForce(Vec3 force) {
	this->force += force;
}

void Masspoint::setPosition(Vec3 position) {
	this->position = position;
}
void Masspoint::setVelocity(Vec3 velocity) {
	this->velocity = velocity;
}
void Masspoint::setForce(Vec3 force) {
	this->force = force;
}
void Masspoint::setIsFixed(bool isFixed) {
	this->isFixed = isFixed;
}
void Masspoint::setMass(float mass) {
	this->mass = mass;
}
void Masspoint::setDamping(float damping) {
	this->damping = damping;
}
Vec3 Masspoint::getPosition() {
	return position;
}
Vec3 Masspoint::getVelocity() {
	return velocity;
}
Vec3 Masspoint::getForce() {
	return force;
}
bool  Masspoint::getIsFixed() {
	return isFixed;
}
float Masspoint::getMass() {
	return mass;
}
float Masspoint::getDamping() {
	return damping;
}