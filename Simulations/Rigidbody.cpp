#include "Rigidbody.h"



Rigidbody::Rigidbody()
{

}

Rigidbody::Rigidbody(Vec3 size, Vec3 position, float mass, float dampingVel, float dampingRot, bool fixed, bool visible) : size(size), m_position(position), mass(mass), dampingVel(dampingVel), dampingRot(dampingRot), isFixed(fixed), visible(visible)
{
	// 45 deg. 
	// need to be removed
	// ------------------------
	//orientation = sqrt(2) / 2;
	// ------------------------
	orientation = Quat(0.0f, 0.0f, 0.0f, 1.0f);
	rotMat = orientation.getRotMat();
	transMat.initTranslation(position.x, position.y, position.z);
	scaleMat.initScaling(size.x, size.y, size.z);
	angluarvelocity = 0;
	calculateInteriaTensor();
	interiatensorInv = interiatensor.inverse();
}

Rigidbody::~Rigidbody()
{
}

void Rigidbody::applyForce(Vec3& loc, Vec3& f)
{
	//for 3D vector
	// F += f;
	// xi = loc - center
	// torque += xi x force
	
	force += f;
	//loc is probably in world space as m_position
	// armvector = x - loc
	//Vec3 armVector = loc - m_position;
	//Watch out should be += not =
	torque += GamePhysics::cross(loc, f);

}

void Rigidbody::updateStep(float elapsedTime)
{
	if (!isFixed) {
		float h = elapsedTime;
		Mat4 rotation = orientation.getRotMat();
		Mat4 rotMatTranspose = rotation;
		rotMatTranspose.transpose();

			m_position += h * velocity;
			//velocity += h * (force / mass);
			//angularMomentum += h * torque;
			velocity += h * ((force - dampingVel * velocity) / mass);
			angularMomentum += h * (torque * 0.5 - dampingRot * angularMomentum);

		Mat4 tempInteriatensor = rotation * interiatensorInv * rotMatTranspose;
		//angluarvelocity = tempInteriatensor * angularMomentum;
		angluarvelocity = tempInteriatensor.transformVector(angularMomentum);

		orientation += h / 2.0f * Quat(angluarvelocity.x, angluarvelocity.y, angluarvelocity.z,0) * orientation;
		orientation = orientation.unit();

		transMat.initTranslation(m_position.x, m_position.y, m_position.z);
		rotMat = orientation.getRotMat();

		clearForce();
	}
}

void Rigidbody::clearForce() {
	force.x = 0;
	force.y = 0;
	force.z = 0;

	torque.x = 0;
	torque.y = 0;
	torque.z = 0;
}

void Rigidbody::calculateInteriaTensor() {	
	//Rotation around z-achse
	//float a = size.x;
	//float b = size.y;
	//interiatensor = mass*(a*a + b*b) / 12.0f;

	float w = size.x;
	float h = size.y;
	float d = size.z;

	interiatensor = Mat4((mass*(h*h + d*d))/12.0f, 0.0f, 0.0f, 0.0f,
						 0.0f, (mass*(w*w + d*d))/12.0f, 0.0f, 0.0f,
					     0.0f, 0.0f, (mass*(w*w + h*h))/12.0f, 0.0f,
						 0.0f, 0.0f, 0.0f, 1.0f);
};

void Rigidbody::addGravity(float gravityAccel) {
	force.y -= mass * gravityAccel;
}