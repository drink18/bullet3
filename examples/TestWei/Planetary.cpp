#include "btBulletCollisionCommon.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "Planetary.h"


Planetary::Planetary(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper)
{
}

btRigidBody* Planetary::createKinematicBody(const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	btRigidBody* body = new btRigidBody(0.0f, NULL, shape);
	body->setWorldTransform(startTransform);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setUserIndex(-1);
	m_dynamicsWorld->addRigidBody(body);
	return body;
}

void Planetary::initPhysics()
{
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	btSphereShape* sphere = new btSphereShape(1.0f);
    btTransform trans;trans.setIdentity();
    {
        trans.setOrigin(btVector3(10, 0, 0));
        btRigidBody* planet = createRigidBody(10.0f, trans, sphere);
        planet->setLinearVelocity(btVector3(0, 0, 10));
        m_planets.push_back(planet);
    }

    {
        trans.setOrigin(btVector3(0, 10, 0));
        btRigidBody* planet = createRigidBody(10.0f, trans, sphere);
        planet->setLinearVelocity(btVector3(0, 0, 10));
        m_planets.push_back(planet);
    }

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
}


void Planetary::createEmptyDynamicsWorld()
{
	CommonRigidBodyBase::createEmptyDynamicsWorld();
}

void Planetary::stepSimulation(float dt)
{

    for(int i = 0; i < m_planets.size(); ++i)
    {
        simulatePlanet(m_planets[i]);
    }

	CommonRigidBodyBase::stepSimulation(dt);
}

void Planetary::simulatePlanet(btRigidBody* body)
{
	btVector3 v = body->getWorldTransform().getOrigin();
	v *= -1;
	btScalar r = v.length();
	const btScalar g = 1000.0f;
	if (!btFuzzyZero(r))
	{
		v /= r;
		btScalar r2 = r * r;

		btVector3 f = v * g / r2 / body->getInvMass();
		body->applyCentralForce(f);
	}

}

bool Planetary::keyboardCallback(int key, int state)
{
	return CommonRigidBodyBase::keyboardCallback(key, state);
}


CommonExampleInterface* PlanetaryCreateFunc(CommonExampleOptions& options)
{
	return new Planetary(options.m_guiHelper);
}


B3_STANDALONE_EXAMPLE( PlanetaryCreateFunc)
