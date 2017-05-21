#pragma once

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

class CommonExampleInterface* PlanetaryCreateFunc(struct CommonExampleOptions& options);

class Planetary: public CommonRigidBodyBase
{
public:
	 Planetary(struct GUIHelperInterface* helper);

	virtual ~Planetary() {}
	virtual void initPhysics();
	virtual void createEmptyDynamicsWorld();
	btRigidBody* createKinematicBody(const btTransform& startTransform, btCollisionShape* shape);
    void simulatePlanet(btRigidBody* planet);

	void resetCamera()
	{
	}
    virtual void stepSimulation(float deltaTime); 

	virtual bool keyboardCallback(int key, int state);

    btAlignedObjectArray<btRigidBody*> m_planets;
};


