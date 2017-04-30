#pragma once

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

class CommonExampleInterface* TestWeiCreateFunc(struct CommonExampleOptions& options);

class TestWei : public CommonRigidBodyBase
{
public:
	enum DemoSolverType
	{
		BT_SequentialSolver  = 0,
		WeiSolver = 1,
		SovlerType_count
	};

public:
	TestWei(struct GUIHelperInterface* helper);

	virtual ~TestWei() {}
	virtual void initPhysics() override ;
	virtual void createEmptyDynamicsWorld() override;
	virtual void renderScene() override;

	void resetCamera()
	{
		float dist = 4;
		float pitch = 0;
		float yaw = 35;
		float targetPos[3] = { 0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
    virtual void stepSimulation(float deltaTime) override; 

	virtual bool keyboardCallback(int key, int state) override;
	void createUI();
	void drawDebugText();
private:
	void step(float deltaTime);
private:
    btRigidBody* m_body = nullptr;
	bool m_paused = false;
	DemoSolverType m_solverType;
};


