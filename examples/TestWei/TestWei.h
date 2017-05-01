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
	TestWei(struct GUIHelperInterface* helper, int testCase);

	virtual ~TestWei() {}
	virtual void initPhysics() override ;
	virtual void createEmptyDynamicsWorld() override;
	virtual void renderScene() override;

	void resetCamera()
	{
	}
    virtual void stepSimulation(float deltaTime) override; 

	virtual bool keyboardCallback(int key, int state) override;
	void createUI();
	void drawDebugText();


	// setup test cases
	void setupCase0();
	void setupCase1();
	void setupCase2();
	void setupCase3(); // rolling friction
private:
	void step(float deltaTime);
private:
	btRigidBody* m_body = nullptr;
	bool m_paused = false;
	DemoSolverType m_solverType;
	int m_testCase = 0;
};


