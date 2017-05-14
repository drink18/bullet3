#pragma once

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

class CommonExampleInterface* TestWeiCreateFunc(struct CommonExampleOptions& options);

class TestWeiMultiBody : public CommonRigidBodyBase
{
public:
	enum DemoSolverType
	{
		BT_SequentialSolver  = 0,
		WeiSolver = 1,
		BT_MLCP = 2,
		SovlerType_count
	};

	enum DemoScene
	{
		Wei_ChainBridge = 0
	};
public:
	TestWeiMultiBody(struct GUIHelperInterface* helper, int testCase);

	virtual ~TestWeiMultiBody() {}
	virtual void initPhysics();
	virtual void createEmptyDynamicsWorld();
	virtual void renderScene();

	void resetCamera()
	{
	}
    virtual void stepSimulation(float deltaTime); 

	virtual bool keyboardCallback(int key, int state);
	void createUI();
	void drawDebugText();


	// setup test cases
	void setupChainBridge(); 
private:
	void step(float deltaTime);
private:
	bool m_paused;
	DemoSolverType m_solverType;
	int m_testCase;
};


