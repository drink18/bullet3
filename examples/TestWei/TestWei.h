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
		BT_MLCP = 2,
		SovlerType_count
	};

	enum DemoScene
	{
		Wei_BasicExmaple = 0,
		Wei_SlopeDemo,
		Wei_Constraint,
		Wei_SoftContact,
		Wei_StressTest,
		Wei_StressChainTest,
	};
public:
	TestWei(struct GUIHelperInterface* helper, int testCase);

	virtual ~TestWei() {}
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
	void setupCase0();
	void setupSlopeDemo();
	void setupDemoConstraints();
	void setupSoftContact(); // rolling friction
	void setupStressTest(); 
	void setupStress_Chain(); 
private:
	void step(float deltaTime);
private:
	bool m_paused;
	DemoSolverType m_solverType;
	int m_testCase;
};


