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
		BT_NNCG = 3,
		BT_WEI_NNCG = 4,
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
		Wei_ChainBridge,
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
	void setupChainBridge(); 
private:
	void step(float deltaTime);
	void spawnBox(const btTransform& initTrans, const btVector3& halfExt);
private:
	bool m_paused;
	DemoSolverType m_solverType;
	int m_testCase;
};


