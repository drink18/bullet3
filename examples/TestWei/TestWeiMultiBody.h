#pragma once

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"

class CommonExampleInterface* TestWeiMBCreateFunc(struct CommonExampleOptions& options);
class btMultiBody;
class btMultiBodyDynamicsWorld;

class TestWeiMultiBody : public CommonMultiBodyBase
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
	btMultiBody* createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld *pWorld, int numLinks, const btVector3 &basePosition, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents, bool spherical, bool floating);
	void addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents);
	void spawnBox(const btTransform& initTrans, const btVector3& halfExt);
private:
	bool m_paused;
	DemoSolverType m_solverType;
	int m_testCase;
};


