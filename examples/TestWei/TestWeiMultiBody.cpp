#include "TestWeiMultiBody.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "BulletDynamics/ConstraintSolver/btWeiSISolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"



namespace
{

	const char* getSolverTypeName(TestWeiMultiBody::DemoSolverType type)
	{

		const char* name = "";
		switch (type)
		{
		case TestWeiMultiBody::BT_SequentialSolver:
			name = "Sequential solver";
			break;
		case  TestWeiMultiBody::WeiSolver:
			name = "Wei solver";
			break;
		case TestWeiMultiBody::BT_MLCP:
			name = "MLCP solver";
			break;
		default:
			break;
		}

		return name;
	}

	TestWeiMultiBody::DemoSolverType gDemoSolverType = TestWeiMultiBody::WeiSolver;
	btScalar gSolverIterations = 10;
	void setSolverTypeCallback(int buttonId, bool buttonState, void* userPointer)
	{
		gDemoSolverType = static_cast<TestWeiMultiBody::DemoSolverType>(buttonId);
	}
	void setSolverIterationCountCallback(float val, void* userPtr)
	{
		if (btDiscreteDynamicsWorld* world = reinterpret_cast<btDiscreteDynamicsWorld*>(userPtr))
		{
			world->getSolverInfo().m_numIterations = btMax(1, int(gSolverIterations));
		}
	}

	btScalar gBoxMass = 10.0f;
	void setBoxMassCallback(float val, void* userPtr)
	{
		gBoxMass = val;
	}
}

void TestWeiMultiBody::spawnBox(const btTransform& initTrans, const btVector3& halfExt)
{
	btBoxShape* shape = new btBoxShape(halfExt);
	createRigidBody(gBoxMass, initTrans, shape);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

TestWeiMultiBody::TestWeiMultiBody(struct GUIHelperInterface* helper, int testCase)
	:CommonMultiBodyBase(helper),
	m_testCase(testCase),
    m_paused(false)
{
}


void TestWeiMultiBody::initPhysics()
{
	m_solver = new btMultiBodyConstraintSolver;
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	btBoxShape* groundShape = createBoxShape(btVector3(50, 50, 50));

	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	createRigidBody(0, groundTransform, groundShape, btVector4(0, 0, 1, 1));


	switch (m_testCase)
	{
    case Wei_ChainBridge:
        setupChainBridge();
        break;
	default:
		break;
	}
    //m_body->setLinearVelocity(btVector3(-3.5f, 0, 0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);


#if  !defined(B3_USE_STANDALONE_EXAMPLE)
	createUI();
#endif
}

void TestWeiMultiBody::createUI()
{
    // add buttons for switching to different solver types
	//for (int i = 0; i < SovlerType_count; ++i)
	//{
	//	//char* buttonName = "Sequential impulse solver";
	//	char buttonName[256];
	//	DemoSolverType solverType = static_cast<DemoSolverType>(i);
	//	sprintf(buttonName, "Solver: %s", getSolverTypeName(solverType));
	//	ButtonParams button(buttonName, 0, false);
	//	button.m_buttonId = solverType;
	//	button.m_callback = setSolverTypeCallback;
	//	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	//}
	{
		// a slider for the number of solver iterations
		SliderParams slider("Solver iterations", &gSolverIterations);
		slider.m_minVal = 1.0f;
		slider.m_maxVal = 200; 
		slider.m_callback = setSolverIterationCountCallback;
		slider.m_userPointer = m_dynamicsWorld;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Box Mass", &gBoxMass);
		slider.m_minVal = 10.0f;
		slider.m_maxVal = 200.0f;
		slider.m_userPointer = nullptr;
		slider.m_callback = setBoxMassCallback;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
}

void TestWeiMultiBody::renderScene()
{
	drawDebugText();
	CommonMultiBodyBase::renderScene();
}

void TestWeiMultiBody::drawDebugText()
{
	{
		const btScalar lineH = 30.0f;
		btScalar y = 40;
		btScalar x = 300;
		m_guiHelper->getAppInterface()->drawText(getSolverTypeName(gDemoSolverType), x, y, 0.4f);
		y += lineH;
		if (m_solverType != gDemoSolverType)
		{
			m_guiHelper->getAppInterface()->drawText("Restart Demo to use new solver", x, y, 0.4f);
			y += lineH;
		}
	}
}

void TestWeiMultiBody::createEmptyDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase, m_solver, m_collisionConfiguration);
}

void TestWeiMultiBody::stepSimulation(float deltaTime)
{
	if (m_paused)
		return;
	step(deltaTime);
}

void TestWeiMultiBody::step(float deltaTime)
{
	CommonMultiBodyBase::stepSimulation(deltaTime);
}



bool TestWeiMultiBody::keyboardCallback(int key, int state)
{
	if (key == B3G_F5 && state == 0)
	{
		m_paused = !m_paused;
		return false;
	}

	if (key == B3G_F6 && state == 0)
	{
		m_paused = true;
		step(1.0f / 60);
		return false;
	}

	if (key == B3G_F7 && state == 0)
	{
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(btVector3(0, 7, 0));
		spawnBox(trans, btVector3(0.5f, 0.5f, 0.5f)) ;
	}

	return CommonMultiBodyBase::keyboardCallback(key, state);
}


void TestWeiMultiBody::setupChainBridge()
{
	// camera setup
	{
		float dist = 11.7f;
		float pitch = -10.0f;
		float yaw = 35;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	{
		const btScalar pillarH = 3.0f;
		btVector3 baseHalfExt(0.5f, pillarH, 0.5f);
		btVector3 linkHalfExt(0.5f, 0.05f, 0.5f);
		const int numSeg = 8;

		btScalar linkMass = 1.0f;
		btVector3 linkIteriaDiag(0, 0, 0);
		btCollisionShape *linkShape = new btBoxShape(linkHalfExt);
		linkShape->calculateLocalInertia(linkMass, linkIteriaDiag);
		delete linkShape;
		btMultiBody* bridgeMB = new btMultiBody(numSeg, 0, btVector3(0, 0, 0), true, true);

		//init base
		btQuaternion baseQuat = btQuaternion::getIdentity();
		bridgeMB->setBasePos(btVector3(-4.0f, pillarH, 0));
		bridgeMB->setWorldToBaseRot(baseQuat);

		//init links
		btVector3 hingeJointAxis(0, 0, 1);

		btVector3 parentComToCurrentCom(linkHalfExt[0] + baseHalfExt[0], baseHalfExt[1] + linkHalfExt[1], 0);
		btVector3 currentPivotToCurrentCom(linkHalfExt[0], 0, 0);
		btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;
		btQuaternion rotParentToLink = btQuaternion::getIdentity();// btVector3(0, 0, 1), SIMD_PI / 2);

		btScalar q0 = 0.f * SIMD_PI / 180.0f;
		btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);

		for (int i = 0; i < numSeg; ++i)
		{
			bridgeMB->setupRevolute(i, linkMass, linkIteriaDiag, i - 1,
				rotParentToLink,
				hingeJointAxis, parentComToCurrentPivot,
				currentPivotToCurrentCom, true);

			rotParentToLink = btQuaternion::getIdentity();
			parentComToCurrentCom.setValue(linkHalfExt[0] * 2, 0, 0);
			currentPivotToCurrentCom.setValue(linkHalfExt[0], 0, 0);
			parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;
		}
		bridgeMB->finalizeMultiDof();
		((btMultiBodyDynamicsWorld*)m_dynamicsWorld)->addMultiBody(bridgeMB);

		addColliders_testMultiDof(bridgeMB, (btMultiBodyDynamicsWorld*)m_dynamicsWorld, baseHalfExt, linkHalfExt);

		btMatrix3x3 frameInA; frameInA.setIdentity();
		btMatrix3x3 frameInB; frameInB.setIdentity();
		btMultiBodyFixedConstraint* fcA = new btMultiBodyFixedConstraint(bridgeMB, 0, 
			bridgeMB, -1, 
			btVector3(linkHalfExt[0], 0, 0), 
			btVector3(baseHalfExt[0], baseHalfExt[1], 0),
			frameInA, frameInB
			);
		btScalar relaxe = 0.3f;
		btMultiBodyFixedConstraint* fcB = new btMultiBodyFixedConstraint(bridgeMB, numSeg - 1,
			bridgeMB, -1, 
			btVector3(linkHalfExt[0], 0, 0), 
			btVector3(linkHalfExt[0] * numSeg  * 2 - relaxe + baseHalfExt[0], baseHalfExt[1], 0),
			frameInA, frameInB
			);

		m_dynamicsWorld->addMultiBodyConstraint(fcA);
		m_dynamicsWorld->addMultiBodyConstraint(fcB);
	}
}

btMultiBody* TestWeiMultiBody::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld *pWorld, int numLinks, const btVector3 &basePosition, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents, bool spherical, bool floating)
{
	//init the base	
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;
	
	if(baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;
	
	btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);
//	pMultiBody->setBaseVel(vel);

	//init the links	
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape *pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);						//par body's COM to cur body's COM offset	
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);							//cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset

	//////
	btScalar q0 = 0.f * SIMD_PI/ 180.f;
	btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
	quat0.normalize();	
	/////

	for(int i = 0; i < numLinks; ++i)
	{
		if(!spherical)			
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	}

	pMultiBody->finalizeMultiDof();

	///
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void TestWeiMultiBody::addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents)
{			
	
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();
	
	const btScalar friction = 0.1f;
	{
	//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
		btScalar quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};

			
		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);			
			btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(pMultiBody, -1);			
			col->setCollisionShape(box);
								
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));			
			col->setWorldTransform(tr);
				
			pWorld->addCollisionObject(col, 2,1+2);

	

			col->setFriction(friction);
			pMultiBody->setBaseCollider(col);
				
		}
	}


	for (int i=0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i+1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent+1];
		local_origin[i+1] = local_origin[parent+1] + (quatRotate(world_to_local[i+1].inverse() , pMultiBody->getRVector(i)));
	}

		
	for (int i=0; i < pMultiBody->getNumLinks(); ++i)
	{
		
		btVector3 posr = local_origin[i+1];
	//	float pos[4]={posr.x(),posr.y(),posr.z(),1};
			
		btScalar quat[4]={-world_to_local[i+1].x(),-world_to_local[i+1].y(),-world_to_local[i+1].z(),world_to_local[i+1].w()};

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		pWorld->addCollisionObject(col,2,1+2);
	
			
		pMultiBody->getLink(i).m_collider=col;		
	}
}

CommonExampleInterface* TestWeiMBCreateFunc(CommonExampleOptions& options)
{
	return new TestWeiMultiBody(options.m_guiHelper, options.m_option);
}


B3_STANDALONE_EXAMPLE(TestWeiMBCreateFunc)
