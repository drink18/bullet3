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
}


TestWeiMultiBody::TestWeiMultiBody(struct GUIHelperInterface* helper, int testCase)
	: CommonRigidBodyBase(helper),
	m_testCase(testCase),
    m_paused(false)
{
}


void TestWeiMultiBody::initPhysics()
{
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
	m_solver = new btWeiSISolver();


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
	for(int i  = 0 ; i < SovlerType_count; ++i)
    {
		//char* buttonName = "Sequential impulse solver";
		char buttonName[256];
		DemoSolverType solverType = static_cast<DemoSolverType>(i);
		sprintf(buttonName, "Solver: %s", getSolverTypeName(solverType));
        ButtonParams button( buttonName, 0, false );
		button.m_buttonId = solverType;
        button.m_callback = setSolverTypeCallback;
        m_guiHelper->getParameterInterface()->registerButtonParameter( button );
    }
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
}

void TestWeiMultiBody::renderScene()
{
	drawDebugText();
	CommonRigidBodyBase::renderScene();
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

#if 0
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solverType = gDemoSolverType;
	switch (m_solverType)
	{
	case BT_SequentialSolver:
		m_solver = new btSequentialImpulseConstraintSolver;
		break;
	case WeiSolver:
		m_solver = new btWeiSISolver();
		break;
	case BT_MLCP:
		m_solver = new btMLCPSolver(new btDantzigSolver());
		break;
	default:
		break;
	}
#endif

	btMultiBodyConstraintSolver* sol = new btMultiBodyConstraintSolver;

	m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase, sol, m_collisionConfiguration);

}

void TestWeiMultiBody::stepSimulation(float deltaTime)
{
	if (m_paused)
		return;
	step(deltaTime);
}

void TestWeiMultiBody::step(float deltaTime)
{
	CommonRigidBodyBase::stepSimulation(deltaTime);
}



bool TestWeiMultiBody::keyboardCallback(int key, int state)
{
	if (key == B3G_F5)
	{
		m_paused = !m_paused;
		return false;
	}

	if (key == B3G_F6)
	{
		m_paused = true;
		step(1.0f / 60);
		return false;
	}

	return CommonRigidBodyBase::keyboardCallback(key, state);
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
		const btScalar pillarH = 6.0f;
		btBoxShape* bigBox = new btBoxShape(btVector3(1, pillarH / 2, 1));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(btVector3(-4, pillarH / 2, 0));
		btRigidBody* pillarA = createRigidBody(0, trans, bigBox);
		trans.setOrigin(btVector3(4, pillarH / 2, 0));
		btRigidBody* pillarB = createRigidBody(0, trans, bigBox);
		btRigidBody* lastChain = pillarA;
		btRigidBody* firstChain = nullptr;

		const btScalar chainHalfLen = 0.5f;
		const btScalar chainMass = 1.0f;
		btBoxShape* chainShape = new btBoxShape(btVector3(chainHalfLen, 0.05f, 0.35f));
		const int numChain = 6;
		btTransform chainTrans; chainTrans.setIdentity();
		chainTrans.setOrigin(btVector3(-2.0f, pillarH, 0));
		for (int i = 0; i < numChain; ++i)
		{
			btRigidBody* chain = createRigidBody(chainMass, chainTrans, chainShape);
			chainTrans.setOrigin(chainTrans.getOrigin() + btVector3(chainHalfLen * 2, 0, 0));
			if (i == 0)
			{
				firstChain = chain;
			}
			else
			{
				btHingeConstraint* hinge = new btHingeConstraint(*lastChain, *chain,
					btVector3(chainHalfLen, 0, 0), btVector3(-chainHalfLen, 0, 0),
					btVector3(0, 0, 1), btVector3(0, 0, 1));
				m_dynamicsWorld->addConstraint(hinge);
			}
			lastChain = chain;
		}

		btHingeConstraint* hingeA = new btHingeConstraint(*pillarA, *firstChain,
			btVector3(1.1f, pillarH / 2, 0), btVector3(-chainHalfLen, 0, 0),
			btVector3(0, 0, 1), btVector3(0, 0, 1));
		m_dynamicsWorld->addConstraint(hingeA);
		btHingeConstraint* hingeB = new btHingeConstraint(*pillarB, *lastChain,
			btVector3(-1.1f, pillarH / 2, 0), btVector3(chainHalfLen, 0, 0),
			btVector3(0, 0, 1), btVector3(0, 0, 1));
		m_dynamicsWorld->addConstraint(hingeB);
	}
	{
		// drop a heavy box
		btBoxShape* box = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(btVector3(0, 10.0f, 0));
		const btScalar mass = 50.0f;
		createRigidBody(mass, trans, box);
	}
}

CommonExampleInterface* TestWeiCreateFunc(CommonExampleOptions& options)
{
	return new TestWeiMultiBody(options.m_guiHelper, options.m_option);
}


B3_STANDALONE_EXAMPLE(TestWeiCreateFunc)
