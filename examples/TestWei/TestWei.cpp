#include "TestWei.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "BulletDynamics/ConstraintSolver/btCustomSISolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"



namespace
{

	const char* getSolverTypeName(TestWei::DemoSolverType type)
	{

		const char* name = "";
		switch (type)
		{
		case TestWei::BT_SequentialSolver:
			name = "Sequential solver";
			break;
		case TestWei::WeiSolver:
			name = "Wei solver";
			break;
		case TestWei::BT_MLCP:
			name = "MLCP solver";
			break;
		default:
			break;
		}

		return name;
	}

	TestWei::DemoSolverType gDemoSolverType = TestWei::WeiSolver;
	btScalar gSolverIterations = 10;
	void setSolverTypeCallback(int buttonId, bool buttonState, void* userPointer)
	{
		gDemoSolverType = static_cast<TestWei::DemoSolverType>(buttonId);
	}
	void setSolverIterationCountCallback(float val, void* userPtr)
	{
		if (btDiscreteDynamicsWorld* world = reinterpret_cast<btDiscreteDynamicsWorld*>(userPtr))
		{
			world->getSolverInfo().m_numIterations = btMax(1, int(gSolverIterations));
		}
	}
}


TestWei::TestWei(struct GUIHelperInterface* helper, int testCase)
	: CommonRigidBodyBase(helper),
	m_testCase(testCase),
    m_paused(false)
{
}


void TestWei::initPhysics()
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
	m_solver = new btCustomSISolver();


	switch (m_testCase)
	{
	case Wei_BasicExmaple:
		setupCase0();
		break;
	case Wei_SlopeDemo:
		setupSlopeDemo();
		break;
	case Wei_Constraint:
		setupDemoConstraints();
		break;
	case Wei_SoftContact:
		setupSoftContact();
		break;
	case Wei_StressTest:
		setupStressTest();
		break;
    case Wei_StressChainTest:
        setupStress_Chain();
        break;
	default:
		break;
	}
    //m_body->setLinearVelocity(btVector3(-3.5f, 0, 0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	createUI();
}

void TestWei::createUI()
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

void TestWei::renderScene()
{
	drawDebugText();
	CommonRigidBodyBase::renderScene();
}

void TestWei::drawDebugText()
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

void TestWei::createEmptyDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solverType = gDemoSolverType;
	switch (m_solverType)
	{
	case BT_SequentialSolver:
		m_solver = new btSequentialImpulseConstraintSolver;
		break;
	case WeiSolver:
		m_solver = new btCustomSISolver();
		break;
	case BT_MLCP:
		m_solver = new btMLCPSolver(new btDantzigSolver());
		break;
	default:
		break;
	}

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	//m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase, m_solver, m_collisionConfiguration);
	//m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.005f;

}

void TestWei::stepSimulation(float deltaTime)
{
	if (m_paused)
		return;
	step(deltaTime);
}

void TestWei::step(float deltaTime)
{
	CommonRigidBodyBase::stepSimulation(deltaTime);
}



bool TestWei::keyboardCallback(int key, int state)
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

void TestWei::setupCase0()
{
	const int ARRAY_SIZE_Y = 5;
	const int ARRAY_SIZE_X = 5;
	const int ARRAY_SIZE_Z = 5;
	// camera setup
	{
		float dist = 4.6f;
		float pitch = -75.0f;
		float yaw = 31.0f;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}

	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance
	btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));
	btBoxShape* colBig = createBoxShape(btVector3(1, .1, .5));

	//btCollisionShape* colShape = new btSphereShape(btScalar(0.1));
	m_collisionShapes.push_back(colShape);
	m_collisionShapes.push_back(colBig);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar	mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
	{
		colShape->calculateLocalInertia(mass, localInertia);
		colBig->calculateLocalInertia(mass, localInertia);
	}


	for (int k = 0; k < ARRAY_SIZE_Y; k++)
	{
		for (int i = 0; i < ARRAY_SIZE_X; i++)
		{
			for (int j = 0; j < ARRAY_SIZE_Z; j++)
			{
				startTransform.setOrigin(btVector3(
					btScalar(0.2*i),
					btScalar(0.55f + .2*k),
					btScalar(0.2*j)));
				//startTransform.setRotation(btQuaternion(btVector3(0, 0, 1), 0.2f));


				createRigidBody(mass, startTransform, colShape);
				//m_body->setActivationState(DISABLE_DEACTIVATION);
				//m_body->setRestitution(0.5f);
			}
		}
	}

	////btTransform trans1; trans1.setIdentity();
	////trans1.setOrigin(btVector3(0.7f, 0.7f, 0));
	////m_body = createRigidBody(mass, trans1, colShape); 
	//btTransform trans2; trans2.setIdentity();
	//trans2.setOrigin(btVector3(0, 0.15f, 0));
	//createRigidBody(mass, trans2, colBig); 

}

void TestWei::setupSlopeDemo()
{
	// camera setup
	{
		float dist = 10.6f;
		float pitch = 87.79f;
		float yaw = 31;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	{
		///create a few basic rigid bodies
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(25.)));


		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, 3, 0));
		groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI*0.06));
		//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

	{

		btTransform startTransform; startTransform.setIdentity();
		btBoxShape* colShape = createBoxShape(btVector3(.5f, .5f, .5f));
		startTransform.setOrigin(btVector3(0, 8.0f, 0));
		btRigidBody* bodyA = createRigidBody(1.0f, startTransform, colShape);
		bodyA->setFriction(0.8f);

		startTransform.setOrigin(btVector3(0, 8.0f, 1));
		btRigidBody* bodyB = createRigidBody(1.0f, startTransform, colShape);
		bodyB->setFriction(0.8f);
	}
}

void TestWei::setupDemoConstraints()
{
	const btScalar THETA = SIMD_PI / 4.f;
	const btScalar L_1 = (2 - tan(THETA));
	const btScalar L_2 = (1 / cos(THETA));
	const btScalar RATIO = L_2 / L_1;
	btRigidBody* bodyA = 0;
	btRigidBody* bodyB=0;
	// camera setup
	{
		float dist = 25.0f;
		float pitch = 13.79f;
		float yaw = 25;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	{
		btCollisionShape* cylA = new btCylinderShape(btVector3(0.2,0.26,0.2));
		btCollisionShape* cylB = new btCylinderShape(btVector3(L_2,0.025,L_2));
		btCompoundShape* cyl0 = new btCompoundShape();
		cyl0->addChildShape(btTransform::getIdentity(),cylA);
		cyl0->addChildShape(btTransform::getIdentity(),cylB);

		btScalar mass = 6.28;
		btVector3 localInertia;
		cyl0->calculateLocalInertia(mass,localInertia);
		btRigidBody::btRigidBodyConstructionInfo ci(mass,0,cyl0,localInertia);
		ci.m_startWorldTransform.setOrigin(btVector3(-10,2,-8));


		btQuaternion orn(btVector3(0,0,1),-THETA);
		ci.m_startWorldTransform.setRotation(orn);

		btRigidBody* body = new btRigidBody(ci);//1,0,cyl0,localInertia);
		body->setLinearFactor(btVector3(0,0,0));
		btHingeConstraint* hinge = new btHingeConstraint(*body,btVector3(0,0,0),btVector3(0,1,0),true);
		m_dynamicsWorld->addConstraint(hinge);
		bodyB= body;
		body->setAngularVelocity(btVector3(0,3,0));

		m_dynamicsWorld->addRigidBody(body);
	}

	{
		const btScalar CUBE_HALF_EXTENTS = 1.0f;
		btScalar mass = 5.0f;
		btCollisionShape* shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(btVector3(0, 3.0f, -5));

		btRigidBody* body0 = createRigidBody( mass,trans,shape);
		trans.setOrigin(btVector3(2*CUBE_HALF_EXTENTS,20,0));
		mass = 1.f;
		btVector3 pivotInA(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,0);
		btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body0, pivotInA);
		p2p->m_setting.m_damping = 0.2f;
		//p2p->setParam(BT_CONSTRAINT_CFM, 0.02f);
		p2p->setParam(BT_CONSTRAINT_ERP, 0.2f);
		m_dynamicsWorld->addConstraint(p2p);
		p2p ->setBreakingImpulseThreshold(100.2);
		p2p->setDbgDrawSize(btScalar(5.f));
	}
}

void TestWei::setupSoftContact()
{

	// camera setup
	{
		float dist = 10.7f;
		float pitch = -75.0f;
		float yaw = 31;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	{
		btSphereShape* sphere = new btSphereShape(1.0f);
		btTransform trans; trans.setIdentity();
		trans.setOrigin(btVector3(0, 8.0f, 0));
		btRigidBody* body = createRigidBody(1.0f, trans, sphere);
		body->setAngularVelocity(btVector3(2.0f, 0, 0));
		body->setRollingFriction(0.2f);
		body->setActivationState(DISABLE_DEACTIVATION);
		m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.05f;
		m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;
	}
}

void TestWei::setupStressTest()
{
	// camera setup
	{
		float dist = 10.7f;
		float pitch = -75.0f;
		float yaw = 31;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	{
		btBoxShape* box = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(btVector3(0, 0.5f, 0));
		btRigidBody* body = createRigidBody(1.0f, trans, box);
		trans.setOrigin(btVector3(0, 2.0f, 0));
		btBoxShape* smallBox = new btBoxShape(btVector3(0.45f, 0.45f, 0.45f));
		btRigidBody* heavyBody = createRigidBody(100.0f, trans, smallBox);
		body->setActivationState(DISABLE_DEACTIVATION);
	}
}

void TestWei::setupStress_Chain()
{
	// camera setup
	{
		float dist = 10.7f;
		float pitch = -75.0f;
		float yaw = 31;
		float targetPos[3] = {0, 0, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	{
        btSphereShape* sphere = new btSphereShape(0.5f);

        btTransform trans;trans.setIdentity();
        trans.setOrigin(btVector3(0, 7.0f, 0));
        btRigidBody* root = createRigidBody(0.0f, trans, sphere);

        btRigidBody* lastChain = root;
        btVector3 pivot(0, 0, 0);

        const int numSeg = 5;
        for(int i = 0 ; i < numSeg; ++i)
        {
            btScalar mass = i == 0 ? 1.0f : 100.0f;
            trans.setOrigin(trans.getOrigin() + btVector3(0, -1.0f, 0.0));
            btRigidBody* chain = createRigidBody(mass, trans, sphere);
            btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*chain, *lastChain, 
                    btVector3(0, 0.5f, 0), btVector3(0, -0.5f, 0));
            m_dynamicsWorld->addConstraint(p2p);
            lastChain = chain;
        }
	}
	// chain using revolve constraint
	{
		btScalar segHalfLen = 0.5f;
		btBoxShape* box = new btBoxShape(btVector3(0.5f, segHalfLen, 0.1f));

		btTransform trans; trans.setIdentity();
		trans.setOrigin(btVector3(0, 7.0f, 3));
		btRigidBody* root = createRigidBody(0.0f, trans, box);
		btRigidBody* lastChain = root;

		const int numSeg = 5;
		for (int i = 0; i < numSeg; ++i)
		{
			btScalar mass = 1.0f;
			trans.setOrigin(trans.getOrigin() + btVector3(0, -segHalfLen * 2, 0.0f));
			btRigidBody* chain = createRigidBody(mass, trans, box);

			btHingeConstraint* hinge = new btHingeConstraint(*lastChain, *chain,
				btVector3(0, -segHalfLen, 0), btVector3(0, segHalfLen, 0),
				btVector3(1.0f, 0, 0), btVector3(1.0f, 0, 0));
			m_dynamicsWorld->addConstraint(hinge);

			lastChain = chain;
		}
	}
}

CommonExampleInterface* TestWeiCreateFunc(CommonExampleOptions& options)
{
	return new TestWei(options.m_guiHelper, options.m_option);
}


B3_STANDALONE_EXAMPLE(TestWeiCreateFunc)
