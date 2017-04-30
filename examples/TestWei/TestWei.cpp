#include "TestWei.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "BulletDynamics/ConstraintSolver/btCustomSISolver.h"
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
		default:
			break;
		}

		return name;
	}

	TestWei::DemoSolverType gDemoSolverType = TestWei::WeiSolver;
	void setSolverTypeCallback(int buttonId, bool buttonState, void* userPointer)
	{
		gDemoSolverType = static_cast<TestWei::DemoSolverType>(buttonId);
	}
}


TestWei::TestWei(struct GUIHelperInterface* helper, int testCase)
	: CommonRigidBodyBase(helper),
	m_testCase(testCase)
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
	case 0:
		setupCase0();
		break;
	case 1:
		setupCase1();
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
	for(int i  = 0 ; i < DemoSolverType::SovlerType_count; ++i)
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
	default:
		break;
	}

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.005f;

}

void TestWei::stepSimulation(float deltaTime)
{
	if (m_paused)
		return;
    {
        //float dist = 2;
        //float pitch = 0;
        //float yaw = 35;
        //btVector3 targetPos = m_body->getCenterOfMassPosition();
        //m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
    }
    //m_body->applyCentralImpulse(btVector3(-0.05f, 0, 0));
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


				m_body = createRigidBody(mass, startTransform, colShape);
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

void TestWei::setupCase1()
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
		groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI*0.03));
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
		body->setFriction(.5);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

	if(false)
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(100.),btScalar(100.),btScalar(50.)));
	
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,0,-54));
		//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction(.1);
		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	{

		btTransform startTransform; startTransform.setIdentity();
		btBoxShape* colShape = createBoxShape(btVector3(.5f, .5f, .5f));
		startTransform.setOrigin(btVector3(0, 8.0f, 0));
		createRigidBody(1.0f, startTransform, colShape);

		startTransform.setOrigin(btVector3(0, 8.0f, 1));
		createRigidBody(1.0f, startTransform, colShape);
	}
}

CommonExampleInterface* TestWeiCreateFunc(CommonExampleOptions& options)
{
	return new TestWei(options.m_guiHelper, options.m_option);
}


B3_STANDALONE_EXAMPLE(TestWeiCreateFunc)
