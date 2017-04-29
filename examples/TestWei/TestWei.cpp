#include "TestWei.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletDynamics/ConstraintSolver/btCustomSISolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 1 
#define ARRAY_SIZE_Z 1 

class TestWei : public CommonRigidBodyBase
{
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
private:
	void step(float deltaTime);
private:
    btRigidBody* m_body = nullptr;
	bool m_paused = false;
};


TestWei::TestWei(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper)
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

	{
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
						btScalar(0.25f + .2*k),
						btScalar(0.2*j)));				 
					//startTransform.setRotation(btQuaternion(btVector3(0, 0, 1), 0.2f));


					m_body = createRigidBody(mass , startTransform, colShape);
					//m_body->setActivationState(DISABLE_DEACTIVATION);
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

    //m_body->setLinearVelocity(btVector3(-3.5f, 0, 0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TestWei::renderScene()
{
	CommonRigidBodyBase::renderScene();
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
	//m_solver = new btSequentialImpulseConstraintSolver;
	m_solver = new btCustomSISolver();
	//btDantzigSolver* mlcp = new btDantzigSolver();
	//m_solver = new btMLCPSolver(mlcp);

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.001f;

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
    btVector3 angVel= m_body->getAngularVelocity();
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

CommonExampleInterface* TestWeiCreateFunc(CommonExampleOptions& options)
{
	return new TestWei(options.m_guiHelper);
}


B3_STANDALONE_EXAMPLE(TestWeiCreateFunc)
