#include "TestWei.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletDynamics/ConstraintSolver/btCustomSISolver.h"

#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 1 
#define ARRAY_SIZE_Z 1 
struct TestWei : public CommonRigidBodyBase
{
    TestWei(struct GUIHelperInterface* helper)
        : CommonRigidBodyBase(helper)
    {
    }

    virtual ~TestWei(){}
    virtual void initPhysics() override;
    virtual void renderScene() override;

    void resetCamera()
    {
        float dist = 4;
        float pitch = 52;
        float yaw = 35;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
    }
};


void TestWei::initPhysics()
{
    m_guiHelper->setUpAxis(1);
    createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    if(m_dynamicsWorld->getDebugDrawer())
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

    btBoxShape* groundShape = createBoxShape(btVector3(50, 50, 50));

    m_collisionShapes.push_back(groundShape);
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -50, 0));

    createRigidBody(0, groundTransform, groundShape, btVector4(0, 0, 1, 1));
    m_solver = new btCustomSISolver();

#if 0
    {
        // create dynamic bodies 
        btBoxShape* colShape = createBoxShape(btVector3(0.1f, 0.1f, 0.1f));
        m_collisionShapes.push_back(colShape);

        // create dynamic body
        //
        btTransform startTrans;
        startTrans.setOrigin(btVector3(0, 2, 0));
        

        btScalar mass(1.0f);

        btVector3 localInertia(0, 0, 0);
        colShape->calculateLocalInertia(mass, localInertia);

        createRigidBody(mass, startTrans, colShape);
    }
#endif
{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1,.1,.1));
		

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										btScalar(0.2*i),
										btScalar(.2*k),
										btScalar(0.2*j)));

			
					createRigidBody(mass,startTransform,colShape);
					

				}
			}
		}
	}

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TestWei::renderScene()
{
    CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* TestWeiCreateFunc(CommonExampleOptions& options)
{
    return new TestWei(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(TestWeiCreateFunc)
