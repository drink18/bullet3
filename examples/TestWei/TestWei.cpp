#include "TestWei.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

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
