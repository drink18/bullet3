#include "btCustomSISolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

btCustomSISolver::btCustomSISolver()
{

}


btCustomSISolver::~btCustomSISolver()
{

}

void btCustomSISolver::solve(btSolverConstraint& c, btScalar dt)
{
    btVector3 linVel = c.m_body->getLinearVelocity() ;
    btVector3 angVel = c.m_body->getAngularVelocity();
    btScalar lambda = -(linVel.dot(c.m_Jl) + angVel.dot(c.m_Ja)) / c.m_effM;

    btScalar  accuLambda = c.m_appliedImpulse + lambda;
    accuLambda = accuLambda < 0 ? 0 : accuLambda;

    btScalar l = accuLambda - c.m_appliedImpulse;
    c.m_appliedImpulse = accuLambda;

    linVel += c.m_Jl * l * c.m_invM;
    angVel += c.m_Ja * l* c.m_invI;

    c.m_body->setLinearVelocity(linVel);
    c.m_body->setAngularVelocity(angVel);
}

void btCustomSISolver::setupAllContactConstratins(btPersistentManifold& manifold, const btContactSolverInfo& info)
{
    const btScalar dt = info.m_timeStep;
    for (int i = 0; i < manifold.getNumContacts(); ++i)
    {
        btManifoldPoint& pt = manifold.getContactPoint(i);

        btRigidBody* bodyA = (btRigidBody*)btRigidBody::upcast(manifold.getBody0());
        btRigidBody* bodyB = (btRigidBody*)btRigidBody::upcast(manifold.getBody1());

        btScalar invMA = bodyA->getInvMass();
        if (invMA != 0)
        {
            btVector3 extImp = bodyA->getTotalForce() * invMA * dt ;
            btVector3 rA = pt.getPositionWorldOnA() - bodyA->getWorldTransform().getOrigin();
            btVector3 nA = pt.m_normalWorldOnB;
            btVector3 rXnA = rA.cross(nA);
            btMatrix3x3 invIMA = bodyA->getInvInertiaTensorWorld();
            btVector3 invIA(invIMA[0][0], invIMA[1][1], invIMA[2][2]);

            setupContactConstraint(bodyA, nA, rXnA, invIA);
        }

        btScalar invMB = bodyB->getInvMass();
        if (invMB != 0)
        {
            btVector3 extImp = bodyB->getTotalForce() * invMB * dt / info.m_numIterations;
            btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();
            btVector3 nB = - pt.m_normalWorldOnB;
            btVector3 rXnB = rB.cross(nB);
            btMatrix3x3 invIMB = bodyB->getInvInertiaTensorWorld();
            btVector3 invIB(invIMB[0][0], invIMB[1][1], invIMB[2][2]);
            setupContactConstraint(bodyB, nB, rXnB, invIB);
        }
    }
}

void btCustomSISolver::setupContactConstraint(btRigidBody* body, btVector3& n, btVector3& rXn, btVector3& invI)
{
    btSolverConstraint& c = m_tmpConstraintPool.expand();

    c.m_Jl = n;
    c.m_Ja = rXn;
    c.m_effM = body->getInvMass() + rXn.x() * rXn.x() * invI.x()
               + rXn.y() * rXn.y() * invI.y()
               + rXn.z() * rXn.z() * invI.z();
    c.m_body = body;
    c.m_invI = invI;
    c.m_invM = body->getInvMass();
}

void btCustomSISolver::solveAllContacts(btScalar dt)
{
    for(int i = 0; i < m_tmpConstraintPool.size(); ++i)
    {
        btSolverConstraint& c = m_tmpConstraintPool[i];
        solve(c, dt);
    }
}

btScalar btCustomSISolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
                                      , int numManifolds, btTypedConstraint** constraints, int numConstraints
                                      , const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
                                      , btDispatcher* dispatcher)
{

    m_tmpConstraintPool.clear();

    for (int i = 0; i < numManifolds; ++i)
    {
        setupAllContactConstratins(*manifold[i], info);
    }

    const int numIter =  info.m_numIterations;
    for (int j = 0; j <  numIter; j++)
    {
        // apply external impulse
        const btScalar dt = info.m_timeStep;
        for (int i = 0; i < numBodies; ++i)
        {
            btRigidBody* bodyA = (btRigidBody*)btRigidBody::upcast(bodies[i]);
            if (bodyA->getInvMass() != 0)
            {
                btVector3 extImp = bodyA->getTotalForce() * bodyA->getInvMass() * dt / numIter; 
                bodyA->setLinearVelocity(extImp + bodyA->getLinearVelocity());
            }
        }

        solveAllContacts(dt );
    }

    return 0.0f;
}

void btCustomSISolver::reset()
{

}
