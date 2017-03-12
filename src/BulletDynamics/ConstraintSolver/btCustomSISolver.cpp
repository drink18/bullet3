#include "btCustomSISolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

btCustomSISolver::btCustomSISolver()
{

}


btCustomSISolver::~btCustomSISolver()
{

}


namespace
{

    void solve(btVector3& n, btVector3& rXn, btScalar invM, btVector3& invI, btRigidBody* body)
    {
            // compute effective mass
            btScalar effM = n.x() * n.x() * invM + n.y() * n.y() * invM + n.z() * n.z() * invM +
                rXn.x() * rXn.x() * invI.x() + rXn.y() * rXn.y() * invI.y() + rXn.z() * rXn.z() * invI.z();

            btVector3 Jl = n; // linear part of J
            btVector3 Ja = rXn; // angular part of J
            btVector3 linVel = body->getLinearVelocity();
            btVector3 angVel = body->getAngularVelocity();
            btScalar lambda = -(linVel.dot(Jl) + angVel.dot(Ja)) / effM;
            linVel += Jl * lambda * invM;
            angVel += Ja * lambda * invM;

            body->setLinearVelocity(linVel);
            body->setAngularVelocity(angVel);
 
    }
}

void btCustomSISolver::SolveContact(btPersistentManifold& manifold)
{
    for (int i = 0; i < manifold.getNumContacts(); ++i)
    {
        btManifoldPoint& pt = manifold.getContactPoint(i);
        
        btRigidBody* bodyA = (btRigidBody*)btRigidBody::upcast(manifold.getBody0());
        btRigidBody* bodyB = (btRigidBody*)btRigidBody::upcast(manifold.getBody1());


        btVector3 rA = pt.getPositionWorldOnA() - bodyA->getWorldTransform().getOrigin();
        btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();

        btVector3 nA = pt.m_normalWorldOnB;
        btVector3 rXnA = rA.cross(nA);
        btScalar invMA = bodyA->getInvMass();
        btMatrix3x3 invIMA = bodyA->getInvInertiaTensorWorld();
        btVector3 invIA(invIMA[0][0], invIMA[1][1], invIMA[2][2]);

        btVector3 nB = -pt.m_normalWorldOnB;
        btVector3 rXnB = rB.cross(nB);
        btScalar invMB = bodyB->getInvMass();
        btMatrix3x3 invIMB = bodyB->getInvInertiaTensorWorld();
        btVector3 invIB(invIMB[0][0], invIMB[1][1], invIMB[2][2]);

        solve(nA, rXnA, invMA, invIA, bodyA);
        solve(nB, rXnB, invMB, invIB, bodyB);

    }

}


btScalar btCustomSISolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
    , int numManifolds, btTypedConstraint** constraints, int numConstraints
    , const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
    , btDispatcher* dispatcher)
{
    for (int i = 0; i < numManifolds; ++i)
    {
        SolveContact(*manifold[i]);
    }
    return 0.0f;
}

void btCustomSISolver::reset()
{

}