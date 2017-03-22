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
	btScalar jV1 = c.m_body1->getLinearVelocity().dot(c.m_Jl1) + c.m_body1->getAngularVelocity().dot(c.m_Ja1);
	jV1 += c.m_body2->getLinearVelocity().dot(c.m_Jl2) + c.m_body2->getAngularVelocity().dot(c.m_Ja2);
	btScalar lambda = -jV1 / c.m_effM;

	btScalar  accuLambda1 = c.m_appliedImpulse1 + lambda;
	btScalar  accuLambda2 = c.m_appliedImpulse2 + lambda;
	accuLambda1 = accuLambda1 < 0 ? 0 : accuLambda1;
	accuLambda2 = accuLambda2 < 0 ? 0 : accuLambda2;

	btScalar l1 = accuLambda1 - c.m_appliedImpulse1;
	btScalar l2 = accuLambda2 - c.m_appliedImpulse2;
	c.m_appliedImpulse1 = accuLambda1;
	c.m_appliedImpulse2 = accuLambda2;

	c.m_body1->setLinearVelocity(c.m_body1->getLinearVelocity() + c.m_Jl1 * l1 * c.m_invM1);
	c.m_body2->setLinearVelocity(c.m_body2->getLinearVelocity() + c.m_Jl2 * l2 * c.m_invM2);
	c.m_body1->setAngularVelocity(c.m_body1->getAngularVelocity() + c.m_Ja1 * l1 * c.m_invI1);
	c.m_body2->setAngularVelocity(c.m_body2->getAngularVelocity() + c.m_Ja2 * l2 * c.m_invI2);
}

namespace {
	btScalar _computeBodyEffMass(const btVector3& invI, const btScalar invM, const btVector3& rXn)
	{
		btScalar effM1 = invM + rXn.x() * rXn.x() * invI.x()
			+ rXn.y() * rXn.y() * invI.y()
			+ rXn.z() * rXn.z() * invI.z()
			+ rXn.z() * rXn.z() * invI.z();
		return effM1;
	}
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
        btScalar invMB = bodyB->getInvMass();
		if (invMA == 0 && invMB == 0)
			continue;
		
		btSolverConstraint& c = m_tmpConstraintPool.expand();
		c.m_body1 = bodyA;
		c.m_body2 = bodyB;

		btVector3 rA = pt.getPositionWorldOnA() - bodyA->getWorldTransform().getOrigin();
		btVector3 nA = pt.m_normalWorldOnB;
		btVector3 rXnA = rA.cross(nA);
		btMatrix3x3 invIMA = bodyA->getInvInertiaTensorWorld();
		btVector3 invIA(invIMA[0][0], invIMA[1][1], invIMA[2][2]);
		btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();
		btVector3 nB = -pt.m_normalWorldOnB;
		btVector3 rXnB = rB.cross(nB);
		btMatrix3x3 invIMB = bodyB->getInvInertiaTensorWorld();
		btVector3 invIB(invIMB[0][0], invIMB[1][1], invIMB[2][2]);


		btScalar effM1 = _computeBodyEffMass(invIA, bodyA->getInvMass(), rXnA);
		btScalar effM2 = _computeBodyEffMass(invIB, bodyB->getInvMass(), rXnB);
		c.m_effM = effM1 + effM2;

		c.m_Jl1 = nA;
		c.m_Ja1 = rXnA;
		c.m_Jl2 = nB;
		c.m_Ja2 = rXnB;
		c.m_invM1 = invMA;
		c.m_invM2 = invMB;
		c.m_invI1 = invIA;
		c.m_invI2 = invIB;
    }
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
