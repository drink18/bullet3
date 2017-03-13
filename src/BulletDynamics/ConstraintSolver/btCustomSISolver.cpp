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
	void solve(btVector3& n, btVector3& rXn, btScalar invM, btVector3& invI, btRigidBody* body, btVector3& extImpl)
	{
		// compute effective mass
		btScalar effM = n.x() * n.x() * invM + n.y() * n.y() * invM + n.z() * n.z() * invM +
			rXn.x() * rXn.x() * invI.x() + rXn.y() * rXn.y() * invI.y() + rXn.z() * rXn.z() * invI.z();

		btVector3 Jl = n; // linear part of J
		btVector3 Ja = rXn; // angular part of J
		btVector3 linVel = body->getLinearVelocity() + extImpl;
		btVector3 angVel = body->getAngularVelocity();
		btScalar lambda = -(linVel.dot(Jl) + angVel.dot(Ja)) / effM;
		linVel += Jl * lambda * invM;
		angVel += Ja * lambda * invI;

		body->setLinearVelocity(linVel);
		body->setAngularVelocity(angVel);
	}

	void SolveContact(btPersistentManifold& manifold, const btContactSolverInfo& info)
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
				btVector3 extImp = bodyA->getTotalForce() * invMA * dt;
				btVector3 rA = pt.getPositionWorldOnA() - bodyA->getWorldTransform().getOrigin();
				btVector3 nA = pt.m_normalWorldOnB;
				btVector3 rXnA = rA.cross(nA);
				btMatrix3x3 invIMA = bodyA->getInvInertiaTensorWorld();
				btVector3 invIA(invIMA[0][0], invIMA[1][1], invIMA[2][2]);
				solve(nA, rXnA, invMA, invIA, bodyA, extImp);
			}

			btScalar invMB = bodyB->getInvMass();
			if (invMB != 0)
			{
				btVector3 extImp = bodyB->getTotalForce() * invMB * dt;
				btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();
				btVector3 nB = - pt.m_normalWorldOnB;
				btVector3 rXnB = rB.cross(nB);
				btMatrix3x3 invIMB = bodyB->getInvInertiaTensorWorld();
				btVector3 invIB(invIMB[0][0], invIMB[1][1], invIMB[2][2]);
				solve(nB, rXnB, invMB, invIB, bodyB, extImp);

				btVector3 cptVel = bodyB->getLinearVelocity();
				btVector3 angVel = bodyB->getAngularVelocity().cross(rB);
				cptVel += angVel;
				printf("%.3f\n", cptVel.x());
			}
		}

	}
}


btScalar btCustomSISolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
	, int numManifolds, btTypedConstraint** constraints, int numConstraints
	, const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
	, btDispatcher* dispatcher)
{
	// apply external impulse
	const btScalar dt = info.m_timeStep;
	for (int i = 0; i < numBodies; ++i)
	{
		btRigidBody* bodyA = (btRigidBody*)btRigidBody::upcast(bodies[i]);
		if (bodyA->getInvMass() != 0)
		{
			btVector3 extImp = bodyA->getTotalForce() * bodyA->getInvMass() * dt;
			bodyA->setLinearVelocity(extImp + bodyA->getLinearVelocity());
		}

	}

	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < numManifolds; ++i)
		{
			SolveContact(*manifold[i], info);
		}
	}

	
	return 0.0f;
}

void btCustomSISolver::reset()
{

}