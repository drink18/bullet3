#include "btCustomSISolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

btCustomSISolver::btCustomSISolver()
{

}


btCustomSISolver::~btCustomSISolver()
{

}

void btCustomSISolver::solvePenetration(btSIConstraintInfo& c, btScalar dt)
{
	if (c.m_pentrationRhs != 0)
	{
		btScalar jV = 0;
		btVelocityAccumulator& accu1 = m_accumulatorPool[c.m_accumId1];
		btVelocityAccumulator& accu2 = m_accumulatorPool[c.m_accumId2];

		jV += accu1.m_pushLinVelocity.dot(c.m_Jl1) + accu1.m_pushAngVelcity.dot(c.m_Ja1);
		jV += accu2.m_pushLinVelocity.dot(c.m_Jl2) + accu2.m_pushAngVelcity.dot(c.m_Ja2);
		btScalar lambda =  (-jV + c.m_pentrationRhs) / c.m_effM;

		btScalar  accuLambda = c.m_appliedPeneImpulse + lambda;
		accuLambda = accuLambda < 0 ? 0 : accuLambda;

		btScalar impulse = accuLambda - c.m_appliedPeneImpulse;
		c.m_appliedPeneImpulse = accuLambda;

		accu1.m_pushLinVelocity += c.m_Jl1 * impulse * c.m_invM1;
		accu1.m_pushAngVelcity += c.m_Ja1 * impulse * c.m_invI1;
		accu2.m_pushLinVelocity += c.m_Jl2 * impulse * c.m_invM2;
		accu2.m_pushAngVelcity += c.m_Ja2 * impulse * c.m_invI2;
	}
}

void btCustomSISolver::solve(btSIConstraintInfo& c, const btContactSolverInfo& info)
{
	btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
	btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];

	btScalar jV = accum1.m_linearVelocity.dot(c.m_Jl1) + accum1.m_linearVelocity.dot(c.m_Ja1);
	jV += accum2.m_linearVelocity.dot(c.m_Jl2) + accum2.m_angularVelocity.dot(c.m_Ja2);
	btScalar lambda = (-jV + c.m_rhs) / c.m_effM;

	btScalar  accuLambda = c.m_appliedImpulse + lambda;
	accuLambda = accuLambda < 0 ? 0 : accuLambda;

	btScalar impulse = accuLambda - c.m_appliedImpulse;
	c.m_appliedImpulse = accuLambda;

	accum1.m_linearVelocity = accum1.m_linearVelocity + c.m_Jl1 * impulse * c.m_invM1;
	accum2.m_linearVelocity = accum2.m_linearVelocity + c.m_Jl2 * impulse * c.m_invM2;
	accum1.m_angularVelocity = accum1.m_angularVelocity + c.m_Ja1 * impulse * c.m_invI1;
	accum2.m_angularVelocity = accum2.m_angularVelocity + c.m_Ja2 * impulse * c.m_invI2;
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

void btCustomSISolver::initAccumulator(btVelocityAccumulator& accum, btCollisionObject* body, const btContactSolverInfo& info)
{
	
	btRigidBody* rigidBody = btRigidBody::upcast(body);
	if (rigidBody)
	{
		accum.m_externalForce = rigidBody->getTotalForce() * rigidBody->getInvMass() * info.m_timeStep;
		accum.m_linearVelocity = rigidBody->getLinearVelocity() + accum.m_externalForce;
		accum.m_angularVelocity = rigidBody->getAngularVelocity();
		accum.m_originalBody = rigidBody;
	}
	else
	{
	}
}

int btCustomSISolver::getOrAllocateAccumulator(btCollisionObject* btBody, const btContactSolverInfo& info)
{
	int accumId = btBody->getCompanionId();

	if (accumId < 0)
	{
		accumId = m_accumulatorPool.size();
		btBody->setCompanionId(accumId);
		m_accumulatorPool.expand();
		initAccumulator(m_accumulatorPool[accumId], btBody, info);
	}

	return accumId;
}

void btCustomSISolver::initAllAccumulators(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& info)
{
	m_accumulatorPool.reserve(numBodies);
	for (int i = 0; i < numBodies; ++i)
	{
		int id = getOrAllocateAccumulator(bodies[i], info);
	}
}

void btCustomSISolver::setupAllContactConstraints( btPersistentManifold& manifold, const btContactSolverInfo& info)
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

		btSIConstraintInfo& c = m_tmpConstraintPool.expand();
		
		c.m_accumId1 = getOrAllocateAccumulator(bodyA, info);
		c.m_accumId2 = getOrAllocateAccumulator(bodyB, info);

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
		c.m_origManifoldPoint = &pt;

		btScalar penetration = pt.getDistance() + info.m_linearSlop;;
		if (penetration < 0)
		{
			const float beta = 0.2f;
			c.m_pentrationRhs = -beta* penetration / info.m_timeStep ;
			//c.m_rhs = -beta* pt.getDistance() / info.m_timeStep;
		}
		else
		{
			c.m_pentrationRhs = 0.0f;
			//c.m_rhs = 0.0f;
		}

		// warm starting
		c.m_appliedImpulse = pt.m_appliedImpulse;
	}
}

void btCustomSISolver::solveAllContacts(const btContactSolverInfo& info, int numIter)
{
	for (int iter = 0; iter < numIter; ++iter)
	{
		for (int i = 0; i < m_tmpConstraintPool.size(); ++i)
		{
			btSIConstraintInfo& c = m_tmpConstraintPool[i];
			solve(c, info);
		}
	}

	for (int i = 0; i < m_accumulatorPool.size(); ++i)
	{
		btVelocityAccumulator& accum = m_accumulatorPool[i];

		accum.m_originalBody->setLinearVelocity(accum.m_linearVelocity);
		accum.m_originalBody->setAngularVelocity(accum.m_angularVelocity);
	}

	for (int i = 0; i < m_tmpConstraintPool.size(); ++i)
	{
		btSIConstraintInfo& c = m_tmpConstraintPool[i];
		c.m_origManifoldPoint->m_appliedImpulse = c.m_appliedImpulse;
	}

}

void btCustomSISolver::solveAllPenetrations(const btContactSolverInfo& info, int numIter)
{
	const btScalar dt = info.m_timeStep;
	for (int i = 0; i < m_tmpConstraintPool.size(); ++i)
	{
		btSIConstraintInfo& c = m_tmpConstraintPool[i];
		solvePenetration(c, dt);
	}

	for (int i = 0; i < m_accumulatorPool.size(); ++i)
	{
		btVelocityAccumulator& accu = m_accumulatorPool[i];
		btTransform trans;
		btTransformUtil::integrateTransform(accu.m_originalBody->getWorldTransform(), accu.m_pushLinVelocity, 
				accu.m_angularVelocity, dt, trans);
		accu.m_originalBody->setWorldTransform(trans);
	
	}
}

btScalar btCustomSISolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
	, int numManifolds, btTypedConstraint** constraints, int numConstraints
	, const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
	, btDispatcher* dispatcher)
{

	const btScalar dt = info.m_timeStep;
	m_tmpConstraintPool.clear();
	m_accumulatorPool.clear();

	initAllAccumulators(bodies, numBodies, info);

	for (int i = 0; i < numManifolds; ++i)
	{
		setupAllContactConstraints(*manifold[i], info);
	}

	const int numIter = info.m_numIterations;

	// position error correction
	solveAllPenetrations(info, numIter);

	// solver contact constraint
	solveAllContacts(info, numIter);

	return 0.0f;
}

void btCustomSISolver::reset()
{
}
