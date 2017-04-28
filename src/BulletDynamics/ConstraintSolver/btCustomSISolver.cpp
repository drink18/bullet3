#include "btCustomSISolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "LinearMath/btIDebugDraw.h"

btCustomSISolver::btCustomSISolver()
    :m_debugDrawer(nullptr)
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
		btScalar lambda =  (-jV + c.m_pentrationRhs) * c.m_invEffM;

		btScalar  accuLambda = c.m_appliedPeneImpulse + lambda;
		if (accuLambda < 0)
		{
			accuLambda = 0;
		}
		accuLambda = accuLambda < 0 ? 0 : accuLambda;

		btScalar impulse = accuLambda - c.m_appliedPeneImpulse;
		c.m_appliedPeneImpulse = accuLambda;

		accu1.m_pushLinVelocity += c.m_Jl1 * impulse * c.m_invM1;
		accu1.m_pushAngVelcity += c.m_Ja1 * impulse * c.m_invI1;
		accu2.m_pushLinVelocity += c.m_Jl2 * impulse * c.m_invM2;
		accu2.m_pushAngVelcity += c.m_Ja2 * impulse * c.m_invI2;
	}
}

void btCustomSISolver::solve(btSIConstraintInfo& c)
{
	btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
	btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];

	btScalar jV = 0;
	jV = accum1.m_deltaLinearVelocity.dot(c.m_Jl1) + accum1.m_deltaAngularVelocity.dot(c.m_Ja1);
	jV += accum2.m_deltaLinearVelocity.dot(c.m_Jl2) + accum2.m_deltaAngularVelocity.dot(c.m_Ja2);
	btScalar lambda = (-jV + c.m_rhs) * c.m_invEffM;

	btScalar  accuLambda = c.m_appliedImpulse + lambda;
	if (accuLambda > c.m_upperLimit)
	{
		accuLambda = c.m_upperLimit;
	}
	else if(accuLambda < c.m_lowerLimit)
	{
		accuLambda = c.m_lowerLimit;
	}
	btScalar impulse = accuLambda - c.m_appliedImpulse;
	c.m_appliedImpulse = accuLambda;

	accum1.m_deltaLinearVelocity += c.m_Jl1 * impulse * c.m_invM1;
	accum2.m_deltaLinearVelocity += c.m_Jl2 * impulse * c.m_invM2;
	accum1.m_deltaAngularVelocity += c.m_Ja1 * impulse * c.m_invI1;
	accum2.m_deltaAngularVelocity += c.m_Ja2 * impulse * c.m_invI2;
}

namespace {
	btScalar _computeBodyEffMass(const btVector3& invI, const btScalar invM, const btVector3& rXn)
	{
		btScalar effM = invM + rXn.x() * rXn.x() * invI.x()
			+ rXn.y() * rXn.y() * invI.y()
			+ rXn.z() * rXn.z() * invI.z();
		return effM;
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
		accum.m_deltaLinearVelocity.setValue(0, 0, 0);
		accum.m_deltaAngularVelocity.setValue(0, 0, 0);
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
	const btScalar  invDt = 1.0f / info.m_timeStep;
	for (int i = 0; i < manifold.getNumContacts(); ++i)
	{
		btManifoldPoint& pt = manifold.getContactPoint(i);

        //btIDebugDraw::DefaultColors defaultColors = m_debugDrawer->getDefaultColors();
        //m_debugDrawer->drawContactPoint(pt.getPositionWorldOnB(), pt.m_normalWorldOnB, pt.getDistance(), pt.getLifeTime(), defaultColors.m_contactPoint);

		btRigidBody* bodyA = (btRigidBody*)btRigidBody::upcast(manifold.getBody0());
		btRigidBody* bodyB = (btRigidBody*)btRigidBody::upcast(manifold.getBody1());

		btScalar invMA = bodyA->getInvMass();
		btScalar invMB = bodyB->getInvMass();
		if (invMA == 0 && invMB == 0)
			continue;

		btSIConstraintInfo& c = m_tmpContactConstraintPool.expand();
		
		c.m_accumId1 = getOrAllocateAccumulator(bodyA, info);
		c.m_accumId2 = getOrAllocateAccumulator(bodyB, info);
		btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
		btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];

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

		btScalar relaxation = info.m_sor;
		btScalar cfm = info.m_globalCfm * invDt;
		c.m_invEffM = relaxation / (effM1 + effM2 + cfm);

		c.m_Jl1 = nA;
		c.m_Ja1 = rXnA;
		c.m_Jl2 = nB;
		c.m_Ja2 = rXnB;
		c.m_invM1 = invMA;
		c.m_invM2 = invMB;
		c.m_invI1 = invIA;
		c.m_invI2 = invIB;
		c.m_origManifoldPoint = &pt;
		c.m_upperLimit = 1e10f;
		c.m_lowerLimit = 0;
		c.m_linearFactor1 = bodyA->getLinearFactor();
		c.m_linearFactor2 = bodyB->getLinearFactor();
		c.m_angularFactor1 = bodyA->getAngularFactor();
		c.m_angularFactor2 = bodyB->getAngularFactor();

		btScalar relVel = accum1.getVelocityAtContact(nA, rXnA) + accum2.getVelocityAtContact(nB, rXnB);

		btScalar penetration = pt.getDistance() + info.m_linearSlop;// +0.001f;
		btScalar positionError = 0;
		const btScalar restituion = pt.m_combinedRestitution;
		btScalar velocityError = restituion - relVel;
		if (penetration > 0)
		{
			positionError = 0.0f;
			velocityError -= penetration * invDt;
		}
		else
		{
			positionError = -penetration * info.m_erp * invDt;
		}


		c.m_rhs = velocityError;
		c.m_pentrationRhs = positionError;

		//apply warm starting impulse
		if (info.m_solverMode & SOLVER_USE_WARMSTARTING)
		{
			btScalar warmStartingImp = pt.m_appliedImpulse * info.m_warmstartingFactor;
			accum1.applyWarmStartImpulse(warmStartingImp, c.m_Jl1 * c.m_invM1 * c.m_linearFactor1, c.m_Ja1 * c.m_invI1 * c.m_angularFactor1);
			accum2.applyWarmStartImpulse(warmStartingImp, c.m_Jl2 * c.m_invM2 * c.m_linearFactor2, c.m_Ja2 * c.m_invI2 * c.m_angularFactor2);
			c.m_appliedImpulse = warmStartingImp;
		}

		setupFrictionConstraint(bodyA, bodyB, pt, info);
	}
}

void btCustomSISolver::setupFrictionConstraint(btRigidBody* bodyA, btRigidBody* bodyB, btManifoldPoint& pt, 
		const btContactSolverInfo& info)
{
	btVector3 rA = pt.getPositionWorldOnA() - bodyA->getWorldTransform().getOrigin();
	btVector3 nA = pt.m_normalWorldOnB;
	btMatrix3x3 invIMA = bodyA->getInvInertiaTensorWorld();
	btVector3 invIA(invIMA[0][0], invIMA[1][1], invIMA[2][2]);
	btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();
	btVector3 nB = -pt.m_normalWorldOnB;
	btMatrix3x3 invIMB = bodyB->getInvInertiaTensorWorld();
	btVector3 invIB(invIMB[0][0], invIMB[1][1], invIMB[2][2]);


	int accuId1 = getOrAllocateAccumulator(bodyA, info);
	int accuId2 = getOrAllocateAccumulator(bodyB, info);
	btVelocityAccumulator& accum1 = m_accumulatorPool[accuId1];
	btVelocityAccumulator& accum2 = m_accumulatorPool[accuId2];

	// friction
	btVector3  vel1;
	btVector3  vel2;

	vel1 = bodyA->getVelocityInLocalPoint(rA);
	vel2 = bodyB->getVelocityInLocalPoint(rB);
	btVector3 vel = vel1 - vel2;

	btVector3 t = vel - vel.dot(nA) * nA;
	const btScalar lenT = t.length();
	if (lenT > SIMD_EPSILON)
	{
		t /= lenT;
		pt.m_lateralFrictionDir1 = t;
	}
	else
	{
		//t = pt.m_lateralFrictionDir1;
		btPlaneSpace1(nA, t, pt.m_lateralFrictionDir2);
		pt.m_lateralFrictionDir1 = t;
	}

	btVector3 rXtA = rA.cross(t);
	btVector3 rXtB = rB.cross(-t);
	btScalar rel_vel;
	rel_vel = accum1.getVelocityAtContact(t, rXtA) + accum2.getVelocityAtContact(-t, rXtB);

	btSIConstraintInfo& c = m_tmpFrictionConstraintPool.expand();
	c.m_frcitionIdx = m_tmpContactConstraintPool.size() - 1;
	c.m_accumId1 = accuId1;
	c.m_accumId2 = accuId2;

	btScalar effM1 = _computeBodyEffMass(invIA, bodyA->getInvMass(), rXtA);
	btScalar effM2 = _computeBodyEffMass(invIB, bodyB->getInvMass(), rXtB);
	c.m_invEffM = 1.0f / (effM1 + effM2);

	c.m_Jl1 = t;
	c.m_Ja1 = rXtA;
	c.m_Jl2 = -t;
	c.m_Ja2 = rXtB;
	c.m_invM1 = bodyA->getInvMass();
	c.m_invM2 = bodyB->getInvMass();
	c.m_invI1 = invIA;
	c.m_invI2 = invIB;
	c.m_rhs = -rel_vel;
	c.m_origManifoldPoint = &pt;
	c.m_linearFactor1 = bodyA->getLinearFactor();
	c.m_linearFactor2 = bodyB->getLinearFactor();
	c.m_angularFactor1 = bodyA->getAngularFactor();
	c.m_angularFactor2 = bodyB->getAngularFactor();
	c.m_friction = pt.m_combinedFriction;

	if (info.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		btScalar warmStartingImp = pt.m_appliedImpulseLateral1 * info.m_warmstartingFactor;
		accum1.applyWarmStartImpulse(warmStartingImp, c.m_Jl1 * c.m_invM1 * c.m_linearFactor1, c.m_Ja1 * c.m_invI1 * c.m_angularFactor1);
		accum2.applyWarmStartImpulse(warmStartingImp, c.m_Jl2 * c.m_invM2 * c.m_linearFactor2, c.m_Ja2 * c.m_invI2 * c.m_angularFactor2);
		c.m_appliedImpulse = warmStartingImp;
	}
}

void btCustomSISolver::solveAllContacts(const btContactSolverInfo& info)
{
	for (int i = 0; i < info.m_numIterations; ++i)
	{
		const int contactCount = m_tmpContactConstraintPool.size();
		for (int ic = 0; ic < m_tmpContactConstraintPool.size(); ++ic)
		{
			solve(m_tmpContactConstraintPool[ic]);
		}
		
		for (int ic = 0; ic < m_tmpFrictionConstraintPool.size(); ++ic)
		{
			{
				btSIConstraintInfo& c = m_tmpFrictionConstraintPool[ic];
				const int frictionIdx = c.m_frcitionIdx;
				const btScalar impulse = m_tmpContactConstraintPool[frictionIdx].m_appliedImpulse;
				if (impulse >= 0)
				{
					c.m_lowerLimit = -c.m_friction * impulse;
					c.m_upperLimit = c.m_friction * impulse;
				}
			}
			solve(m_tmpFrictionConstraintPool[ic]);
		}
	}
}

void btCustomSISolver::solveAllPenetrations(const btContactSolverInfo& info)
{
	const btScalar dt = info.m_timeStep;
	for (int iter = 0; iter < info.m_numIterations; ++iter)
	{
		for (int ic = 0; ic < m_tmpContactConstraintPool.size(); ++ic)
		{
			btSIConstraintInfo& c = m_tmpContactConstraintPool[ic];
			solvePenetration(c, dt);
		}
	}
}

void btCustomSISolver::finishSolving(const btContactSolverInfo& info)
{
	for (int i = 0; i < m_accumulatorPool.size(); ++i)
	{
		btVelocityAccumulator& accum = m_accumulatorPool[i];

		btRigidBody* body = accum.m_originalBody;
		accum.applyDeltaVelocities();
		body->setLinearVelocity(accum.m_linearVelocity);
		body->setAngularVelocity(accum.m_angularVelocity);
        body->setCompanionId(-1);

		btTransform trans;
		btTransformUtil::integrateTransform(accum.m_originalBody->getWorldTransform(), accum.m_pushLinVelocity, 
				accum.m_pushAngVelcity, info.m_timeStep, trans);
		accum.m_originalBody->setWorldTransform(trans);

		//if (body->getInvMass() > 0)
		//{
		//	btVector3 lv = body->getLinearVelocity();
		//	printf("linV=%.4f, %.4f, %.4f\n", lv.getX(), lv.getY(), lv.getZ());
		//}
	}

	for (int i = 0; i < m_tmpContactConstraintPool.size(); ++i)
	{
		btSIConstraintInfo& c = m_tmpContactConstraintPool[i];
		c.m_origManifoldPoint->m_appliedImpulse = c.m_appliedImpulse;
	}

    for(int i = 0; i < m_tmpFrictionConstraintPool.size(); ++i)
    {
		btSIConstraintInfo& c = m_tmpFrictionConstraintPool[i];
		c.m_origManifoldPoint->m_appliedImpulseLateral1 = c.m_appliedImpulse;
    }
}

btScalar btCustomSISolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
	, int numManifolds, btTypedConstraint** constraints, int numConstraints
	, const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
	, btDispatcher* dispatcher)
{
	m_debugDrawer = debugDrawer;
	const btScalar dt = info.m_timeStep;
	m_tmpContactConstraintPool.clear();
	m_tmpFrictionConstraintPool.clear();
	m_accumulatorPool.clear();

	initAllAccumulators(bodies, numBodies, info);

	for (int i = 0; i < numManifolds; ++i)
	{
		setupAllContactConstraints(*manifold[i], info);
	}


	// position error correction
	solveAllPenetrations(info);

	// solver contact constraint
	solveAllContacts(info);

	finishSolving(info);

	return 0.0f;
}


void btCustomSISolver::reset()
{
}
