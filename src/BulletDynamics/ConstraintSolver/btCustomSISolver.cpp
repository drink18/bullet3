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
		btScalar lambda =  (-jV + c.m_pentrationRhs) / c.m_effM;

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

	btScalar jV = accum1.m_linearVelocity.dot(c.m_Jl1) + accum1.m_angularVelocity.dot(c.m_Ja1);
	jV += accum2.m_linearVelocity.dot(c.m_Jl2) + accum2.m_angularVelocity.dot(c.m_Ja2);
	btScalar lambda = (-jV + c.m_rhs) / c.m_effM;

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

	accum1.m_linearVelocity += c.m_Jl1 * impulse * c.m_invM1;
	accum2.m_linearVelocity += c.m_Jl2 * impulse * c.m_invM2;
	accum1.m_angularVelocity += c.m_Ja1 * impulse * c.m_invI1;
	accum2.m_angularVelocity += c.m_Ja2 * impulse * c.m_invI2;
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
		c.m_upperLimit = 1e10f;
		c.m_lowerLimit = 0;
        c.m_angularFactor1 = bodyA->getAngularFactor();
        c.m_angularFactor2 = bodyB->getAngularFactor();

		//apply warm starting impulse
		if (info.m_solverMode & SOLVER_USE_WARMSTARTING)
		{
			btScalar warmStartingImp = pt.m_appliedImpulse * info.m_warmstartingFactor;
			m_accumulatorPool[c.m_accumId1].m_linearVelocity += c.m_Jl1 * warmStartingImp * c.m_invM1;
			m_accumulatorPool[c.m_accumId1].m_angularVelocity += c.m_Ja1 * warmStartingImp * c.m_invI1;// *c.m_angularFactor1;
			m_accumulatorPool[c.m_accumId2].m_linearVelocity += c.m_Jl2 * warmStartingImp * c.m_invM2 ;
			m_accumulatorPool[c.m_accumId2].m_angularVelocity += c.m_Ja2 * warmStartingImp * c.m_invI2;// *c.m_angularFactor2;
			c.m_appliedImpulse = warmStartingImp;
		}

		btScalar penetration = pt.getDistance() + info.m_linearSlop;;
		const float beta = 0.2f;
		const float peneVelocity = -beta * penetration / info.m_timeStep;
		if (penetration < 0)
		{
            c.m_pentrationRhs = peneVelocity;
            //c.m_rhs = peneVelocity; //WWU : ?????
		}
		else
		{
			c.m_pentrationRhs = 0.0f;
            c.m_rhs = peneVelocity;
		}

		setupFrictionConstraint(bodyA, bodyB, pt, info);
	}
}

void btCustomSISolver::setupFrictionConstraint(btRigidBody* bodyA, btRigidBody* bodyB, btManifoldPoint& pt, 
		const btContactSolverInfo& info)
{
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


	// friction
	btVector3  vel1;
	btVector3  vel2;

	vel1 = bodyA->getAngularVelocity().cross(rA) + bodyA->getLinearVelocity();
	vel2 = bodyB->getAngularVelocity().cross(rB) + bodyB->getLinearVelocity();

	btVector3 vel = vel2 - vel1;
	btScalar relVel = nA.dot(vel);
    //printf("nA = %.3f, %.3f, %.3f, revVel = %.3f\n", nA.getX(), nA.getY(), nA.getZ(), relVel);

	btVector3 t = vel - relVel * nA;
	if (t.length2() > SIMD_EPSILON)
	{
		t.normalize();
		pt.m_lateralFrictionDir1 = t;
	}
	else
	{
		t = pt.m_lateralFrictionDir1;
	}


    //if(t.length2() > SIMD_EPSILON)
    {
        btSIConstraintInfo& c = m_tmpFrictionConstraintPool.expand();
        c.m_frcitionIdx = m_tmpContactConstraintPool.size() - 1;
        c.m_accumId1 = getOrAllocateAccumulator(bodyA, info);
        c.m_accumId2 = getOrAllocateAccumulator(bodyB, info);

        //if (info.m_solverMode & SOLVER_USE_WARMSTARTING)
        //{
        //btScalar warmStartingImp = pt.m_appliedImpulseLateral1 * info.m_warmstartingFactor;
        //m_accumulatorPool[c.m_accumId1].m_linearVelocity += c.m_Jl1 * warmStartingImp * c.m_invM1;
        //m_accumulatorPool[c.m_accumId1].m_angularVelocity += c.m_Ja1 * warmStartingImp * c.m_invI1;
        //m_accumulatorPool[c.m_accumId2].m_linearVelocity += c.m_Jl2 * warmStartingImp * c.m_invM2;
        //m_accumulatorPool[c.m_accumId2].m_angularVelocity += c.m_Ja2 * warmStartingImp * c.m_invI2;
        //c.m_appliedImpulse = warmStartingImp;
        //}

        btVector3 rXtA = rA.cross(t);
        btVector3 rXtB = rB.cross(-t);
        c.m_effM = _computeBodyEffMass(invIA, bodyA->getInvMass(), rXtA) + _computeBodyEffMass(invIB, bodyB->getInvMass(), rXtB);

        c.m_Jl1 = t;
        c.m_Ja1 = rXtA;
        c.m_Jl2 = -t;
        c.m_Ja2 = rXtB;
        c.m_invM1 = bodyA->getInvMass();
        c.m_invM2 = bodyB->getInvMass();
        c.m_invI1 = invIA;
        c.m_invI2 = invIB;
        c.m_origManifoldPoint = &pt;
        c.m_angularFactor1 = bodyA->getAngularFactor();
        c.m_angularFactor2 = bodyB->getAngularFactor();
    }
}

void btCustomSISolver::solveAllContacts(const btContactSolverInfo& info)
{
	for (int i = 0; i < info.m_numIterations; ++i)
	{
		for (int ic = 0; ic < m_tmpContactConstraintPool.size(); ++ic)
		{
			btSIConstraintInfo& c = m_tmpContactConstraintPool[ic];
			solve(c);
		}

		for (int ic = 0; ic < m_tmpFrictionConstraintPool.size(); ++ic)
		{
			const btScalar u = 0.3f;
			btSIConstraintInfo& c = m_tmpFrictionConstraintPool[ic];
			const int frictionIdx = c.m_frcitionIdx;
			const btScalar impulse = m_tmpContactConstraintPool[frictionIdx].m_appliedImpulse;
            if(impulse > 0)
            {
                c.m_lowerLimit = -u * impulse;
                c.m_upperLimit = u * impulse;
            }
		}

		for (int ic = 0; ic < m_tmpFrictionConstraintPool.size(); ++ic)
		{
			btSIConstraintInfo& c = m_tmpFrictionConstraintPool[ic];
			solve(c);
		}
	}
}

void btCustomSISolver::solveAllPenetrations(const btContactSolverInfo& info, int numIter)
{
	const btScalar dt = info.m_timeStep;
	for (int i = 0; i < m_tmpContactConstraintPool.size(); ++i)
	{
		btSIConstraintInfo& c = m_tmpContactConstraintPool[i];
		solvePenetration(c, dt);
	}
}

void btCustomSISolver::finishSolving(const btContactSolverInfo& info)
{
	for (int i = 0; i < m_accumulatorPool.size(); ++i)
	{
		btVelocityAccumulator& accum = m_accumulatorPool[i];

		btAssert(accum.m_linearVelocity.length2() < 1e2);
		//btAssert(accum.m_angularVelocity.length2() < 1e2);
		accum.m_originalBody->setLinearVelocity(accum.m_linearVelocity);
		accum.m_originalBody->setAngularVelocity(accum.m_angularVelocity);
        accum.m_originalBody->setCompanionId(-1);
	}

	for (int i = 0; i < m_accumulatorPool.size(); ++i)
	{
		btVelocityAccumulator& accu = m_accumulatorPool[i];
		btTransform trans;
		btTransformUtil::integrateTransform(accu.m_originalBody->getWorldTransform(), accu.m_pushLinVelocity, 
				accu.m_pushAngVelcity, info.m_timeStep, trans);
		accu.m_originalBody->setWorldTransform(trans);
	}

	for (int i = 0; i < m_tmpContactConstraintPool.size(); ++i)
	{
		btSIConstraintInfo& c = m_tmpContactConstraintPool[i];
		c.m_origManifoldPoint->m_appliedImpulse = c.m_appliedImpulse;
	}

    for(int i = 0; i < m_tmpFrictionConstraintPool.size(); ++i)
    {
		btSIConstraintInfo& c = m_tmpContactConstraintPool[i];
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

	const int numIter = info.m_numIterations;

	// position error correction
	solveAllPenetrations(info, numIter);


	//for(int i = 0; i < numIter; ++i)
	{
		// solver contact constraint
		solveAllContacts(info);
	}

	finishSolving(info);

	//for (int i = 0; i < m_tmpFrictionConstraintPool.size(); ++i)
	//{
	//	btSIConstraintInfo& c = m_tmpFrictionConstraintPool[i];
	//	if (c.m_appliedImpulse > SIMD_EPSILON)
	//	{
	//		btVelocityAccumulator& acc1 = m_accumulatorPool[c.m_accumId1];
	//		btVelocityAccumulator& acc2 = m_accumulatorPool[c.m_accumId2];
	//		btVector3 t = c.m_Jl1;
	//		btVector3 v1 = acc1.m_originalBody->getLinearVelocity();
	//		btVector3 v2 = acc2.m_originalBody->getLinearVelocity();
	//		printf("impulse= %.3f, t=%f, %f, %f, v1=%f, %f. %f, v2=%f, %f, %f\n", c.m_appliedImpulse,
	//			t.getX(), t.getY(), t.getZ(), v1.getX(), v1.getY(), v1.getZ(),
	//			v2.getX(), v2.getY(), v2.getZ());
	//	}
	//}
	//printf("*******************************************\n");
	return 0.0f;
}


void btCustomSISolver::reset()
{
}
