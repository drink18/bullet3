#include "btWeiSISolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "LinearMath/btIDebugDraw.h"

btWeiSISolver::btWeiSISolver()
    :m_debugDrawer(nullptr),
	m_leastResidualSquare(0.0f)
{

}


btWeiSISolver::~btWeiSISolver()
{

}

void btWeiSISolver::solvePenetration(btSIConstraintInfo& c, btScalar dt)
{
	if (c.m_pentrationRhs != 0)
	{
		btScalar jV = 0;
		btVelocityAccumulator& accu1 = m_accumulatorPool[c.m_accumId1];
		btVelocityAccumulator& accu2 = m_accumulatorPool[c.m_accumId2];

		jV += accu1.m_pushLinVelocity.dot(c.m_Jl1) + accu1.m_pushAngVelocity.dot(c.m_Ja1);
		jV += accu2.m_pushLinVelocity.dot(c.m_Jl2) + accu2.m_pushAngVelocity.dot(c.m_Ja2);
		btScalar lambda =  (-jV + c.m_pentrationRhs - c.m_appliedPeneImpulse * c.m_cfm) * c.m_invEffM;

		btScalar  accuLambda = c.m_appliedPeneImpulse + lambda;
		if (accuLambda < 0)
		{
			accuLambda = 0;
		}

		btScalar impulse = accuLambda - c.m_appliedPeneImpulse;
		c.m_appliedPeneImpulse = accuLambda;

		accu1.applyPushImpulse(impulse, c.m_Jl1 * c.m_invM1, c.m_Ja1 * c.m_invI1);
		accu2.applyPushImpulse(impulse, c.m_Jl2 * c.m_invM2, c.m_Ja2 * c.m_invI2);
	}
}

btScalar btWeiSISolver::solve(btSIConstraintInfo& c)
{
	btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
	btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];

	btScalar jV = 0;
	jV = accum1.m_deltaLinearVelocity.dot(c.m_Jl1) + accum1.m_deltaAngularVelocity.dot(c.m_Ja1);
	jV += accum2.m_deltaLinearVelocity.dot(c.m_Jl2) + accum2.m_deltaAngularVelocity.dot(c.m_Ja2);
	btScalar lambda = (-jV + c.m_rhs - c.m_appliedImpulse * c.m_cfm) * c.m_invEffM;

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

	accum1.applyDeltaImpulse(impulse, c.m_Jl1 * c.m_invM1, c.m_Ja1 * c.m_invI1);
	accum2.applyDeltaImpulse(impulse, c.m_Jl2 * c.m_invM2, c.m_Ja2 * c.m_invI2);

	return accuLambda;
}

namespace {
	btScalar _computeBodyEffMass(const btMatrix3x3& invI, const btScalar invM, const btVector3& rXn)
	{
		return invM + (invI * rXn).dot(rXn);
	}
}

void btWeiSISolver::initAccumulator(btVelocityAccumulator& accum, btCollisionObject* body, const btContactSolverInfo& info)
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

int btWeiSISolver::getOrAllocateAccumulator(btCollisionObject* btBody, const btContactSolverInfo& info)
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

void btWeiSISolver::initAllAccumulators(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& info)
{
	m_accumulatorPool.reserve(numBodies);
	for (int i = 0; i < numBodies; ++i)
	{
		int id = getOrAllocateAccumulator(bodies[i], info);
	}

}

void btWeiSISolver::setupAllTypedContraint(btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info)
{
	int totalNumRows = 0;
	for (int i = 0; i < numConstraints; ++i)
	{

		btTypedConstraint* constraint = constraints[i];
		constraint->buildJacobian();
		constraint->internalSetAppliedImpulse(0.0f);
	}

	for (int i = 0; i < numConstraints; ++i)
	{
		btTypedConstraint& typedC = *constraints[i];

		if(!typedC.isEnabled())
			continue;

		btRigidBody& rbA = typedC.getRigidBodyA();
		btRigidBody& rbB = typedC.getRigidBodyB();
		btMatrix3x3 invIA = rbA.getInvInertiaTensorWorld();
		btMatrix3x3 invIB = rbB.getInvInertiaTensorWorld();
		btScalar invMA = rbA.getInvMass();
		btScalar invMB = rbB.getInvMass();

		int accuId1 = getOrAllocateAccumulator(&rbA, info);
		int accuId2 = getOrAllocateAccumulator(&rbB, info);

		btTypedConstraint::btConstraintInfo1 info1;
		typedC.getInfo1(&info1);
		int curConstraintStartIdx = m_tmpTypedConstraintPool.size();
		for (int j = 0; j < info1.m_numConstraintRows; ++j)
		{
			btSIConstraintInfo& c = m_tmpTypedConstraintPool.expand();
			c.m_originalContraint = &typedC;
			c.m_lowerLimit = -SIMD_INFINITY;
			c.m_upperLimit = SIMD_INFINITY;
			c.m_appliedImpulse = 0.0f;
			c.m_appliedPeneImpulse = 0.0f;
			c.m_accumId1 = accuId1;
			c.m_accumId2 = accuId2;
			c.m_invM1 = invMA;
			c.m_invI1 = invIA;
			c.m_invM2 = invMB;
			c.m_invI2 = invIB;
		}

		btSIConstraintInfo& curTypeConstraint = m_tmpTypedConstraintPool[curConstraintStartIdx];
		btTypedConstraint::btConstraintInfo2 info2;
		info2.fps = 1.f / info.m_timeStep;
		info2.erp = info.m_erp;
		info2.m_J1linearAxis = curTypeConstraint.m_Jl1;
		info2.m_J1angularAxis = curTypeConstraint.m_Ja1;
		info2.m_J2linearAxis = curTypeConstraint.m_Jl2;
		info2.m_J2angularAxis = curTypeConstraint.m_Ja2;
		info2.cfm = &curTypeConstraint.m_cfm;
		info2.rowskip = sizeof(btSIConstraintInfo) / sizeof(btScalar);//check this
		info2.m_constraintError = &curTypeConstraint.m_rhs;
		curTypeConstraint.m_cfm = info.m_globalCfm;
		info2.m_damping = info.m_damping;
		info2.m_lowerLimit = &curTypeConstraint.m_lowerLimit;
		info2.m_upperLimit = &curTypeConstraint.m_upperLimit;
		info2.m_numIterations = info.m_numIterations;
		typedC.getInfo2(&info2);

		for (int j = 0; j < info1.m_numConstraintRows; ++j)
		{
			btSIConstraintInfo& c = m_tmpTypedConstraintPool[ curConstraintStartIdx + j];
			
			c.m_upperLimit = btMin(c.m_upperLimit, typedC.getBreakingImpulseThreshold());
			c.m_lowerLimit = btMax(c.m_lowerLimit, -typedC.getBreakingImpulseThreshold());

			btScalar invEffM = _computeBodyEffMass(rbA.getInvInertiaTensorWorld(), invMA, c.m_Ja1)
				+ _computeBodyEffMass(rbB.getInvInertiaTensorWorld(), invMB, c.m_Ja2);

			c.m_invEffM = 1.0f / invEffM;

			btScalar rel_vel = c.m_Jl1.dot(rbA.getLinearVelocity()) + c.m_Ja1.dot(rbA.getAngularVelocity());
			rel_vel += c.m_Jl2.dot(rbB.getLinearVelocity()) + c.m_Ja2.dot(rbB.getAngularVelocity());

			btScalar positionError = c.m_rhs;
			btScalar velocityError = -rel_vel * info2.m_damping;

			c.m_rhs = velocityError + positionError;
			c.m_appliedImpulse = 0.0f;
		}
	}
}

void btWeiSISolver::setupAllContactConstraints( btPersistentManifold& manifold, const btContactSolverInfo& info)
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
		btMatrix3x3 invIA = bodyA->getInvInertiaTensorWorld();
		btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();
		btVector3 nB = -pt.m_normalWorldOnB;
		btVector3 rXnB = rB.cross(nB);
		btMatrix3x3 invIB = bodyB->getInvInertiaTensorWorld();

		btScalar effM1 = _computeBodyEffMass(bodyA->getInvInertiaTensorWorld(), bodyA->getInvMass(), rXnA);
		//btScalar effM1 = bodyA->getInvMass() + jA1.dot(invIA);
		btScalar effM2 = _computeBodyEffMass(bodyB->getInvInertiaTensorWorld(), bodyB->getInvMass(), rXnB);
		//btScalar effM2 = bodyB->getInvMass() + jA2.dot(invIB);

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
		c.m_cfm = cfm;

		btScalar relVel = accum1.getVelocityAtContact(nA, rXnA) + accum2.getVelocityAtContact(nB, rXnB);

		btScalar penetration = pt.getDistance() + info.m_linearSlop;
		btScalar positionError = 0;
		const btScalar restituion = pt.m_combinedRestitution;
		btScalar velocityError = restituion - relVel;
		if (penetration > 0)
		{
			positionError = 0.0f;
			velocityError -= penetration * invDt;  // remove gap by adding velocity
		}
		else
		{
			positionError = -penetration * info.m_erp2 * invDt;
		}

		if (info.m_splitImpulse && penetration < info.m_splitImpulsePenetrationThreshold)
		{
			c.m_rhs = velocityError;
			c.m_pentrationRhs = positionError;
		}
		else
		{
			c.m_rhs = velocityError + positionError;
			c.m_pentrationRhs = 0.0f;
		}

		//apply warm starting impulse
		if (info.m_solverMode & SOLVER_USE_WARMSTARTING)
		{
			btScalar warmStartingImp = pt.m_appliedImpulse * info.m_warmstartingFactor;
			accum1.applyDeltaImpulse(warmStartingImp, c.m_Jl1 * c.m_invM1 * c.m_linearFactor1, c.m_Ja1 * c.m_invI1 * c.m_angularFactor1);
			accum2.applyDeltaImpulse(warmStartingImp, c.m_Jl2 * c.m_invM2 * c.m_linearFactor2, c.m_Ja2 * c.m_invI2 * c.m_angularFactor2);
			c.m_appliedImpulse = warmStartingImp;
		}

		setupFrictionConstraint(bodyA, bodyB, pt, info);
	}
}

void btWeiSISolver::setupFrictionConstraint(btRigidBody* bodyA, btRigidBody* bodyB, btManifoldPoint& pt, 
		const btContactSolverInfo& info)
{
	btVector3 rA = pt.getPositionWorldOnA() - bodyA->getWorldTransform().getOrigin();
	btVector3 nA = pt.m_normalWorldOnB;
	btMatrix3x3 invIA = bodyA->getInvInertiaTensorWorld();
	btVector3 rB = pt.getPositionWorldOnB() - bodyB->getWorldTransform().getOrigin();
	btVector3 nB = -pt.m_normalWorldOnB;
	btMatrix3x3 invIB = bodyB->getInvInertiaTensorWorld();


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
	c.m_cfm = 0;

	if (info.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		btScalar warmStartingImp = pt.m_appliedImpulseLateral1 * info.m_warmstartingFactor;
		accum1.applyDeltaImpulse(warmStartingImp, c.m_Jl1 * c.m_invM1 * c.m_linearFactor1, c.m_Ja1 * c.m_invI1 * c.m_angularFactor1);
		accum2.applyDeltaImpulse(warmStartingImp, c.m_Jl2 * c.m_invM2 * c.m_linearFactor2, c.m_Ja2 * c.m_invI2 * c.m_angularFactor2);
		c.m_appliedImpulse = warmStartingImp;
	}
}

btScalar btWeiSISolver::solveAllContacts(const btContactSolverInfo& info)
{
	btScalar residualSquare = 0.0f;
	for (int ic = 0; ic < m_tmpTypedConstraintPool.size(); ++ic)
	{
		btScalar error = solve(m_tmpTypedConstraintPool[ic]);
		residualSquare += error * error;
	}

	for (int ic = 0; ic < m_tmpContactConstraintPool.size(); ++ic)
	{
		btScalar error = solve(m_tmpContactConstraintPool[ic]);
		residualSquare += error * error;
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
		btScalar error = solve(m_tmpFrictionConstraintPool[ic]);
		residualSquare += error * error;
	}
	return residualSquare;
}

btScalar btWeiSISolver::solveSingleIteration(int iteration, btCollisionObject** /*bodies */, int /*numBodies*/, btPersistentManifold** /*manifoldPtr*/, int /*numManifolds*/, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* /*debugDrawer*/)
{
	solvePositionErrors(infoGlobal);
	btScalar residualSquare = solveAllContacts(infoGlobal);

	return residualSquare;
}

void btWeiSISolver::solvePositionErrors(const btContactSolverInfo& info)
{
	const btScalar dt = info.m_timeStep;
	for (int ic = 0; ic < m_tmpContactConstraintPool.size(); ++ic)
	{
		btSIConstraintInfo& c = m_tmpContactConstraintPool[ic];
		solvePenetration(c, dt);
	}
}

void btWeiSISolver::finishSolving(const btContactSolverInfo& info)
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
				accum.m_pushAngVelocity, info.m_timeStep, trans);
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

	for (int i = 0; i < m_tmpTypedConstraintPool.size(); ++i)
	{
		btSIConstraintInfo& c = m_tmpTypedConstraintPool[i];
		if (btFabs(c.m_appliedImpulse) >= c.m_originalContraint->getBreakingImpulseThreshold())
		{
			c.m_originalContraint->setEnabled(false);
		}
	}
}


btScalar btWeiSISolver::solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr,
		int numManifolds, btTypedConstraint** constraints, int numConstraints, 
		const btContactSolverInfo& info, btIDebugDraw* debugDrawer)
{
	m_debugDrawer = debugDrawer;
	const btScalar dt = info.m_timeStep;
	m_tmpContactConstraintPool.clear();
	m_tmpFrictionConstraintPool.clear();
	m_tmpTypedConstraintPool.clear();
	m_accumulatorPool.clear();

	initAllAccumulators(bodies, numBodies, info);

	setupAllTypedContraint(constraints, numConstraints, info);
	for (int i = 0; i < numManifolds; ++i)
	{
		setupAllContactConstraints(*manifoldPtr[i], info);
	}
	return 0.0f;
}

btScalar btWeiSISolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
	, int numManifolds, btTypedConstraint** constraints, int numConstraints
	, const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
	, btDispatcher* dispatcher)
{
	solveGroupCacheFriendlySetup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

	for (int iter = 0; iter < info.m_numIterations; ++iter)
	{
		m_leastResidualSquare = solveSingleIteration(iter, bodies, numBodies, manifold, numManifolds, 
			constraints, numConstraints, info, debugDrawer);
	}

	solveGroupCacheFriendlyFinish(bodies, numBodies, info);

	return 0.0f;
}

btScalar btWeiSISolver::solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies,
	const btContactSolverInfo& infoGlobal)
{
	finishSolving(infoGlobal);
	return 0.0f;
}



void btWeiSISolver::reset()
{
}
