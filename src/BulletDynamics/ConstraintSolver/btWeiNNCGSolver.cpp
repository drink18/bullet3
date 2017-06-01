#include "btWeiNNCGSolver.h"

btWeiNNCGConstraintSolver::btWeiNNCGConstraintSolver()
	: m_prevDeltaErrorSqr(0)
{
}

btScalar btWeiNNCGConstraintSolver::solveSingleIteration(int iteration, btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	int numNonContactPool = m_tmpTypedConstraintPool.size();
	int numContraintsPool = m_tmpContactConstraintPool.size();
	int numFrictionPool = m_tmpFrictionConstraintPool.size();

	btScalar deltaErrSqr = 0;
	
	for (int j = 0; j < m_tmpTypedConstraintPool.size(); j++)
	{
		btScalar deltaf = solve(m_tmpTypedConstraintPool[j]);
		m_deltaNC[j] = deltaf;
		deltaErrSqr += deltaf * deltaf;
	}

	for (int j = 0; j < m_tmpContactConstraintPool.size(); j++)
	{

		btScalar deltaf = solve(m_tmpContactConstraintPool[j]);
		m_deltaC[j] = deltaf;
		deltaErrSqr += deltaf * deltaf;
	}

	for (int j = 0; j < m_tmpFrictionConstraintPool.size(); j++)
	{

		{
			btSIConstraintInfo& c = m_tmpFrictionConstraintPool[j];
			const int frictionIdx = c.m_frcitionIdx;
			const btScalar impulse = m_tmpContactConstraintPool[frictionIdx].m_appliedImpulse;
			if (impulse >= 0)
			{
				c.m_lowerLimit = -c.m_friction * impulse;
				c.m_upperLimit = c.m_friction * impulse;
			}
		}
		btScalar deltaf = solve(m_tmpFrictionConstraintPool[j]);
		m_deltaCF[j] = deltaf;
		deltaErrSqr += deltaf * deltaf;
	}

	if (iteration == 0)
	{
		ClearDeltaImpulseBuffers();

	}
	else
	{
		btScalar beta = m_prevDeltaErrorSqr > 0 ? deltaErrSqr / m_prevDeltaErrorSqr : 2;
		if (beta > 1)
		{
			ClearDeltaImpulseBuffers();
		}
		else
		{
			for (int j = 0; j < m_tmpTypedConstraintPool.size(); j++)
			{
				btSIConstraintInfo& c = m_tmpTypedConstraintPool[j];
				m_pNC[j] = beta * m_pNC[j] + m_deltaNC[j];
				btScalar additionalImp = beta * m_pNC[j];
				c.m_appliedImpulse += additionalImp;
				
				btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
				btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];
				accum1.applyDeltaImpulse(additionalImp, c.m_Jl1 * c.m_invM1, c.m_Ja1 * c.m_invI1);
				accum2.applyDeltaImpulse(additionalImp, c.m_Jl2 * c.m_invM2, c.m_Ja2 * c.m_invI2);
			}

			for (int j = 0; j < m_tmpContactConstraintPool.size(); j++)
			{
				btSIConstraintInfo& c =  m_tmpContactConstraintPool[j];
				m_pC[j] = beta * m_pC[j] + m_deltaC[j];
				btScalar additionalImp = beta * m_pC[j];
				c.m_appliedImpulse += additionalImp;
				
				btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
				btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];
				accum1.applyDeltaImpulse(additionalImp, c.m_Jl1 * c.m_invM1, c.m_Ja1 * c.m_invI1);
				accum2.applyDeltaImpulse(additionalImp, c.m_Jl2 * c.m_invM2, c.m_Ja2 * c.m_invI2);
			}

			for (int j = 0; j < m_tmpFrictionConstraintPool.size(); j++)
			{
				btSIConstraintInfo& c =   m_tmpFrictionConstraintPool[j];
				m_pCF[j] = beta * m_pCF[j] + m_deltaCF[j];
				btScalar additionalImp = beta * m_pCF[j];
				c.m_appliedImpulse += additionalImp;
				
				btVelocityAccumulator& accum1 = m_accumulatorPool[c.m_accumId1];
				btVelocityAccumulator& accum2 = m_accumulatorPool[c.m_accumId2];
				accum1.applyDeltaImpulse(additionalImp, c.m_Jl1 * c.m_invM1, c.m_Ja1 * c.m_invI1);
				accum2.applyDeltaImpulse(additionalImp, c.m_Jl2 * c.m_invM2, c.m_Ja2 * c.m_invI2);
			}
		}
	}

	m_prevDeltaErrorSqr = deltaErrSqr;
	return deltaErrSqr;

}

btScalar btWeiNNCGConstraintSolver::solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal)
{
	m_pNC.resizeNoInitialize(0);
	m_pC.resizeNoInitialize(0);
	m_pCF.resizeNoInitialize(0);

	m_deltaNC.resizeNoInitialize(0);
	m_deltaC.resizeNoInitialize(0);
	m_deltaCF.resizeNoInitialize(0);

	return btWeiSISolver::solveGroupCacheFriendlyFinish(bodies, numBodies, infoGlobal);
}

btScalar btWeiNNCGConstraintSolver::solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies, 
	btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, 
	const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	btScalar val = btWeiSISolver::solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
	
	m_pNC.resizeNoInitialize(m_tmpTypedConstraintPool.size());
	m_pC.resizeNoInitialize(m_tmpContactConstraintPool.size());
	m_pCF.resizeNoInitialize(m_tmpFrictionConstraintPool.size());

	m_deltaNC.resizeNoInitialize(m_tmpTypedConstraintPool.size());
	m_deltaC.resizeNoInitialize(m_tmpContactConstraintPool.size());
	m_deltaCF.resizeNoInitialize(m_tmpFrictionConstraintPool.size());

	return val;
}

void btWeiNNCGConstraintSolver::ClearDeltaImpulseBuffers()
{
	for (int j = 0; j < m_tmpTypedConstraintPool.size(); j++)
		m_pNC[j] = 0;

	for (int j = 0; j < m_tmpContactConstraintPool.size(); j++)
		m_pC[j] = 0;

	for (int j = 0; j < m_tmpFrictionConstraintPool.size(); j++)
		m_pCF[j] = 0;
}

