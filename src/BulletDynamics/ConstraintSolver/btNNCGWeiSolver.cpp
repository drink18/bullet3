#include "btWeiNNCGSolver.h"

btWeiNNCGConstraintSolver::btWeiNNCGConstraintSolver()
	: m_prevDeltaErrorSqr(0)
{
}

btScalar btWeiNNCGConstraintSolver::solveSingleIteration(int iteration, btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	return 0.0f;
}

btScalar btWeiNNCGConstraintSolver::solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal)
{
	return 0.0f;
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

