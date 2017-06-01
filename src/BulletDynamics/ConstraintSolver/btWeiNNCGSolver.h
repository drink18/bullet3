#pragma once

#include "btWeiSISolver.h"


ATTRIBUTE_ALIGNED16(class) btWeiNNCGConstraintSolver : public btWeiSISolver
{

protected:
	btScalar m_prevDeltaErrorSqr;

	btAlignedObjectArray<btScalar> m_pNC; //non contact constraint
	btAlignedObjectArray<btScalar> m_pC; // contact constraint
	btAlignedObjectArray<btScalar> m_pCF; // contact friction constraint

	btAlignedObjectArray<btScalar> m_deltaNC; // delta non contact constraint
	btAlignedObjectArray<btScalar> m_deltaC; // delta contact constraint
	btAlignedObjectArray<btScalar> m_deltaCF; // delta contact friction constraint

protected:
	virtual btScalar solveGroupCacheFriendlyFinish(btCollisionObject** bodies, 
				int numBodies, const btContactSolverInfo& infoGlobal);
	virtual btScalar solveSingleIteration(int iteration, btCollisionObject** bodies, int numBodies,
				btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer);

	void ClearDeltaImpulseBuffers();

	virtual btScalar solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies,
			btPersistentManifold** manifoldPtr, int numManifolds, 
			btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, 
			btIDebugDraw* debugDrawer);
public:
	btWeiNNCGConstraintSolver();
	virtual btConstraintSolverType getSolverType() const { return BT_WEI_NNCG_SOLVER; }
};
