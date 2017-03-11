#pragma once
class btIDebugDraw;
class btPersistentManifold;
class btDispatcher;
class btCollisionObject;

#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"

ATTRIBUTE_ALIGNED16(class) btCustomSISolver : public btConstraintSolver
{
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    btCustomSISolver();
    virtual ~btCustomSISolver();

    virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold
        , int numManifolds, btTypedConstraint** constraints, int numConstraints
        , const btContactSolverInfo& info, class btIDebugDraw* debugDrawer
        , btDispatcher* dispatcher) override;
    ///clear internal cached data and reset random seed
    virtual	void	reset();

protected:

    btAlignedObjectArray<btSolverBody> m_tmpSolverBodyPool;
    btConstraintArray m_tmpSolverContactConstraintPool;

};
