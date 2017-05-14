#ifndef BT_MULTIBODY_CONSTRAINT_SOLVER_I_H
#define BT_MULTIBODY_CONSTRAINT_SOLVER_I_H

class btMultiBody;


ATTRIBUTE_ALIGNED16(class) btMultiBodyConstraintSolverI  
{

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this method should not be called, it was just used during porting/integration of Featherstone btMultiBody, providing backwards compatibility but no support for btMultiBodyConstraint (only contact constraints)
	virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher) = 0;
	virtual btScalar solveGroupCacheFriendlyFinish(btCollisionObject** bodies,int numBodies,const btContactSolverInfo& infoGlobal) =0;
	
	virtual void solveMultiBodyGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher) = 0;

	virtual btConstraintSolver* getBaseSolver() = 0;
};

#endif //BT_MULTIBODY_CONSTRAINT_SOLVER_H
