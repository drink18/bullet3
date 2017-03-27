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

ATTRIBUTE_ALIGNED16(class) btCustomSISolver :
public btConstraintSolver
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
	virtual	void	reset() override;

	virtual btConstraintSolverType	getSolverType() const
	{
		return BT_SEQUENTIAL_IMPULSE_SOLVER;
	}

	// body data used for solver
	ATTRIBUTE_ALIGNED16(struct) btBodyData
	{
		BT_DECLARE_ALIGNED_ALLOCATOR(); 
		btVector3 m_deltaLinearVelocity;
		btVector3 m_deltaAngularVelocity;
		btVector3 m_invM;
		btVector3 m_pushLinVelocity;
		btVector3 m_pushAngVelcity;

		btRigidBody* m_originalBody;
	};

	struct btSolverConstraint
	{
		btScalar m_appliedImpulse;
		btScalar m_appliedPeneImpulse;
		btVector3 m_Jl1; //linear part of J
		btVector3 m_Ja1; // angular part of J
		btVector3 m_Jl2; //linear part of J
		btVector3 m_Ja2; // angular part of J
		btScalar m_effM; // effective mass
		btScalar m_invM1;  //inver mass 1
		btScalar m_invM2;  //inversed mass 2
		btVector3 m_invI1;  //inverse of inertial diag
		btVector3 m_invI2;  //inverse of inertial diag
		btScalar m_rhs;
		btScalar m_pentrationRhs;
		btVector3 m_pushLinearVelocity1;
		btVector3 m_pushLinearVelocity2;
		btVector3 m_pushAngularVelocity1;
		btVector3 m_pushAngularVelocity2;
		btRigidBody* m_body1; //linked rigid body
		btRigidBody* m_body2; //linked rigid body
		btBodyData m_solBody1;
		btBodyData m_solBody2;

		btSolverConstraint()
			: m_appliedImpulse(0)
			, m_appliedPeneImpulse(0)
			, m_pushLinearVelocity1(0, 0, 0)
			, m_pushLinearVelocity2(0, 0, 0)
			, m_pushAngularVelocity1(0, 0, 0)
			, m_pushAngularVelocity2(0, 0, 0)
			, m_rhs(0)
			, m_pentrationRhs(0)
			, m_body1(nullptr)
			, m_body2(nullptr)
		{
		}
	};

protected:
	void setupAllContactConstratins(btPersistentManifold& manifold, const btContactSolverInfo& info);
	void setupContactConstraint(btRigidBody* body1, btRigidBody* body2, btVector3& n
									, btVector3& rXn, btVector3& rXn2);

	void solveAllContacts(btScalar dt, int numIter);
	void solveAllPenetrations(btScalar dt, int numIter);
	void solve(btSolverConstraint& c, btScalar dt);
	void solvePenetration(btSolverConstraint& c, btScalar dt);
protected:

	btAlignedObjectArray<btSolverConstraint> m_tmpConstraintPool;
private:

};
