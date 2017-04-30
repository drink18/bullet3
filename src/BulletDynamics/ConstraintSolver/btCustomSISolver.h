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
	ATTRIBUTE_ALIGNED16(struct)  btVelocityAccumulator
	{
		BT_DECLARE_ALIGNED_ALLOCATOR(); 
		btVector3 m_externalForce;
		btVector3 m_externalTorque;
		btVector3 m_linearVelocity;
		btVector3 m_angularVelocity;
		btVector3 m_pushLinVelocity;
		btVector3 m_pushAngVelcity;
		btVector3 m_deltaLinearVelocity;
		btVector3 m_deltaAngularVelocity;

		btRigidBody* m_originalBody;

		btVelocityAccumulator()
			: m_externalForce(0, 0, 0)
			, m_externalTorque(0, 0, 0)
			, m_angularVelocity(0, 0, 0)
			, m_linearVelocity(0, 0, 0)
			, m_pushLinVelocity(0, 0, 0)
			, m_pushAngVelcity(0, 0, 0)
			, m_deltaLinearVelocity(0, 0, 0)
			, m_deltaAngularVelocity(0, 0, 0)
			, m_originalBody(nullptr)
		{
		}

		btScalar getVelocityAtContact(const btVector3& n, const btVector3& rXn) const
		{
			return n.dot(m_linearVelocity ) + rXn.dot(m_angularVelocity);
		}

		void applyDeltaVelocities()
		{
			m_linearVelocity += m_deltaLinearVelocity;
			m_angularVelocity += m_deltaAngularVelocity;
		}

		void applyDeltaImpulse(btScalar impulse,  const btVector3& linearComponent, btVector3& angularComponent)
		{
			m_deltaLinearVelocity += linearComponent * impulse;
			m_deltaAngularVelocity += angularComponent * impulse;
		}

		void applyPushImpulse(btScalar impulse,  const btVector3& linearComponent, btVector3& angularComponent)
		{
			m_pushLinVelocity += linearComponent * impulse;
			m_pushAngVelcity += angularComponent * impulse;
		}
	};

	struct btSIConstraintInfo
	{
		btScalar m_appliedImpulse;
		btScalar m_appliedPeneImpulse;
		btVector3 m_Jl1; //linear part of J
		btVector3 m_Ja1; // angular part of J
		btVector3 m_Jl2; //linear part of J
		btVector3 m_Ja2; // angular part of J
		btScalar m_invEffM; // effective mass
		btScalar m_invM1;  //inver mass 1
		btScalar m_invM2;  //inversed mass 2
		btVector3 m_invI1;  //inverse of inertial diag
		btVector3 m_invI2;  //inverse of inertial diag
		btVector3 m_linearFactor1;
		btVector3 m_linearFactor2;
		btVector3 m_angularFactor1;
		btVector3 m_angularFactor2;
		btScalar m_rhs;
		btScalar m_pentrationRhs;
		btScalar m_friction;
		btScalar m_upperLimit;
		btScalar m_lowerLimit;
		btScalar m_cfm;


		int m_accumId1;
		int m_accumId2;
		btManifoldPoint* m_origManifoldPoint;
		int m_frcitionIdx;

		btSIConstraintInfo()
			: m_appliedImpulse(0)
			, m_appliedPeneImpulse(0)
			, m_rhs(0)
			, m_pentrationRhs(0)
			, m_origManifoldPoint(nullptr)
			, m_frcitionIdx(-1)
			, m_friction(0)
			, m_lowerLimit(0)
			, m_upperLimit(0)
			, m_cfm(0)
		{
		}
	};

protected:
	void setupAllContactConstraints(btPersistentManifold& manifold, const btContactSolverInfo& info);
	void setupFrictionConstraint(btRigidBody* bodyA, btRigidBody* bodyB, btManifoldPoint& pt, 
		const btContactSolverInfo& info);
	void initAccumulator(btVelocityAccumulator& accum, btCollisionObject* body, const btContactSolverInfo& info);
	void initAllAccumulators(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& info);
	int getOrAllocateAccumulator(btCollisionObject* btBody, const btContactSolverInfo& info);

	void solveAllContacts(const btContactSolverInfo& info);
	void solveAllPenetrations(const btContactSolverInfo& info);
	void solve(btSIConstraintInfo& c);
	void solvePenetration(btSIConstraintInfo& c, btScalar dt);
	void finishSolving(const btContactSolverInfo& info);

protected:
	btAlignedObjectArray<btSIConstraintInfo> m_tmpContactConstraintPool;
	btAlignedObjectArray<btSIConstraintInfo> m_tmpFrictionConstraintPool;
	btAlignedObjectArray<btVelocityAccumulator> m_accumulatorPool;
    btIDebugDraw* m_debugDrawer;
};
