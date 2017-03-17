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

    struct btSolverConstraint
    {
        btScalar m_appliedImpulse1;
        btScalar m_appliedImpulse2;
        btVector3 m_Jl1; //linear part of J
        btVector3 m_Ja1; // angular part of J
        btVector3 m_Jl2; //linear part of J
        btVector3 m_Ja2; // angular part of J
        btScalar m_effM; // effective mass
        btScalar m_invM1;  //inver mass 1
        btScalar m_invM2;  //inversed mass 2
        btVector3 m_invI1;  //inverse of inertial diag
        btVector3 m_invI2;  //inverse of inertial diag
        btRigidBody* m_body1; //linked rigid body
        btRigidBody* m_body2; //linked rigid body

        btSolverConstraint()
            : m_appliedImpulse1(0)
            , m_appliedImpulse2(0)
			, m_body1(nullptr)
			, m_body2(nullptr)
        {
        }
    };
protected:
    void setupAllContactConstratins(btPersistentManifold& manifold, const btContactSolverInfo& info);
    void setupContactConstraint(btRigidBody* body1, btRigidBody* body2, btVector3& n
									, btVector3& rXn, btVector3& rXn2);

    void solveAllContacts(btScalar dt);
    void solve(btSolverConstraint& c, btScalar dt);
protected:

    btAlignedObjectArray<btSolverConstraint> m_tmpConstraintPool;
private:

};
