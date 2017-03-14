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
        btScalar m_appliedImpulse;
        btVector3 m_Jl; //linear part of J
        btVector3 m_Ja; // angular part of J
        btScalar m_effM; // effective mass
        btScalar m_invM; // inverse mass
        btVector3 m_invI;  //inverse of inertial diag
        btRigidBody* m_body; //linked rigid body

        btSolverConstraint()
            : m_appliedImpulse(0)
        {
        }
    };
protected:
    void setupAllContactConstratins(btPersistentManifold& manifold, const btContactSolverInfo& info);
    void setupContactConstraint(btRigidBody* body, btVector3& n, btVector3& rXn, btVector3& invI);

    void solveAllContacts(btScalar dt);
    void solve(btSolverConstraint& c, btScalar dt);
protected:

    btAlignedObjectArray<btSolverBody> m_tmpSolverBodyPool;
    btAlignedObjectArray<btSolverConstraint> m_tmpConstraintPool;
private:

};
