#include "btCustomSISolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

btCustomSISolver::btCustomSISolver()
{

}


btCustomSISolver::~btCustomSISolver()
{

}


namespace
{

}

void btCustomSISolver::SolveContact(btPersistentManifold& manifold)
{
    for (int i = 0; i < manifold.getNumContacts(); ++i)
    {
        btManifoldPoint& pt = manifold.getContactPoint(i);
        
        {
            const btRigidBody* body = btRigidBody::upcast(manifold.getBody0());
            btVector3 r = pt.getPositionWorldOnA() - body->getWorldTransform().getOrigin();
            btVector3 n = pt.m_normalWorldOnB;
            btVector3 rXn = r.cross(n);
            btScalar massB = 1.0f / body->getInvMass;
            btVector3 jL = n * massB;
        }

    }

}