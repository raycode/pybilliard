#ifndef BT_USER_MOTION_STATE_H
#define BT_USER_MOTION_STATE_H

#include "btBulletCollisionCommon.h"

typedef btTransform (* pyGetWorldTransform)(void *);

///The btUserMotionState provides a common implementation to synchronize world transforms with offsets.
struct	btUserMotionState : public btMotionState
{
	btTransform          m_graphicsWorldTrans;
	btTransform          m_centerOfMassOffset;
	btTransform          m_startWorldTrans;
	void*		     m_userPointer;
        void*                m_pyObject;
        pyGetWorldTransform  m_func;

	btUserMotionState(pyGetWorldTransform func, void *pyObject)
            : m_pyObject(pyObject), m_func(func)
	{
            m_graphicsWorldTrans = btTransform::getIdentity();
	    m_centerOfMassOffset = btTransform::getIdentity();
	    m_startWorldTrans = btTransform::getIdentity();
	    m_userPointer = NULL;
	}

	///synchronizes world transform from user to physics
	virtual void	getWorldTransform(btTransform& centerOfMassWorldTrans) const 
	{
            centerOfMassWorldTrans = m_func(m_pyObject);
        }

	///synchronizes world transform from physics to user
	///Bullet only calls the update of worldtransform for active objects
	virtual void	setWorldTransform(const btTransform& centerOfMassWorldTrans)
	{
            m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset ;
	}
};

#endif //BT_USER_MOTION_STATE_Hinclude "btBulletCollisionCommon.h"


