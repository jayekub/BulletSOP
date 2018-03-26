#pragma once
/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
* can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement (which
* also govern the use of this file).  You may share a modified version of this
* file with another authorized licensee of Derivative's TouchDesigner software.
* Otherwise, no redistribution or sharing of this file, with or without
* modification, is permitted.
*/


#include <string>

#include <map>
#include <vector>

#define BT_USE_DOUBLE_PRECISION
#include <btBulletDynamicsCommon.h>

#include "SOP_CPlusPlusBase.h"

// To get more help about these functions, look at SOP_CPlusPlusBase.h
class BulletSOP : public SOP_CPlusPlusBase
{
public:

	BulletSOP(const OP_NodeInfo*);
	virtual ~BulletSOP();

	virtual void getGeneralInfo(SOP_GeneralInfo*) override;

	virtual void execute(SOP_Output*, OP_Inputs*, void* reserved) override;
	virtual void executeVBO(SOP_VBOOutput*, OP_Inputs*, void*) override { }

	virtual bool getInfoDATSize(OP_InfoDATSize*) override;
	virtual void getInfoDATEntries(int32_t, int32_t, OP_InfoDATEntries*) override;

	virtual void setupParameters(OP_ParameterManager*) override;
	virtual void pulsePressed(char const*) override;

	virtual char const* getInfoPopupString() override { return _infoString; }
	virtual char const* getWarningString() override { return _warningString; }
	virtual char const* getErrorString() override { return _errorString;  }

private:

	btScalar _getDeltaTimeMicroseconds();

	void _addSOPInput(OP_SOPInput const*);
	void _syncSOPInput(OP_SOPInput const*);

	void _worldSetup();
	void _worldDestroy();

	void _objectsDestroy();

	void _recenterMesh(btTriangleMesh*, btVector3 const&);

	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	btClock clock;

	bool _reset;

	struct _object {
		/*
		enum CollisionShapeType
		{
			CONVEX_HULL = 0,
			SPHERE,
			BOX,
			STATIC_BVH_MESH
		};
		CollisionShapeType shapeType;
		*/

		~_object()
		{
			delete mesh;
			delete shape;
			delete motionState;
			delete body;
		}

		int32_t id;
		float mass;

		std::vector<int> inputIndices;

		btTriangleMesh* mesh;
		btCollisionShape* shape;

		btMotionState* motionState;
		btRigidBody* body;
	};

	std::map<int32_t, _object*> _objects;
	std::vector<int32_t> _objectIds;

	const char* _infoString;
	const char* _warningString;
	const char* _errorString;
};
