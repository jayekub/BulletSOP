#include "BulletSOP.h"

#include <sstream>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

BulletSOP::BulletSOP(const OP_NodeInfo* info) :
	_initialized(false), _infoString(nullptr), _warningString(nullptr), _errorString(nullptr)
{
	worldSetup();
}

BulletSOP::~BulletSOP()
{
	worldDestroy();
}

void
BulletSOP::getGeneralInfo(SOP_GeneralInfo* info)
{
	info->cookEveryFrame = true;
	info->cookEveryFrameIfAsked = false;
	info->directToGPU = false;
}

void
BulletSOP::execute(SOP_Output* output, OP_Inputs* inputs, void* reserved)
{
	if (!_initialized)
	{
		_initialized = true;

		for (int i = 0; i < inputs->getNumInputs(); ++i)
		{
			addSOPGeometry(inputs->getInputSOP(i));
		}
	}

	dynamicsWorld->setGravity(btVector3(0, -10, 0));
	btScalar ms = getDeltaTimeMicroseconds()/1e6;
	dynamicsWorld->stepSimulation(ms);

	int outPointIndex = 0;

	btCollisionObjectArray colObjs = dynamicsWorld->getCollisionObjectArray();
	for (int o = 0; o < dynamicsWorld->getNumCollisionObjects(); ++o)
	{
		btCollisionObject* obj = colObjs[o];
		btRigidBody* body = btRigidBody::upcast(obj);
		// TODO support more shape types
		btConvexHullShape* shape = dynamic_cast<btConvexHullShape*>(body->getCollisionShape());

		// avoid inevitable crashes
		if (!shape)
			continue;

		btVector3 const* points = shape->getUnscaledPoints();
		btTransform transform = body->getWorldTransform();

		for (int i = 0; i < shape->getNumPoints(); ++i)
		{
			btVector3 point = points[i];

			point = transform*point;

			output->addPoint(
				static_cast<float>(point.x()), 
				static_cast<float>(point.y()), 
				static_cast<float>(point.z()));

			if (i%3 == 2)
			{
				output->addTriangle(outPointIndex - 2, outPointIndex - 1, outPointIndex);
			}

			++outPointIndex;
		}
	}
}

bool
BulletSOP::getInfoDATSize(
	OP_InfoDATSize* infoSize)
{
	infoSize->rows = static_cast<int32_t>(_objects.size() + 2);
	infoSize->cols = 2;
	infoSize->byColumn = false;

	return true;
}

void
BulletSOP::getInfoDATEntries(
	int32_t index,
	int32_t nEntries,
	OP_InfoDATEntries* entries)
{
	switch (index)
	{
	case 0:
		entries->values[0] = "info";
		entries->values[1] = strdup(_infoString);
		break;

	case 1:
		entries->values[0] = "id";
		entries->values[1] = "mass";
		break;

	default:
		{
		_object* obj = _objects[_objectIds[index - 2]];

		entries->values[0] = strdup(std::to_string(obj->id).c_str());
		entries->values[1] = strdup(std::to_string(obj->mass).c_str());
		break;
		}
	}

}

btScalar
BulletSOP::getDeltaTimeMicroseconds()
{
	btScalar dt = (btScalar)clock.getTimeMicroseconds();
	clock.reset();
	return dt;
}

void
BulletSOP::worldSetup() 
{
	collisionConfiguration = new btDefaultCollisionConfiguration;
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;

	broadphase = new btDbvtBroadphase();

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
	info.m_splitImpulse = 1;
	info.m_splitImpulsePenetrationThreshold = -0.02;

}

void 
BulletSOP::worldDestroy()
{
	delete dynamicsWorld;
	delete solver;
	delete broadphase;
	delete dispatcher;
	delete collisionConfiguration;
}

void
BulletSOP::addSOPGeometry(OP_SOPInput const* sopGeom)
{
	/*
	CustomAttribInfo const* massAttr = sopGeom->getCustomAttribute("mass");
	CustomAttribInfo const* objIdAttr = sopGeom->getCustomAttribute("objId");

	std::stringstream info;

	for (int p = 0; p < sopGeom->getNumPoints(); ++p)
	{
		info << "(" << p << " " << objIdAttr->intData[p] << " " << massAttr->floatData[p] << ") ";
	}

	_infoString = info.str().c_str();

	if (!(massAttr &&
		  massAttr->attribType == AttribType::Float &&
		  massAttr->numComponents == 1) ||

		!(objIdAttr &&
		  objIdAttr->attribType == AttribType::Int &&
		  objIdAttr->numComponents == 1))
	{
		_errorString = "expected point attributes mass[1] and/or objId[1](int) not found";
		return;
	}
	*/

	Position const* sopPoints = sopGeom->getPointPositions();
	ColorInfo const* sopColors = sopGeom->getColors();

	for (int p = 0; p < sopGeom->getNumPrimitives(); ++p)
	{
		PrimitiveInfo const& primInfo = sopGeom->getPrimitive(p);

		// XXX only point attrs are supported, assume constant
		int32_t firstIndex = primInfo.pointIndices[0];
		int32_t objId;// = objIdAttr->intData[firstIndex];

		// XXX hackity hack hack
		Color const& primColor = sopColors->colors[firstIndex];
		objId = primColor.r;

		_object* obj;
		auto objIt = _objects.find(objId);

		if (objIt == _objects.end())
		{
			obj = new _object();

			obj->id = objId;
			//obj->mass = massAttr->floatData[firstIndex];
			obj->mass = primColor.g;
			obj->shape = new btConvexHullShape();

			_objects[objId] = obj;
			_objectIds.push_back(objId);
		}
		else
		{
			obj = objIt->second;
		}


		//info << "(" << p << " " << firstIndex << " " << objId << " " << obj->mass << ") ";

		for (int v = 0; v < primInfo.numVertices; ++v)
		{
			int index = primInfo.pointIndices[v];
			obj->shape->addPoint(
				btVector3(sopPoints[index].x, sopPoints[index].y, sopPoints[index].z));
		}
	}

	for (auto item : _objects)
	{
		_object* obj = item.second;

		// use bounding sphere to get reasonable local space for prim
		btVector3 primCenter;
		btScalar primRadius;
		obj->shape->getBoundingSphere(primCenter, primRadius);

		bool dynamic = (obj->mass > 0.);

		btVector3 localInertia(0, 0, 0);
		if (dynamic)
			obj->shape->calculateLocalInertia(obj->mass, localInertia);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(primCenter);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* motionState = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(obj->mass, motionState, obj->shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		if (!dynamic) {
			body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}

		dynamicsWorld->addRigidBody(body);
	}
}

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{
	DLLEXPORT
	int32_t
	GetSOPAPIVersion(void)
	{
		// Always return SOP_CPLUSPLUS_API_VERSION in this function.
		return SOP_CPLUSPLUS_API_VERSION;
	}

	DLLEXPORT
	SOP_CPlusPlusBase*
	CreateSOPInstance(const OP_NodeInfo* info)
	{
		// Return a new instance of your class every time this is called.
		// It will be called once per SOP that is using the .dll
		return new BulletSOP(info);
	}

	DLLEXPORT
	void
	DestroySOPInstance(SOP_CPlusPlusBase* instance)
	{
		// Delete the instance here, this will be called when
		// Touch is shutting down, when the SOP using that instance is deleted, or
		// if the SOP loads a different DLL
		delete (BulletSOP*)instance;
	}
};
