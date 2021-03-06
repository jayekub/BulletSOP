#include "BulletSOP.h"

#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <algorithm>
#include <limits>
#include <sstream>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

BulletSOP::BulletSOP(const OP_NodeInfo* info) :
	_reset(true), _infoString(nullptr), _warningString(nullptr), _errorString(nullptr)
{
	_worldSetup();
}

BulletSOP::~BulletSOP()
{
	_worldDestroy();
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
	if (inputs->getNumInputs() < 1)
	{
		return;
	}

	if (_reset)
	{
		_objectsDestroy();
		_addSOPInput(inputs->getInputSOP(0));

		_reset = false;
	}
	else
	{
		if (inputs->getParInt("Syncinput") > 0)
		{
			_syncSOPInput(inputs->getInputSOP(0));
		}
	}

	btScalar gx, gy, gz;
	inputs->getParDouble3("Gravity", gx, gy, gz);
	dynamicsWorld->setGravity(btVector3(gx, gy, gz));

	btScalar ms = _getDeltaTimeMicroseconds()/1e6;
	dynamicsWorld->stepSimulation(
		ms,
		inputs->getParInt("Maxsubsteps"),
		inputs->getParDouble("Timeinc"));

	int outIndex = 0;

	for (auto item : _objects)
	{
		_object* obj = item.second;

		unsigned char const* vertexBase;
		int numVerts;
		PHY_ScalarType type;
		int vertexStride;
		unsigned char const* indexBase;
		int indexStride;
		int numFaces;
		PHY_ScalarType indicesType;

		btTransform transform;
		obj->motionState->getWorldTransform(transform);

		for (int s = 0; s < obj->mesh->getNumSubParts(); ++s)
		{
			obj->mesh->getLockedReadOnlyVertexIndexBase(
				&vertexBase, numVerts, type, vertexStride,
				&indexBase, indexStride, numFaces, indicesType, s);

			if (type != PHY_DOUBLE)
			{
				_warningString = "encountered non-double verts from Bullet, ignored";
				obj->mesh->unLockReadOnlyVertexBase(s);
				continue;
			}

			if (indicesType != PHY_INTEGER)
			{
				_warningString = "encountered non-int indices from Bullet, ignored";
				obj->mesh->unLockReadOnlyVertexBase(s);
				continue;
			}

			for (int i = 0; i < numFaces; ++i)
			{
				int const* indices = reinterpret_cast<int const*>(indexBase + i * indexStride);

				double const* vertBase0 = reinterpret_cast<double const*>(vertexBase + indices[0] * vertexStride);
				double const* vertBase1 = reinterpret_cast<double const*>(vertexBase + indices[1] * vertexStride);
				double const* vertBase2 = reinterpret_cast<double const*>(vertexBase + indices[2] * vertexStride);

				btVector3 vert0(vertBase0[0], vertBase0[1], vertBase0[2]);
				btVector3 vert1(vertBase1[0], vertBase1[1], vertBase1[2]);
				btVector3 vert2(vertBase2[0], vertBase2[1], vertBase2[2]);

				vert0 = transform*vert0;
				vert1 = transform*vert1;
				vert2 = transform*vert2;

				output->addPoint(static_cast<float>(vert0.x()), static_cast<float>(vert0.y()), static_cast<float>(vert0.z()));
				output->addPoint(static_cast<float>(vert1.x()), static_cast<float>(vert1.y()), static_cast<float>(vert1.z()));
				output->addPoint(static_cast<float>(vert2.x()), static_cast<float>(vert2.y()), static_cast<float>(vert2.z()));

				output->addTriangle(outIndex, outIndex + 1, outIndex + 2);

				outIndex += 3;
			}

			/* XXX this should be more efficient, why doesn't it work??
			int indexOffset = outIndex;

			for (int v = 0; v < numVerts; ++v)
			{
				double const* vert = reinterpret_cast<double const*>(vertexBase + v*vertexStride);
				btVector3 point(vert[0], vert[1], vert[2]);

				point = transform * point;

				output->addPoint(
					static_cast<float>(point.x()),
					static_cast<float>(point.y()),
					static_cast<float>(point.z()));

				++outIndex;
			}

			for (int i = 0; i < numFaces; ++i)
			{
				int const* indices = reinterpret_cast<int const*>(indexBase + i*indexStride);
				output->addTriangle(
					indexOffset + indices[0],
					indexOffset + indices[1],
					indexOffset + indices[2]);
			}
			*/

			obj->mesh->unLockReadOnlyVertexBase(s);
		}
	}
}

bool
BulletSOP::getInfoDATSize(
	OP_InfoDATSize* infoSize)
{
	infoSize->rows = static_cast<int32_t>(_objects.size() + 4);
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
		entries->values[1] = const_cast<char*>(_infoString);
		break;

	case 1:
		entries->values[0] = "warning";
		entries->values[1] = const_cast<char*>(_warningString);
		break;

	case 2:
		entries->values[0] = "error";
		entries->values[1] = const_cast<char*>(_errorString);
		break;

	case 3:
		entries->values[0] = "id";
		entries->values[1] = "mass";
		break;

	default:
		{
		_object* obj = _objects[_objectIds[index - 4]];

		static std::string value0;
		static std::string value1;
		
		value0 = std::to_string(obj->id); 
		value1 = std::to_string(obj->mass);
		
		entries->values[0] = const_cast<char*>(value0.c_str());
		entries->values[1] = const_cast<char*>(value1.c_str());
		break;
		}
	}

}

void
BulletSOP::setupParameters(
	OP_ParameterManager* manager)
{
	{
		OP_NumericParameter maxSubsteps;

		maxSubsteps.name = "Maxsubsteps";
		maxSubsteps.label = "Max Substeps";
		maxSubsteps.page = "Bullet";
		maxSubsteps.defaultValues[0] = 1.;
		maxSubsteps.minSliders[0] = 0.;
		maxSubsteps.maxSliders[0] = 10.;

		OP_ParAppendResult res = manager->appendInt(maxSubsteps);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter fixedTimeStep;

		fixedTimeStep.name = "Timeinc";
		fixedTimeStep.label = "Time Inc";
		fixedTimeStep.page = "Bullet";
		fixedTimeStep.defaultValues[0] = 1. / 60.;
		fixedTimeStep.minSliders[0] = 0.;
		fixedTimeStep.maxSliders[0] = 1.;

		OP_ParAppendResult res = manager->appendFloat(fixedTimeStep);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter gravity;

		gravity.name = "Gravity";
		gravity.label = "Gravity";
		gravity.page = "Bullet";
		gravity.defaultValues[0] = 0;
		gravity.defaultValues[1] = -9.8;
		gravity.defaultValues[2] = 0;

		OP_ParAppendResult res = manager->appendXYZ(gravity);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter syncInput;

		syncInput.name = "Syncinput";
		syncInput.label = "Sync Input";
		syncInput.page = "Bullet";
		syncInput.defaultValues[0] = 0;

		OP_ParAppendResult res = manager->appendToggle(syncInput);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter reset;

		reset.name = "Reset";
		reset.label = "Reset";
		reset.page = "Bullet";

		OP_ParAppendResult res = manager->appendPulse(reset);
		assert(res == OP_ParAppendResult::Success);
	}
}

void
BulletSOP::pulsePressed(char const* name)
{
	if (strcmp(name, "Reset") == 0)
	{
		_reset = true;
	}
}

btScalar
BulletSOP::_getDeltaTimeMicroseconds()
{
	btScalar dt = (btScalar)clock.getTimeMicroseconds();
	clock.reset();
	return dt;
}

void
BulletSOP::_worldSetup() 
{
	collisionConfiguration = new btDefaultCollisionConfiguration;
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	broadphase = new btDbvtBroadphase();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
	info.m_splitImpulse = 1;
	info.m_splitImpulsePenetrationThreshold = -0.02;

}

void 
BulletSOP::_worldDestroy()
{
	_objectsDestroy();

	delete dynamicsWorld;
	delete solver;
	delete broadphase;
	delete dispatcher;
	delete collisionConfiguration;
}

void
BulletSOP::_objectsDestroy()
{
	for (auto item : _objects)
	{
		dynamicsWorld->removeRigidBody(item.second->body);
		delete item.second;
	}

	_objects.clear();
	_objectIds.clear();
}

void
BulletSOP::_addSOPInput(OP_SOPInput const* sopInput)
{
	/*
	CustomAttribInfo const* massAttr = sopInput->getCustomAttribute("mass");
	CustomAttribInfo const* objIdAttr = sopInput->getCustomAttribute("objId");

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

	Position const* sopPoints = sopInput->getPointPositions();
	ColorInfo const* sopColors = sopInput->getColors();

	for (int p = 0; p < sopInput->getNumPrimitives(); ++p)
	{
		PrimitiveInfo const& primInfo = sopInput->getPrimitive(p);

		// XXX only point attrs are supported, assume constant
		int32_t firstIndex = primInfo.pointIndices[0];
		int32_t objId;// = objIdAttr->intData[firstIndex];

		// XXX hackity hack hack
		Color const& primColor = sopColors->colors[firstIndex];
		objId = static_cast<int>(primColor.r);

		_object* obj;
		auto objIt = _objects.find(objId);

		if (objIt == _objects.end())
		{
			obj = new _object();

			obj->id = objId;
			//obj->mass = massAttr->floatData[firstIndex];
			obj->mass = primColor.g; // XXX

			// 32-bit indices, 3 component verts
			obj->mesh = new btTriangleMesh(true, false);

			_objectIds.push_back(objId);
			_objects[objId] = obj;
		}
		else
		{
			obj = objIt->second;
		}

		switch (primInfo.type)
		{
		case PrimitiveType::Polygon: 
			if (primInfo.numVertices == 3)
			{
				int const& index0 = primInfo.pointIndices[0];
				int const& index1 = primInfo.pointIndices[1];
				int const& index2 = primInfo.pointIndices[2];

				obj->inputIndices.push_back(index0);
				obj->inputIndices.push_back(index1);
				obj->inputIndices.push_back(index2);

				Position const& position0 = sopPoints[index0];
				Position const& position1 = sopPoints[index1];
				Position const& position2 = sopPoints[index2];

				btVector3 vertex0(position0.x, position0.y, position0.z);
				btVector3 vertex1(position1.x, position1.y, position1.z);
				btVector3 vertex2(position2.x, position2.y, position2.z);

				// XXX disabling de-duplication here to support syncing with input. might be nice to make this optional
				obj->mesh->addTriangle(vertex0, vertex1, vertex2, false);
			}
			else
			{
				_errorString = "only triangular polygons are supported";
				continue;
			}
			break;


		default:
			_errorString = "only polygonal geometry is supported";
			continue;
		}
	}

	for (auto item : _objects)
	{
		_object* obj = item.second;
		btConvexTriangleMeshShape meshShape(obj->mesh);

		btVector3 primCenter;
		btScalar primRadius;
		meshShape.getBoundingSphere(primCenter, primRadius);

		_recenterMesh(obj->mesh, primCenter);

		bool dynamic = (obj->mass > 0.);
		btVector3 localInertia(0, 0, 0);

		if (dynamic)
		{
			// XXX prob need another btConvext...

			btShapeHull meshHull(&meshShape);
			meshHull.buildHull(meshShape.getMargin());

			btConvexHullShape *hullShape = new btConvexHullShape();
			btVector3 const* hullVertices = meshHull.getVertexPointer();

			for (int v = 0; v < meshHull.numVertices(); ++v)
			{
				hullShape->addPoint(hullVertices[v]);
			}

			obj->shape = hullShape;
			obj->shape->calculateLocalInertia(obj->mass, localInertia);
		}
		else
		{
			obj->shape = new btBvhTriangleMeshShape(obj->mesh, true);
		}

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(primCenter);

		obj->motionState = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(obj->mass, obj->motionState, obj->shape, localInertia);
		obj->body = new btRigidBody(rbInfo);

		if (!dynamic) {
			obj->body->setCollisionFlags(obj->body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			obj->body->setActivationState(DISABLE_DEACTIVATION);
		}

		dynamicsWorld->addRigidBody(obj->body);

	}
}

void
BulletSOP::_syncSOPInput(OP_SOPInput const* sopInput)
{
	btScalar minScalar = std::numeric_limits<btScalar>::min();
	btScalar maxScalar = std::numeric_limits<btScalar>::max();

	// XXX assumes input topology hasn't changed

	Position const* sopPoints = sopInput->getPointPositions();

	for (auto item : _objects)
	{
		_object* obj = item.second;

		// only sync static objects
		if (obj->mass > 0)
		{
			continue;
		}

		unsigned char* vertexBase;
		int numVerts;
		PHY_ScalarType type;
		int vertexStride;
		unsigned char* indexBase;
		int indexStride;
		int numFaces;
		PHY_ScalarType indicesType;

		btVector3 minVert(maxScalar, maxScalar, maxScalar);
		btVector3 maxVert(minScalar, minScalar, minScalar);

		for (int s = 0; s < obj->mesh->getNumSubParts(); ++s)
		{
			obj->mesh->getLockedVertexIndexBase(
				&vertexBase, numVerts, type, vertexStride,
				&indexBase, indexStride, numFaces, indicesType, s);

			for (int v = 0; v < obj->inputIndices.size(); ++v)
			{
				Position const& inputVert = sopPoints[obj->inputIndices[v]];
				double* vert = reinterpret_cast<double*>(vertexBase + v * vertexStride);

				vert[0] = inputVert.x;
				vert[1] = inputVert.y;
				vert[2] = inputVert.z;

				minVert[0] = std::min(minVert[0], static_cast<btScalar>(inputVert.x));
				minVert[1] = std::min(minVert[1], static_cast<btScalar>(inputVert.y));
				minVert[2] = std::min(minVert[2], static_cast<btScalar>(inputVert.z));

				maxVert[0] = std::max(maxVert[0], static_cast<btScalar>(inputVert.x));
				maxVert[1] = std::max(maxVert[1], static_cast<btScalar>(inputVert.y));
				maxVert[2] = std::max(maxVert[2], static_cast<btScalar>(inputVert.z));
			}

			obj->mesh->unLockVertexBase(s);
		}

		btVector3 center = (minVert + maxVert) / 2.;
		_recenterMesh(obj->mesh, center);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(center);

		obj->motionState->setWorldTransform(transform);
	}
}

void
BulletSOP::_recenterMesh(btTriangleMesh* mesh, btVector3 const& center)
{
	unsigned char* vertexBase;
	int numVerts;
	PHY_ScalarType type;
	int vertexStride;
	unsigned char* indexBase;
	int indexStride;
	int numFaces;
	PHY_ScalarType indicesType;

	for (int s = 0; s < mesh->getNumSubParts(); ++s)
	{
		mesh->getLockedVertexIndexBase(
			&vertexBase, numVerts, type, vertexStride,
			&indexBase, indexStride, numFaces, indicesType, s);

		for (int v = 0; v < numVerts; ++v)
		{
			double* vert = reinterpret_cast<double*>(vertexBase + v * vertexStride);

			vert[0] -= center.x();
			vert[1] -= center.y();
			vert[2] -= center.z();
		}

		mesh->unLockVertexBase(s);
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
