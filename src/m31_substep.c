// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "m31_joints.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"
#include "box3d/m31.h"

struct b3SubStepContext
{
	b3StepContext* step;
	int subStepIndex;
};

void b3World_SetSubStepCallback( b3WorldId worldId, b3SubStepFcn* fcn, void* userContext )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->subStepFcn = fcn;
	world->subStepContext = userContext;
}

void b3RunSubStepCallback( b3StepContext* context, int subStepIndex )
{
	b3World* world = context->world;
	b3SubStepContext subStep = { context, subStepIndex };
	world->subStepFcn( &subStep, world->subStepContext );
}

float b3SubStep_GetTimeStep( const b3SubStepContext* context )
{
	return context->step->h;
}

int b3SubStep_GetIndex( const b3SubStepContext* context )
{
	return context->subStepIndex;
}

int b3SubStep_GetCount( const b3SubStepContext* context )
{
	return context->step->subStepCount;
}

static int b3AwakeIndex( const b3SubStepContext* context, b3BodyId bodyId )
{
	b3World* world = context->step->world;
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->setIndex == b3_awakeSet ? body->localIndex : B3_NULL_INDEX;
}

bool b3SubStep_GetBodyVelocity( const b3SubStepContext* context, b3BodyId bodyId, b3Vec3* linearVelocity, b3Vec3* angularVelocity )
{
	int index = b3AwakeIndex( context, bodyId );
	if ( index == B3_NULL_INDEX )
	{
		*linearVelocity = b3Vec3_zero;
		*angularVelocity = b3Vec3_zero;
		return false;
	}

	b3BodyState* state = context->step->states + index;
	*linearVelocity = state->linearVelocity;
	*angularVelocity = state->angularVelocity;
	return true;
}

void b3SubStep_ApplyBodyImpulse( b3SubStepContext* context, b3BodyId bodyId, b3Vec3 linearImpulse, b3Vec3 angularImpulse )
{
	int index = b3AwakeIndex( context, bodyId );
	if ( index == B3_NULL_INDEX )
	{
		return;
	}

	b3BodyState* state = context->step->states + index;
	b3BodySim* sim = context->step->sims + index;
	state->linearVelocity = b3MulAdd( state->linearVelocity, sim->invMass, linearImpulse );
	state->angularVelocity = b3Add( state->angularVelocity, b3MulMV( sim->invInertiaWorld, angularImpulse ) );
}

static b3DirectionalContactJoint* b3SubStepJoint( const b3SubStepContext* context, b3JointId jointId )
{
	b3World* world = context->step->world;
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	B3_ASSERT( base->type == b3_directionalContactJoint );
	return &base->directionalContactJoint;
}

float b3SubStep_GetSpin( const b3SubStepContext* context, b3JointId jointId )
{
	return b3SubStepJoint( context, jointId )->spin;
}

void b3SubStep_SetSpin( b3SubStepContext* context, b3JointId jointId, float spin )
{
	b3SubStepJoint( context, jointId )->spin = spin;
}

float b3SubStep_GetLongitudinalImpulse( const b3SubStepContext* context, b3JointId jointId )
{
	return b3SubStepJoint( context, jointId )->longitudinalImpulse;
}

bool b3SubStep_HasContact( const b3SubStepContext* context, b3JointId jointId )
{
	return b3SubStepJoint( context, jointId )->hasContact;
}

void b3SubStep_GetSpinAndImpulse( const b3SubStepContext* context, b3JointId jointId, float* spin, float* impulse )
{
	b3DirectionalContactJoint* joint = b3SubStepJoint( context, jointId );
	*spin = joint->spin;
	*impulse = joint->hasContact ? joint->longitudinalImpulse : 0.0f;
}
