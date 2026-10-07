// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "m31_joints.h"
#include "math_internal.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"
#include "box3d/m31.h"

#include <stddef.h>

// Growing the union would add bytes to every joint of every type.
_Static_assert( sizeof( b3DirectionalContactJoint ) <= sizeof( b3SphericalJoint ), "directional contact joint grows b3JointSim" );

b3DirectionalContactJointDef b3DefaultDirectionalContactJointDef( void )
{
	b3DirectionalContactJointDef def = { 0 };
	def.base.localFrameA.q = b3Quat_identity;
	def.base.localFrameB.q = b3Quat_identity;
	def.base.forceThreshold = FLT_MAX;
	def.base.torqueThreshold = FLT_MAX;
	def.base.constraintHertz = 60.0f;
	def.base.constraintDampingRatio = 2.0f;
	def.base.drawScale = 1.0f;
	// The carrier must keep colliding with the ground it rolls on.
	def.base.collideConnected = true;
	def.base.internalValue = B3_SECRET_COOKIE;
	def.radius = 0.3f;
	def.minLength = 0.3f;
	def.maxLength = 0.5f;
	def.stiffness = 30000.0f;
	def.damping = 3000.0f;
	def.spinInertia = 1.0f;
	return def;
}

b3JointId b3CreateDirectionalContactJoint( b3WorldId worldId, const b3DirectionalContactJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( def->minLength <= def->maxLength );
	B3_ASSERT( def->radius > 0.0f );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return (b3JointId){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_directionalContactJoint );
	b3JointSim* base = pair.jointSim;

	base->directionalContactJoint = (b3DirectionalContactJoint){ 0 };
	b3DirectionalContactJoint* joint = &base->directionalContactJoint;
	joint->radius = def->radius;
	joint->minLength = def->minLength;
	joint->maxLength = def->maxLength;
	joint->preloadLength = def->preloadLength;
	joint->stiffness = def->stiffness;
	joint->damping = def->damping;
	joint->invSpinInertia = def->spinInertia > 0.0f ? 1.0f / def->spinInertia : 0.0f;
	joint->longitudinalFriction = 1.0f;
	joint->lateralFriction = 1.0f;
	joint->localNormalB = b3Vec3_axisY;
	joint->indexA = B3_NULL_INDEX;
	joint->indexB = B3_NULL_INDEX;

	return (b3JointId){ base->jointId + 1, world->worldId, pair.joint->generation };
}

static b3DirectionalContactJoint* b3GetDirectionalContactJoint( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_directionalContactJoint );
	return &base->directionalContactJoint;
}

void b3DirectionalContactJoint_SetContact( b3JointId jointId, b3Pos point, b3Vec3 normal )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_ASSERT( world->locked == false );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	B3_ASSERT( base->type == b3_directionalContactJoint );

	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );
	b3DirectionalContactJoint* dc = &base->directionalContactJoint;
	dc->localPointB = b3InvTransformWorldPoint( transformB, point );
	dc->localNormalB = b3InvRotateVector( transformB.q, normal );
	dc->hasContact = true;
}

void b3DirectionalContactJoint_ClearContact( b3JointId jointId )
{
	b3DirectionalContactJoint* joint = b3GetDirectionalContactJoint( jointId );
	joint->hasContact = false;
	joint->springImpulse = 0.0f;
	joint->bumpImpulse = 0.0f;
	joint->longitudinalImpulse = 0.0f;
	joint->lateralImpulse = 0.0f;
}

b3JointId b3DirectionalContactJoint_SetGround( b3JointId jointId, b3BodyId groundId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return jointId;
	}

	b3Joint* joint = b3GetJointFullId( world, jointId );
	int oldGround = joint->edges[1].bodyId;
	b3Body* newBody = b3GetBodyFullId( world, groundId );
	if ( newBody->id == oldGround )
	{
		return jointId;
	}

	b3Body* oldBody = b3Array_Get( world->bodies, oldGround );
	if ( oldBody->type == b3_staticBody && newBody->type == b3_staticBody )
	{
		// Static bodies carry no solver state; the contact point is stored through B's fixed transform.
		return jointId;
	}

	b3JointSim* base = b3GetJointSim( world, joint );
	b3DirectionalContactJoint saved = base->directionalContactJoint;

	b3JointDef def = { 0 };
	def.userData = joint->userData;
	def.bodyIdA = b3MakeBodyId( world, joint->edges[0].bodyId );
	def.bodyIdB = groundId;
	def.localFrameA = base->localFrameA;
	def.localFrameB = base->localFrameB;
	def.forceThreshold = base->forceThreshold;
	def.torqueThreshold = base->torqueThreshold;
	def.constraintHertz = base->constraintHertz;
	def.constraintDampingRatio = base->constraintDampingRatio;
	def.drawScale = joint->drawScale;
	def.collideConnected = joint->collideConnected;
	def.internalValue = B3_SECRET_COOKIE;

	b3DestroyJointInternal( world, joint, false );
	b3JointPair pair = b3CreateJoint( world, &def, b3_directionalContactJoint );

	saved.springImpulse = 0.0f;
	saved.bumpImpulse = 0.0f;
	saved.longitudinalImpulse = 0.0f;
	saved.lateralImpulse = 0.0f;
	saved.brakeImpulse = 0.0f;
	saved.hasContact = false;
	saved.indexA = B3_NULL_INDEX;
	saved.indexB = B3_NULL_INDEX;
	pair.jointSim->directionalContactJoint = saved;

	return (b3JointId){ pair.jointSim->jointId + 1, world->worldId, pair.joint->generation };
}

void b3DirectionalContactJoint_SetFriction( b3JointId jointId, float longitudinal, float lateral )
{
	B3_ASSERT( b3IsValidFloat( longitudinal ) && longitudinal >= 0.0f );
	B3_ASSERT( b3IsValidFloat( lateral ) && lateral >= 0.0f );
	b3DirectionalContactJoint* joint = b3GetDirectionalContactJoint( jointId );
	joint->longitudinalFriction = longitudinal;
	joint->lateralFriction = lateral;
}

void b3DirectionalContactJoint_SetSteerAngle( b3JointId jointId, float angle )
{
	b3GetDirectionalContactJoint( jointId )->steerAngle = angle;
}

void b3DirectionalContactJoint_SetBrakeTorque( b3JointId jointId, float torque )
{
	B3_ASSERT( b3IsValidFloat( torque ) && torque >= 0.0f );
	b3GetDirectionalContactJoint( jointId )->brakeTorque = torque;
}

void b3DirectionalContactJoint_SetSpin( b3JointId jointId, float spin )
{
	b3GetDirectionalContactJoint( jointId )->spin = spin;
}

float b3DirectionalContactJoint_GetSpin( b3JointId jointId )
{
	return b3GetDirectionalContactJoint( jointId )->spin;
}

b3DirectionalContactState b3DirectionalContactJoint_GetState( b3JointId jointId )
{
	b3DirectionalContactJoint* joint = b3GetDirectionalContactJoint( jointId );
	b3DirectionalContactState state = { 0 };
	state.hasContact = joint->hasContact;
	state.suspensionLength = joint->hasContact ? joint->length0 : joint->maxLength;
	state.spin = joint->spin;
	state.normalImpulse = joint->springImpulse + joint->bumpImpulse;
	state.longitudinalImpulse = joint->longitudinalImpulse;
	state.lateralImpulse = joint->lateralImpulse;
	state.brakeImpulse = joint->brakeImpulse;
	state.longitudinalAxis = joint->longitudinalAxis;
	state.lateralAxis = joint->lateralAxis;
	state.normal = joint->normal;
	return state;
}

static float b3RowMass( float mA, float mB, b3Matrix3 iA, b3Matrix3 iB, b3Vec3 rA, b3Vec3 rB, b3Vec3 axis, float extra )
{
	b3Vec3 sA = b3Cross( rA, axis );
	b3Vec3 sB = b3Cross( rB, axis );
	float k = mA + mB + b3Dot( sA, b3MulMV( iA, sA ) ) + b3Dot( sB, b3MulMV( iB, sB ) ) + extra;
	return k > 0.0f ? 1.0f / k : 0.0f;
}

void b3PrepareDirectionalContactJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_directionalContactJoint );

	b3World* world = context->world;
	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );

	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );
	b3BodySim* simA = b3Array_Get( setA->bodySims, bodyA->localIndex );
	b3BodySim* simB = b3Array_Get( setB->bodySims, bodyB->localIndex );

	base->invMassA = simA->invMass;
	base->invMassB = simB->invMass;
	base->invIA = simA->invInertiaWorld;
	base->invIB = simB->invInertiaWorld;
	base->fixedRotation = false;

	b3DirectionalContactJoint* joint = &base->directionalContactJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;

	if ( joint->hasContact == false )
	{
		return;
	}

	b3Quat qA = b3MulQuat( simA->transform.q, base->localFrameA.q );
	b3Matrix3 frame = b3MakeMatrixFromQuat( qA );
	b3Vec3 up = frame.cy;
	b3CosSin cs = b3ComputeCosSin( joint->steerAngle );
	b3Vec3 forward = b3Add( b3MulSV( cs.cosine, frame.cx ), b3MulSV( cs.sine, b3Cross( up, frame.cx ) ) );

	b3Pos hardPoint = b3TransformWorldPoint( simA->transform, base->localFrameA.p );
	b3Pos contact = b3TransformWorldPoint( simB->transform, joint->localPointB );
	b3Vec3 n = b3RotateVector( simB->transform.q, joint->localNormalB );
	b3Vec3 suspensionDir = b3Neg( up );

	joint->length0 = b3Dot( b3SubPos( contact, hardPoint ), suspensionDir ) - joint->radius;
	joint->normal = n;

	// Longitudinal axis in the plane of the contact normal and the wheel's forward direction.
	b3Vec3 axle = b3Cross( forward, up );
	b3Vec3 lon = b3Cross( n, axle );
	float lonLength = b3Length( lon );
	lon = lonLength > 1.0e-6f ? b3MulSV( 1.0f / lonLength, lon ) : b3Perp( n );
	if ( b3Dot( lon, forward ) < 0.0f )
	{
		lon = b3Neg( lon );
	}
	joint->longitudinalAxis = lon;
	joint->lateralAxis = b3Cross( lon, n );

	joint->rA = b3SubPos( contact, simA->center );
	joint->rB = b3SubPos( contact, simB->center );
	joint->deltaCenter = b3SubPos( simB->center, simA->center );

	float mA = base->invMassA, mB = base->invMassB;
	b3Matrix3 iA = base->invIA, iB = base->invIB;
	joint->normalMass = b3RowMass( mA, mB, iA, iB, joint->rA, joint->rB, n, 0.0f );
	joint->lateralMass = b3RowMass( mA, mB, iA, iB, joint->rA, joint->rB, joint->lateralAxis, 0.0f );
	joint->longitudinalMassLocked = b3RowMass( mA, mB, iA, iB, joint->rA, joint->rB, lon, 0.0f );
	joint->longitudinalMassFree =
		b3RowMass( mA, mB, iA, iB, joint->rA, joint->rB, lon, joint->radius * joint->radius * joint->invSpinInertia );

	// Spring along the contact normal, stiffness and damping divided by the angle between the suspension and
	// the normal so the component along the suspension is the authored spring.
	float cosAngle = b3MaxFloat( 0.1f, -b3Dot( suspensionDir, n ) );
	float k = joint->stiffness / cosAngle;
	float c = joint->damping / cosAngle;
	if ( k > 0.0f && joint->normalMass > 0.0f )
	{
		float omega = sqrtf( k / joint->normalMass );
		float zeta = 0.5f * c / sqrtf( k * joint->normalMass );
		joint->springSoftness = b3MakeSoft( omega / ( 2.0f * B3_PI ), zeta, context->h );
	}
	else
	{
		joint->springSoftness = (b3Softness){ 0.0f, 0.0f, 0.0f };
	}

	if ( context->enableWarmStarting == false )
	{
		joint->springImpulse = 0.0f;
		joint->bumpImpulse = 0.0f;
		joint->longitudinalImpulse = 0.0f;
		joint->lateralImpulse = 0.0f;
		joint->brakeImpulse = 0.0f;
	}
}

typedef struct b3DirectionalFrame
{
	b3Vec3 rA, rB, n, lon, lat;
} b3DirectionalFrame;

static b3DirectionalFrame b3CurrentFrame( const b3DirectionalContactJoint* joint, const b3BodyState* stateA,
										  const b3BodyState* stateB )
{
	b3DirectionalFrame f;
	f.rA = b3RotateVector( stateA->deltaRotation, joint->rA );
	f.rB = b3RotateVector( stateB->deltaRotation, joint->rB );
	f.n = b3RotateVector( stateB->deltaRotation, joint->normal );
	f.lon = b3RotateVector( stateA->deltaRotation, joint->longitudinalAxis );
	f.lat = b3RotateVector( stateA->deltaRotation, joint->lateralAxis );
	return f;
}

void b3WarmStartDirectionalContactJoint( b3JointSim* base, b3StepContext* context )
{
	b3DirectionalContactJoint* joint = &base->directionalContactJoint;
	if ( joint->hasContact == false )
	{
		return;
	}

	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3DirectionalFrame f = b3CurrentFrame( joint, stateA, stateB );

	b3Vec3 P = b3MulSV( joint->springImpulse + joint->bumpImpulse, f.n );
	P = b3MulAdd( P, joint->longitudinalImpulse, f.lon );
	P = b3MulAdd( P, joint->lateralImpulse, f.lat );

	stateA->linearVelocity = b3MulAdd( stateA->linearVelocity, base->invMassA, P );
	stateA->angularVelocity = b3Add( stateA->angularVelocity, b3MulMV( base->invIA, b3Cross( f.rA, P ) ) );
	stateB->linearVelocity = b3MulSub( stateB->linearVelocity, base->invMassB, P );
	stateB->angularVelocity = b3Sub( stateB->angularVelocity, b3MulMV( base->invIB, b3Cross( f.rB, P ) ) );

	joint->spin += joint->invSpinInertia * ( joint->brakeImpulse - joint->radius * joint->longitudinalImpulse );
}

void b3SolveDirectionalContactJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	b3DirectionalContactJoint* joint = &base->directionalContactJoint;
	if ( joint->hasContact == false )
	{
		return;
	}

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;
	float spin = joint->spin;

	b3DirectionalFrame f = b3CurrentFrame( joint, stateA, stateB );

	b3Vec3 d = b3Add( b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b3Sub( f.rB, f.rA ) );
	float length = joint->length0 - b3Dot( d, f.n );

	b3Vec3 snA = b3Cross( f.rA, f.n );
	b3Vec3 snB = b3Cross( f.rB, f.n );

	// Suspension spring. This is a real spring and is applied during relax as well.
	if ( joint->springSoftness.massScale > 0.0f )
	{
		float C = length - ( joint->maxLength + joint->preloadLength );
		float cdot = b3Dot( f.n, b3Sub( vA, vB ) ) + b3Dot( snA, wA ) - b3Dot( snB, wB );
		float impulse = -joint->springSoftness.massScale * joint->normalMass * ( cdot + joint->springSoftness.biasRate * C ) -
						joint->springSoftness.impulseScale * joint->springImpulse;
		float newImpulse = b3MaxFloat( joint->springImpulse + impulse, 0.0f );
		impulse = newImpulse - joint->springImpulse;
		joint->springImpulse = newImpulse;

		vA = b3MulAdd( vA, mA * impulse, f.n );
		wA = b3MulAdd( wA, impulse, b3MulMV( iA, snA ) );
		vB = b3MulSub( vB, mB * impulse, f.n );
		wB = b3MulSub( wB, impulse, b3MulMV( iB, snB ) );
	}

	// Bump stop at the minimum length.
	{
		float C = length - joint->minLength;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( C > 0.0f )
		{
			// speculative
			bias = C * context->inv_h;
		}
		else if ( useBias )
		{
			// A unilateral row keeps any separating velocity the bias gives it, so the push-out speed is
			// clamped as for contacts.
			bias = b3MaxFloat( base->constraintSoftness.biasRate * C, -context->world->contactSpeed );
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		float cdot = b3Dot( f.n, b3Sub( vA, vB ) ) + b3Dot( snA, wA ) - b3Dot( snB, wB );
		float impulse = -massScale * joint->normalMass * ( cdot + bias ) - impulseScale * joint->bumpImpulse;
		float newImpulse = b3MaxFloat( joint->bumpImpulse + impulse, 0.0f );
		impulse = newImpulse - joint->bumpImpulse;
		joint->bumpImpulse = newImpulse;

		vA = b3MulAdd( vA, mA * impulse, f.n );
		wA = b3MulAdd( wA, impulse, b3MulMV( iA, snA ) );
		vB = b3MulSub( vB, mB * impulse, f.n );
		wB = b3MulSub( wB, impulse, b3MulMV( iB, snB ) );
	}

	float normalImpulse = joint->springImpulse + joint->bumpImpulse;

	// Longitudinal row and brake row, solved as one block on the spin degree of freedom.
	{
		b3Vec3 slA = b3Cross( f.rA, f.lon );
		b3Vec3 slB = b3Cross( f.rB, f.lon );
		float vr = b3Dot( f.lon, b3Sub( vA, vB ) ) + b3Dot( slA, wA ) - b3Dot( slB, wB );
		float maxL = joint->longitudinalFriction * normalImpulse;
		float maxB = joint->brakeTorque * context->h;
		float invI = joint->invSpinInertia;
		float r = joint->radius;

		float oldL = joint->longitudinalImpulse;
		float newL;

		bool locked = false;
		if ( invI > 0.0f && maxB > 0.0f )
		{
			// Candidate: the brake holds the wheel at zero spin.
			float lockedL = b3ClampFloat( oldL - joint->longitudinalMassLocked * vr, -maxL, maxL );
			float dB = r * ( lockedL - oldL ) - spin / invI;
			float lockedB = joint->brakeImpulse + dB;
			if ( b3AbsFloat( lockedB ) <= maxB )
			{
				locked = true;
				newL = lockedL;
				joint->brakeImpulse = lockedB;
				spin = 0.0f;
			}
			else
			{
				// The brake slips at its capacity.
				float newB = b3ClampFloat( lockedB, -maxB, maxB );
				spin += invI * ( newB - joint->brakeImpulse );
				joint->brakeImpulse = newB;
			}
		}
		else
		{
			// Released brake: drop its accumulated impulse from the spin.
			spin -= invI * joint->brakeImpulse;
			joint->brakeImpulse = 0.0f;
		}

		if ( locked == false )
		{
			float cdot = vr - spin * r;
			float mass = invI > 0.0f ? joint->longitudinalMassFree : joint->longitudinalMassLocked;
			newL = b3ClampFloat( oldL - mass * cdot, -maxL, maxL );
			spin -= invI * r * ( newL - oldL );
		}

		float impulse = newL - oldL;
		joint->longitudinalImpulse = newL;

		vA = b3MulAdd( vA, mA * impulse, f.lon );
		wA = b3MulAdd( wA, impulse, b3MulMV( iA, slA ) );
		vB = b3MulSub( vB, mB * impulse, f.lon );
		wB = b3MulSub( wB, impulse, b3MulMV( iB, slB ) );
	}

	// Lateral row.
	{
		b3Vec3 stA = b3Cross( f.rA, f.lat );
		b3Vec3 stB = b3Cross( f.rB, f.lat );
		float cdot = b3Dot( f.lat, b3Sub( vA, vB ) ) + b3Dot( stA, wA ) - b3Dot( stB, wB );
		float maxT = joint->lateralFriction * normalImpulse;
		float oldT = joint->lateralImpulse;
		float newT = b3ClampFloat( oldT - joint->lateralMass * cdot, -maxT, maxT );
		float impulse = newT - oldT;
		joint->lateralImpulse = newT;

		vA = b3MulAdd( vA, mA * impulse, f.lat );
		wA = b3MulAdd( wA, impulse, b3MulMV( iA, stA ) );
		vB = b3MulSub( vB, mB * impulse, f.lat );
		wB = b3MulSub( wB, impulse, b3MulMV( iB, stB ) );
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
	joint->spin = spin;
}
