// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

// Suspension joint: a chassis (body A) and an unsprung body (body B: a hub or a solid axle).
//
// Frame A sits at the mount on the chassis: travel is along its y-axis, the vehicle's forward is its x-axis and the
// axle runs along its z-axis. Frame B is the matching point on the unsprung body.
// - Point-on-line: the frame B origin stays on frame A's y-axis (rows along x and z), as in the prismatic joint.
// - Independent wheel: every relative rotation is locked, as in the prismatic joint.
// - Solid axle: the x-axes stay collinear (rows about y and z, as in the revolute joint), so the axle rolls freely.
// - Spring seats: one at the origin (independent), or two at -/+ seatOffset along z (solid axle). Each acts along
//   the travel axis with a physical spring (N/m) and preload (N), bump and rebound damping (N s/m), and limits.
//
// travel = dot(pB - pA, axis) at a seat; positive is compression (the unsprung body rises toward the chassis).
// The spring pushes B along -axis by preload + stiffness * travel, as a soft row whose stiffness and damping are
// converted from physical units at the row's effective mass.

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

_Static_assert( sizeof( b3SuspensionJoint ) <= sizeof( b3SphericalJoint ), "suspension joint grows b3JointSim" );

b3SuspensionJointDef b3DefaultSuspensionJointDef( void )
{
	b3SuspensionJointDef def = { 0 };
	def.base.localFrameA.q = b3Quat_identity;
	def.base.localFrameB.q = b3Quat_identity;
	def.base.forceThreshold = FLT_MAX;
	def.base.torqueThreshold = FLT_MAX;
	def.base.constraintHertz = 60.0f;
	def.base.constraintDampingRatio = 2.0f;
	def.base.drawScale = 1.0f;
	def.base.internalValue = B3_SECRET_COOKIE;
	def.stiffness = 50000.0f;
	def.bumpDamping = 3000.0f;
	def.reboundDamping = 5000.0f;
	def.lowerTravel = -0.15f;
	def.upperTravel = 0.15f;
	return def;
}

b3JointId b3CreateSuspensionJoint( b3WorldId worldId, const b3SuspensionJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( def->lowerTravel <= def->upperTravel );
	B3_ASSERT( def->stiffness >= 0.0f && def->bumpDamping >= 0.0f && def->reboundDamping >= 0.0f );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return (b3JointId){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_suspensionJoint );
	b3JointSim* base = pair.jointSim;

	base->suspensionJoint = (b3SuspensionJoint){ 0 };
	b3SuspensionJoint* joint = &base->suspensionJoint;
	joint->stiffness = def->stiffness;
	joint->bumpDamping = def->bumpDamping;
	joint->reboundDamping = def->reboundDamping;
	joint->preload[0] = def->preload[0];
	joint->preload[1] = def->preload[1];
	joint->lowerTravel = def->lowerTravel;
	joint->upperTravel = def->upperTravel;
	joint->seatOffset = def->solidAxle ? def->seatOffset : 0.0f;
	joint->solidAxle = def->solidAxle;
	joint->indexA = B3_NULL_INDEX;
	joint->indexB = B3_NULL_INDEX;

	return (b3JointId){ base->jointId + 1, world->worldId, pair.joint->generation };
}

static b3SuspensionJoint* b3GetSuspensionJoint( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_suspensionJoint );
	return &base->suspensionJoint;
}

void b3SuspensionJoint_SetSpring( b3JointId jointId, float stiffness, float bumpDamping, float reboundDamping )
{
	B3_ASSERT( b3IsValidFloat( stiffness ) && stiffness >= 0.0f );
	B3_ASSERT( b3IsValidFloat( bumpDamping ) && bumpDamping >= 0.0f );
	B3_ASSERT( b3IsValidFloat( reboundDamping ) && reboundDamping >= 0.0f );
	b3SuspensionJoint* joint = b3GetSuspensionJoint( jointId );
	joint->stiffness = stiffness;
	joint->bumpDamping = bumpDamping;
	joint->reboundDamping = reboundDamping;
}

void b3SuspensionJoint_SetPreload( b3JointId jointId, float seat0, float seat1 )
{
	b3SuspensionJoint* joint = b3GetSuspensionJoint( jointId );
	joint->preload[0] = seat0;
	joint->preload[1] = seat1;
}

void b3SuspensionJoint_SetTravelLimits( b3JointId jointId, float lower, float upper )
{
	B3_ASSERT( lower <= upper );
	b3SuspensionJoint* joint = b3GetSuspensionJoint( jointId );
	joint->lowerTravel = lower;
	joint->upperTravel = upper;
}

static int b3SeatCount( const b3SuspensionJoint* joint )
{
	return joint->solidAxle ? 2 : 1;
}

static float b3SeatSide( int seat )
{
	return seat == 0 ? -1.0f : 1.0f;
}

// Seat points relative to each body's centre of mass in world, and the travel axis, from current transforms.
typedef struct b3SeatGeometry
{
	b3Vec3 axis;
	b3Vec3 d[2];
} b3SeatGeometry;

static b3SeatGeometry b3MeasureSeats( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );
	const b3SuspensionJoint* joint = &base->suspensionJoint;
	b3SeatGeometry g;
	b3Quat qA = b3MulQuat( transformA.q, base->localFrameA.q );
	g.axis = b3RotateVector( qA, b3Vec3_axisY );
	for ( int i = 0; i < b3SeatCount( joint ); ++i )
	{
		b3Vec3 offset = { 0.0f, 0.0f, b3SeatSide( i ) * joint->seatOffset };
		b3Pos pA = b3TransformWorldPoint( transformA, b3Add( base->localFrameA.p, b3RotateVector( base->localFrameA.q, offset ) ) );
		b3Pos pB = b3TransformWorldPoint( transformB, b3Add( base->localFrameB.p, b3RotateVector( base->localFrameB.q, offset ) ) );
		g.d[i] = b3SubPos( pB, pA );
	}
	return g;
}

float b3SuspensionJoint_GetTravel( b3JointId jointId, int seat )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	B3_ASSERT( base->type == b3_suspensionJoint );
	B3_ASSERT( 0 <= seat && seat < b3SeatCount( &base->suspensionJoint ) );
	b3SeatGeometry g = b3MeasureSeats( world, base );
	return b3Dot( g.d[seat], g.axis );
}

float b3SuspensionJoint_GetSpringForce( b3JointId jointId, int seat )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	B3_ASSERT( base->type == b3_suspensionJoint );
	B3_ASSERT( 0 <= seat && seat < b3SeatCount( &base->suspensionJoint ) );
	const b3SuspensionJoint* s = &base->suspensionJoint;
	return -world->inv_h * ( s->springImpulse[seat] + s->lowerImpulse[seat] - s->upperImpulse[seat] );
}

// Roll of a solid axle about the forward axis (rad, positive turns frame B's z-axis toward frame A's y-axis); zero for
// an independent wheel.
float b3SuspensionJoint_GetRoll( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	B3_ASSERT( base->type == b3_suspensionJoint );
	if ( base->suspensionJoint.solidAxle == false )
	{
		return 0.0f;
	}
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );
	b3Quat qA = b3MulQuat( transformA.q, base->localFrameA.q );
	b3Quat qB = b3MulQuat( transformB.q, base->localFrameB.q );
	b3Quat relQ = b3InvMulQuat( qA, qB );
	b3Vec3 axleB = b3RotateVector( relQ, b3Vec3_axisZ );
	return b3Atan2( axleB.y, axleB.z );
}

// The impulse the last sub-step applied to body B in joint space: x and z perpendicular, y along the travel axis.
static b3Vec3 b3SuspensionLinearImpulse( const b3SuspensionJoint* joint )
{
	float axial = 0.0f;
	for ( int i = 0; i < b3SeatCount( joint ); ++i )
	{
		axial += joint->springImpulse[i] + joint->lowerImpulse[i] - joint->upperImpulse[i];
	}
	return (b3Vec3){ joint->perpImpulse.x, axial, joint->perpImpulse.y };
}

b3Vec3 b3GetSuspensionJointForce( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3Vec3 impulse = b3SuspensionLinearImpulse( &base->suspensionJoint );
	b3Vec3 force = b3MulSV( world->inv_h, impulse );
	force = b3RotateVector( base->localFrameA.q, force );
	return b3RotateVector( transformA.q, force );
}

// The rotation lock's angular impulse in world: a vector for an independent wheel, the two collinearity rows' axes
// for a solid axle.
static b3Vec3 b3SuspensionAngularImpulse( const b3SuspensionJoint* joint )
{
	if ( joint->solidAxle )
	{
		return b3Blend2( joint->angularImpulse.y, joint->perpAxisY, joint->angularImpulse.z, joint->perpAxisZ );
	}
	return joint->angularImpulse;
}

// About frame B's origin: the locked rotations plus the moment of a solid axle's two seats.
b3Vec3 b3GetSuspensionJointTorque( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	const b3SuspensionJoint* joint = &base->suspensionJoint;
	b3Vec3 torque = b3MulSV( world->inv_h, b3SuspensionAngularImpulse( joint ) );
	if ( joint->solidAxle )
	{
		// seat i sits at (0, 0, side * offset) in frame A and pushes along y
		float roll = 0.0f;
		for ( int i = 0; i < 2; ++i )
		{
			float axial = joint->springImpulse[i] + joint->lowerImpulse[i] - joint->upperImpulse[i];
			roll += -b3SeatSide( i ) * joint->seatOffset * axial;
		}
		b3Vec3 rollAxis = b3RotateVector( b3MulQuat( transformA.q, base->localFrameA.q ), b3Vec3_axisX );
		torque = b3MulAdd( torque, world->inv_h * roll, rollAxis );
	}
	return torque;
}

float b3GetSuspensionJointLinearImpulse( const b3JointSim* base )
{
	return b3Length( b3SuspensionLinearImpulse( &base->suspensionJoint ) );
}

float b3GetSuspensionJointAngularImpulse( const b3JointSim* base )
{
	return b3Length( b3SuspensionAngularImpulse( &base->suspensionJoint ) );
}

float b3GetSuspensionJointLinearSeparation( b3World* world, b3JointSim* base )
{
	b3SeatGeometry g = b3MeasureSeats( world, base );
	const b3SuspensionJoint* joint = &base->suspensionJoint;
	// Off the travel axis at the origin, and past the travel limits.
	b3Vec3 d = joint->solidAxle ? b3MulSV( 0.5f, b3Add( g.d[0], g.d[1] ) ) : g.d[0];
	float along = b3Dot( d, g.axis );
	b3Vec3 perp = b3MulSub( d, along, g.axis );
	float beyond = 0.0f;
	for ( int i = 0; i < b3SeatCount( joint ); ++i )
	{
		float travel = b3Dot( g.d[i], g.axis );
		beyond = b3MaxFloat( beyond, b3MaxFloat( joint->lowerTravel - travel, travel - joint->upperTravel ) );
	}
	return sqrtf( b3Dot( perp, perp ) + beyond * beyond );
}

float b3GetSuspensionJointAngularSeparation( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );
	b3Quat qA = b3MulQuat( transformA.q, base->localFrameA.q );
	b3Quat qB = b3MulQuat( transformB.q, base->localFrameB.q );
	b3Quat relQ = b3InvMulQuat( qA, qB );
	// Twice the rotation quaternion's vector part: the angle for small errors. Roll is free on a solid axle.
	if ( base->suspensionJoint.solidAxle )
	{
		return 2.0f * sqrtf( relQ.v.y * relQ.v.y + relQ.v.z * relQ.v.z );
	}
	return 2.0f * b3Length( relQ.v );
}

static float b3RowMass( float mA, float mB, b3Matrix3 iA, b3Matrix3 iB, b3Vec3 sA, b3Vec3 sB )
{
	float k = mA + mB + b3Dot( sA, b3MulMV( iA, sA ) ) + b3Dot( sB, b3MulMV( iB, sB ) );
	return k > 0.0f ? 1.0f / k : 0.0f;
}

// exp(x) for 0 <= x, from + and *: a series on x / 8, squared three times.
static float b3ExpPositive( float x )
{
	float y = 0.125f * x;
	float e = 1.0f + y * ( 1.0f + y * ( 0.5f + y * ( 1.0f / 6.0f + y * ( 1.0f / 24.0f + y * ( 1.0f / 120.0f ) ) ) ) );
	e = e * e;
	e = e * e;
	return e * e;
}

b3Softness b3MakePhysicalSpring( float k, float c, float m, float h )
{
	if ( k <= 0.0f || m <= 0.0f || h <= 0.0f )
	{
		return (b3Softness){ 0.0f, 0.0f, 0.0f };
	}
	float omega = sqrtf( k / m );
	float zeta = 0.5f * c / sqrtf( k * m );
	float wh = omega * h;
	float zetaIn = b3MaxFloat( 0.0f, ( b3ExpPositive( 2.0f * zeta * wh ) - 1.0f - wh * wh ) / ( 2.0f * wh ) );
	return b3MakeSoft( omega / ( 2.0f * B3_PI ), zetaIn, h );
}

void b3PrepareSuspensionJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_suspensionJoint );

	b3World* world = context->world;
	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );
	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, bodyA->localIndex );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, bodyB->localIndex );

	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = bodySimA->invInertiaWorld;
	base->invIB = bodySimB->invInertiaWorld;
	b3Matrix3 invInertiaSum = b3AddMM( base->invIA, base->invIB );
	base->fixedRotation = b3Det( invInertiaSum ) < 1000.0f * FLT_MIN;

	b3SuspensionJoint* joint = &base->suspensionJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;

	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );
	joint->deltaCenter = b3SubPos( bodySimB->center, bodySimA->center );

	b3Vec3 axis = b3RotateVector( joint->frameA.q, b3Vec3_axisY );
	b3Vec3 seatAxis = b3RotateVector( joint->frameA.q, b3Vec3_axisZ );
	b3Vec3 seatAxisB = b3RotateVector( joint->frameB.q, b3Vec3_axisZ );
	float mA = base->invMassA, mB = base->invMassB;
	for ( int i = 0; i < b3SeatCount( joint ); ++i )
	{
		float offset = b3SeatSide( i ) * joint->seatOffset;
		b3Vec3 rA = b3MulAdd( joint->frameA.p, offset, seatAxis );
		b3Vec3 rB = b3MulAdd( joint->frameB.p, offset, seatAxisB );
		b3Vec3 d = b3Add( joint->deltaCenter, b3Sub( rB, rA ) );
		float m = b3RowMass( mA, mB, base->invIA, base->invIB, b3Cross( b3Add( rA, d ), axis ), b3Cross( rB, axis ) );
		joint->bumpSoftness[i] = b3MakePhysicalSpring( joint->stiffness, joint->bumpDamping, m, context->h );
		joint->reboundSoftness[i] = b3MakePhysicalSpring( joint->stiffness, joint->reboundDamping, m, context->h );
	}

	if ( joint->solidAxle )
	{
		b3Quat relQ = b3InvMulQuat( joint->frameA.q, joint->frameB.q );
		joint->perpAxisY =
			b3MulSV( 0.5f, b3RotateVector( joint->frameA.q, b3Add( b3MulSV( relQ.s, b3Vec3_axisY ), b3Cross( relQ.v, b3Vec3_axisY ) ) ) );
		joint->perpAxisZ =
			b3MulSV( 0.5f, b3RotateVector( joint->frameA.q, b3Add( b3MulSV( relQ.s, b3Vec3_axisZ ), b3Cross( relQ.v, b3Vec3_axisZ ) ) ) );
	}

	if ( context->enableWarmStarting == false )
	{
		joint->perpImpulse = (b3Vec2){ 0.0f, 0.0f };
		joint->angularImpulse = b3Vec3_zero;
		for ( int i = 0; i < 2; ++i )
		{
			joint->springImpulse[i] = 0.0f;
			joint->lowerImpulse[i] = 0.0f;
			joint->upperImpulse[i] = 0.0f;
		}
	}
}

// Sub-step geometry: anchors rotated by the bodies' sub-step rotations.
typedef struct b3SuspensionFrame
{
	b3Vec3 rA, rB, d;	  // joint origins and their separation
	b3Vec3 axis, perpX, perpZ, seatAxis, seatAxisB;
} b3SuspensionFrame;

static b3SuspensionFrame b3SuspensionCurrentFrame( const b3SuspensionJoint* joint, const b3BodyState* stateA, const b3BodyState* stateB )
{
	b3SuspensionFrame f;
	f.rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	f.rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
	f.d = b3Add( b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b3Sub( f.rB, f.rA ) );
	b3Quat qA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
	b3Quat qB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
	b3Matrix3 mA = b3MakeMatrixFromQuat( qA );
	f.perpX = mA.cx;
	f.axis = mA.cy;
	f.perpZ = mA.cz;
	f.seatAxis = mA.cz;
	f.seatAxisB = b3RotateVector( qB, b3Vec3_axisZ );
	return f;
}

// Seat i's anchors on A and B relative to the centres, and their separation.
static void b3SeatAnchors( const b3SuspensionJoint* joint, const b3SuspensionFrame* f, int seat, b3Vec3* rA, b3Vec3* rB, b3Vec3* d )
{
	float offset = b3SeatSide( seat ) * joint->seatOffset;
	*rA = b3MulAdd( f->rA, offset, f->seatAxis );
	*rB = b3MulAdd( f->rB, offset, f->seatAxisB );
	*d = b3Add( f->d, b3Sub( b3MulSV( offset, f->seatAxisB ), b3MulSV( offset, f->seatAxis ) ) );
}

void b3WarmStartSuspensionJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_suspensionJoint );
	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;
	b3SuspensionJoint* joint = &base->suspensionJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3SuspensionFrame f = b3SuspensionCurrentFrame( joint, stateA, stateB );

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	// Point-on-line rows at the origin.
	{
		b3Vec3 P = b3Blend2( joint->perpImpulse.x, f.perpX, joint->perpImpulse.y, f.perpZ );
		b3Vec3 LA = b3Cross( b3Add( f.rA, f.d ), P );
		b3Vec3 LB = b3Cross( f.rB, P );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, LA ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, LB ) );
	}

	// Seats.
	for ( int i = 0; i < b3SeatCount( joint ); ++i )
	{
		b3Vec3 rA, rB, d;
		b3SeatAnchors( joint, &f, i, &rA, &rB, &d );
		float axial = joint->springImpulse[i] + joint->lowerImpulse[i] - joint->upperImpulse[i];
		b3Vec3 P = b3MulSV( axial, f.axis );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3Cross( b3Add( rA, d ), P ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
	}

	// Rotation lock.
	{
		b3Vec3 L = b3SuspensionAngularImpulse( joint );
		wA = b3Sub( wA, b3MulMV( iA, L ) );
		wB = b3Add( wB, b3MulMV( iB, L ) );
	}

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}
	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

void b3SolveSuspensionJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_suspensionJoint );
	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;
	b3SuspensionJoint* joint = &base->suspensionJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3SuspensionFrame f = b3SuspensionCurrentFrame( joint, stateA, stateB );
	bool fixedRotation = base->fixedRotation;

	// Seats: spring and damper, then the travel limits. The spring runs once per sub-step, in the solve pass.
	if ( fixedRotation == false )
	{
		for ( int i = 0; i < b3SeatCount( joint ); ++i )
		{
			b3Vec3 rA, rB, d;
			b3SeatAnchors( joint, &f, i, &rA, &rB, &d );
			b3Vec3 sA = b3Cross( b3Add( rA, d ), f.axis );
			b3Vec3 sB = b3Cross( rB, f.axis );
			float travel = b3Dot( d, f.axis );
			float axialMass = b3RowMass( mA, mB, iA, iB, sA, sB );

			if ( joint->stiffness > 0.0f && useBias )
			{
				b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, b3Add( rA, d ) ) );
				float cdot = b3Dot( vRel, f.axis );
				// compression: the unsprung body rising toward the chassis
				b3Softness soft = cdot > 0.0f ? joint->bumpSoftness[i] : joint->reboundSoftness[i];
				float c = travel + joint->preload[i] / joint->stiffness;
				float impulse = -soft.massScale * axialMass * ( cdot + soft.biasRate * c ) - soft.impulseScale * joint->springImpulse[i];
				joint->springImpulse[i] += impulse;

				b3Vec3 P = b3MulSV( impulse, f.axis );
				vA = b3MulSub( vA, mA, P );
				wA = b3MulSub( wA, impulse, b3MulMV( iA, sA ) );
				vB = b3MulAdd( vB, mB, P );
				wB = b3MulAdd( wB, impulse, b3MulMV( iB, sB ) );
			}

			float speculativeDistance = 0.25f * ( joint->upperTravel - joint->lowerTravel );

			// Droop: travel >= lowerTravel
			{
				float C = travel - joint->lowerTravel;
				if ( C < speculativeDistance )
				{
					float bias = 0.0f, massScale = 1.0f, impulseScale = 0.0f;
					if ( C > 0.0f )
					{
						bias = C * context->inv_h;
					}
					else if ( useBias )
					{
						bias = base->constraintSoftness.biasRate * C;
						massScale = base->constraintSoftness.massScale;
						impulseScale = base->constraintSoftness.impulseScale;
					}
					b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, b3Add( rA, d ) ) );
					float cdot = b3Dot( vRel, f.axis );
					float oldImpulse = joint->lowerImpulse[i];
					float impulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * oldImpulse;
					joint->lowerImpulse[i] = b3MaxFloat( oldImpulse + impulse, 0.0f );
					impulse = joint->lowerImpulse[i] - oldImpulse;

					b3Vec3 P = b3MulSV( impulse, f.axis );
					vA = b3MulSub( vA, mA, P );
					wA = b3MulSub( wA, impulse, b3MulMV( iA, sA ) );
					vB = b3MulAdd( vB, mB, P );
					wB = b3MulAdd( wB, impulse, b3MulMV( iB, sB ) );
				}
				else
				{
					joint->lowerImpulse[i] = 0.0f;
				}
			}

			// Bump stop: travel <= upperTravel
			{
				float C = joint->upperTravel - travel;
				if ( C < speculativeDistance )
				{
					float bias = 0.0f, massScale = 1.0f, impulseScale = 0.0f;
					if ( C > 0.0f )
					{
						bias = C * context->inv_h;
					}
					else if ( useBias )
					{
						bias = base->constraintSoftness.biasRate * C;
						massScale = base->constraintSoftness.massScale;
						impulseScale = base->constraintSoftness.impulseScale;
					}
					b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, b3Add( rA, d ) ) );
					float cdot = -b3Dot( vRel, f.axis );
					float oldImpulse = joint->upperImpulse[i];
					float impulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * oldImpulse;
					joint->upperImpulse[i] = b3MaxFloat( oldImpulse + impulse, 0.0f );
					float negImpulse = oldImpulse - joint->upperImpulse[i];

					b3Vec3 P = b3MulSV( negImpulse, f.axis );
					vA = b3MulSub( vA, mA, P );
					wA = b3MulSub( wA, negImpulse, b3MulMV( iA, sA ) );
					vB = b3MulAdd( vB, mB, P );
					wB = b3MulAdd( wB, negImpulse, b3MulMV( iB, sB ) );
				}
				else
				{
					joint->upperImpulse[i] = 0.0f;
				}
			}
		}
	}

	// Rotation lock.
	if ( fixedRotation == false )
	{
		b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
		b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
		b3Quat relQ = b3InvMulQuat( quatA, quatB );
		b3Matrix3 invInertiaSum = b3AddMM( iA, iB );

		if ( joint->solidAxle )
		{
			// The x-axes stay collinear: rows on the relative quaternion's y and z, as the revolute joint's on x and y.
			b3Vec2 bias = { 0.0f, 0.0f };
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				b3Vec2 c = { relQ.v.y, relQ.v.z };
				bias = b3MulSV2( base->constraintSoftness.biasRate, c );
				massScale = base->constraintSoftness.massScale;
				impulseScale = base->constraintSoftness.impulseScale;
			}

			b3Vec3 perpY = b3MulSV( 0.5f, b3RotateVector( quatA, b3Add( b3MulSV( relQ.s, b3Vec3_axisY ), b3Cross( relQ.v, b3Vec3_axisY ) ) ) );
			b3Vec3 perpZ = b3MulSV( 0.5f, b3RotateVector( quatA, b3Add( b3MulSV( relQ.s, b3Vec3_axisZ ), b3Cross( relQ.v, b3Vec3_axisZ ) ) ) );
			joint->perpAxisY = perpY;
			joint->perpAxisZ = perpZ;

			float kyy = b3Dot( perpY, b3MulMV( invInertiaSum, perpY ) );
			float kzz = b3Dot( perpZ, b3MulMV( invInertiaSum, perpZ ) );
			float kyz = b3Dot( perpY, b3MulMV( invInertiaSum, perpZ ) );
			b3Matrix2 k = { { kyy, kyz }, { kyz, kzz } };

			b3Vec3 wRel = b3Sub( wB, wA );
			b3Vec2 cdot = { b3Dot( wRel, perpY ), b3Dot( wRel, perpZ ) };
			b3Vec2 oldImpulse = { joint->angularImpulse.y, joint->angularImpulse.z };
			b3Vec2 sol = b3Solve2( k, b3Add2( cdot, bias ) );
			b3Vec2 impulse = b3Sub2( b3MulSV2( -massScale, sol ), b3MulSV2( impulseScale, oldImpulse ) );
			joint->angularImpulse.y += impulse.x;
			joint->angularImpulse.z += impulse.y;

			b3Vec3 L = b3Blend2( impulse.x, perpY, impulse.y, perpZ );
			wA = b3Sub( wA, b3MulMV( iA, L ) );
			wB = b3Add( wB, b3MulMV( iB, L ) );
		}
		else
		{
			b3Vec3 bias = b3Vec3_zero;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, b3Quat_identity );
				b3Vec3 c = b3Neg( b3RotateVector( quatA, deltaRotation ) );
				bias = b3MulSV( base->constraintSoftness.biasRate, c );
				massScale = base->constraintSoftness.massScale;
				impulseScale = base->constraintSoftness.impulseScale;
			}

			b3Matrix3 rotationMass = b3InvertMatrix( invInertiaSum );
			b3Vec3 cdot = b3Sub( wB, wA );
			b3Vec3 impulse = b3Sub( b3MulSV( -massScale, b3MulMV( rotationMass, b3Add( cdot, bias ) ) ),
									b3MulSV( impulseScale, joint->angularImpulse ) );
			joint->angularImpulse = b3Add( joint->angularImpulse, impulse );

			wA = b3Sub( wA, b3MulMV( iA, impulse ) );
			wB = b3Add( wB, b3MulMV( iB, impulse ) );
		}
	}

	// Point-on-line at the origin.
	{
		b3Vec2 bias = { 0.0f, 0.0f };
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b3Vec2 c = { b3Dot( f.perpX, f.d ), b3Dot( f.perpZ, f.d ) };
			bias = b3MulSV2( base->constraintSoftness.biasRate, c );
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, f.rB ) ), vA ), b3Cross( wA, b3Add( f.rA, f.d ) ) );
		b3Vec2 cdot = { b3Dot( f.perpX, vRel ), b3Dot( f.perpZ, vRel ) };

		b3Vec3 sAx = b3Cross( b3Add( f.rA, f.d ), f.perpX );
		b3Vec3 sBx = b3Cross( f.rB, f.perpX );
		b3Vec3 sAz = b3Cross( b3Add( f.rA, f.d ), f.perpZ );
		b3Vec3 sBz = b3Cross( f.rB, f.perpZ );

		float kxx = mA + mB + b3Dot( sAx, b3MulMV( iA, sAx ) ) + b3Dot( sBx, b3MulMV( iB, sBx ) );
		float kxz = b3Dot( sAx, b3MulMV( iA, sAz ) ) + b3Dot( sBx, b3MulMV( iB, sBz ) );
		float kzz = mA + mB + b3Dot( sAz, b3MulMV( iA, sAz ) ) + b3Dot( sBz, b3MulMV( iB, sBz ) );
		b3Matrix2 K = { { kxx, kxz }, { kxz, kzz } };

		b3Vec2 oldImpulse = joint->perpImpulse;
		b3Vec2 sol = b3Solve2( K, b3Add2( cdot, bias ) );
		b3Vec2 impulse = b3Sub2( b3MulSV2( -massScale, sol ), b3MulSV2( impulseScale, oldImpulse ) );
		joint->perpImpulse = b3Add2( oldImpulse, impulse );

		b3Vec3 P = b3Blend2( impulse.x, f.perpX, impulse.y, f.perpZ );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3Blend2( impulse.x, sAx, impulse.y, sAz ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3Blend2( impulse.x, sBx, impulse.y, sBz ) ) );
	}

	B3_ASSERT( b3IsValidVec3( vA ) );
	B3_ASSERT( b3IsValidVec3( wA ) );
	B3_ASSERT( b3IsValidVec3( vB ) );
	B3_ASSERT( b3IsValidVec3( wB ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}
	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

void b3DrawSuspensionJoint( b3DebugDraw* draw, b3JointSim* base, b3WorldTransform transformA, b3WorldTransform transformB, float scale )
{
	B3_UNUSED( scale );
	b3WorldTransform frameA = b3MulWorldTransforms( transformA, base->localFrameA );
	b3WorldTransform frameB = b3MulWorldTransforms( transformB, base->localFrameB );
	b3Matrix3 R = b3MakeMatrixFromQuat( frameA.q );
	const b3SuspensionJoint* joint = &base->suspensionJoint;

	b3Pos p1 = b3OffsetPos( frameA.p, b3MulSV( joint->lowerTravel, R.cy ) );
	b3Pos p2 = b3OffsetPos( frameA.p, b3MulSV( joint->upperTravel, R.cy ) );
	draw->DrawSegmentFcn( p1, p2, b3_colorOrange, draw->context );
	draw->DrawPointFcn( p1, 8.0f, b3_colorGreen, draw->context );
	draw->DrawPointFcn( p2, 8.0f, b3_colorRed, draw->context );
	if ( joint->solidAxle )
	{
		b3Vec3 seat = b3MulSV( joint->seatOffset, b3RotateVector( frameB.q, b3Vec3_axisZ ) );
		draw->DrawSegmentFcn( b3OffsetPos( frameB.p, b3Neg( seat ) ), b3OffsetPos( frameB.p, seat ), b3_colorViolet, draw->context );
	}
	draw->DrawPointFcn( frameB.p, 8.0f, b3_colorViolet, draw->context );
}
