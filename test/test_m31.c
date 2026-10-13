// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

// Tests of the fork additions (include/box3d/m31.h).

#include "arena_allocator.h"
#include "container.h"
#include "core.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/m31.h"

#include <limits.h>
#include <setjmp.h>
#include <stdlib.h>

// ---- Memory ---------------------------------------------------------------------------------------------------------

#define GIB ( (size_t)1 << 30 )
#define MIB ( (size_t)1 << 20 )

// Sizes at or past this are handed a placeholder that is never touched, so the arithmetic past 2 GiB is checked
// without the memory. Smaller requests are real.
#define PLACEHOLDER_THRESHOLD GIB

static _Alignas( 64 ) char s_placeholder[64];
static size_t s_lastHugeAlloc;
static size_t s_lastHugeFree;
static size_t s_failSize;

static void* RecordingAlloc( size_t size, int alignment )
{
	if ( size == s_failSize )
	{
		return NULL;
	}
	if ( size >= PLACEHOLDER_THRESHOLD )
	{
		s_lastHugeAlloc = size;
		return s_placeholder;
	}
	return aligned_alloc( (size_t)alignment, size );
}

static void RecordingFree( void* mem, size_t size )
{
	if ( mem == s_placeholder )
	{
		s_lastHugeFree = size;
		return;
	}
	free( mem );
}

static jmp_buf s_outOfMemoryJump;
static size_t s_outOfMemorySize;

static void CatchOutOfMemory( size_t size, int64_t byteCount )
{
	(void)byteCount;
	s_outOfMemorySize = size;
	longjmp( s_outOfMemoryJump, 1 );
}

typedef struct Megabyte
{
	char bytes[1 << 20];
} Megabyte;

b3DeclareArray( Megabyte );

static int CapacityGrowsInSixtyFourBits( void )
{
	ENSURE( b3ClampCapacity( 16, 9, 4 ) == 16 );
	ENSURE( b3ClampCapacity( 2 * (int64_t)( INT_MAX / 2 + 1 ), INT_MAX / 2 + 2, 4 ) == INT_MAX );

	b3SetOutOfMemoryFcn( CatchOutOfMemory );
	s_outOfMemorySize = 0;
	if ( setjmp( s_outOfMemoryJump ) == 0 )
	{
		b3ClampCapacity( 2 * (int64_t)INT_MAX, (int64_t)INT_MAX + 1, 8 );
		b3SetOutOfMemoryFcn( NULL );
		return 1;
	}
	b3SetOutOfMemoryFcn( NULL );
	ENSURE( s_outOfMemorySize == ( (size_t)INT_MAX + 1 ) * 8 );
	return 0;
}

// Upstream held these byte sizes in int: 4096 one-mebibyte elements wrapped to 0, a 3 GiB stack entry to -1 GiB.
static int ByteSizesPassTwoGibibytes( void )
{
	b3SetAllocator( RecordingAlloc, RecordingFree );
	s_failSize = 0;

	b3Array( Megabyte ) a = { 0 };
	b3Array_Reserve( a, 4096 );
	ENSURE( s_lastHugeAlloc == 4 * GIB );
	ENSURE( a.capacity == 4096 );
	b3Array_Destroy( a );
	ENSURE( s_lastHugeFree == 4 * GIB );

	b3Stack stack = b3CreateStack( 1024 );
	void* big = b3StackAlloc( &stack, 3 * GIB, "big" );
	ENSURE( big == s_placeholder );
	ENSURE( s_lastHugeAlloc == 3 * GIB );
	ENSURE( b3GetMaxStackAllocation( &stack ) == (int64_t)( 3 * GIB ) );
	b3StackFree( &stack, big );
	ENSURE( s_lastHugeFree == 3 * GIB );
	b3DestroyStack( &stack );

	b3SetAllocator( NULL, NULL );
	return 0;
}

static int FailedAllocationIsReported( void )
{
	b3SetAllocator( RecordingAlloc, RecordingFree );
	b3SetOutOfMemoryFcn( CatchOutOfMemory );
	s_failSize = 5 * MIB;
	s_outOfMemorySize = 0;
	if ( setjmp( s_outOfMemoryJump ) == 0 )
	{
		b3Alloc( 5 * MIB );
		b3SetOutOfMemoryFcn( NULL );
		b3SetAllocator( NULL, NULL );
		return 1;
	}
	s_failSize = 0;
	b3SetOutOfMemoryFcn( NULL );
	b3SetAllocator( NULL, NULL );
	ENSURE( s_outOfMemorySize == 5 * MIB );
	return 0;
}

// ---- Contact capacity -------------------------------------------------------------------------------------------------

#define PILE_SIDE 4
#define PILE_LAYERS 4

// Spheres packed closer than their diameter, so every neighbour pair overlaps from the first step.
static b3WorldId CreateOverlappingPile( b3BodyId* bodies )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull floor = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3CreateHullShape( groundId, &shapeDef, &floor.base );

	b3Sphere sphere = { b3Vec3_zero, 0.5f };
	int n = 0;
	for ( int y = 0; y < PILE_LAYERS; ++y )
	{
		for ( int z = 0; z < PILE_SIDE; ++z )
		{
			for ( int x = 0; x < PILE_SIDE; ++x )
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = ( b3Pos ){ 0.8f * x, 1.5f + 0.8f * y, 0.8f * z };
				bodies[n] = b3CreateBody( worldId, &bodyDef );
				b3CreateSphereShape( bodies[n], &shapeDef, &sphere );
				n += 1;
			}
		}
	}
	return worldId;
}

static int ContactCapacityRefusesAndCounts( void )
{
	enum
	{
		bodyCount = PILE_SIDE * PILE_SIDE * PILE_LAYERS,
		capacity = 10,
		steps = 60,
	};
	b3BodyId bodies[bodyCount];
	b3Pos finalPositions[2][bodyCount];

	for ( int run = 0; run < 2; ++run )
	{
		b3WorldId worldId = CreateOverlappingPile( bodies );
		ENSURE( b3World_GetContactCapacity( worldId ) == INT_MAX );
		b3World_SetContactCapacity( worldId, capacity );

		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( b3World_GetCounters( worldId ).contactCount == capacity );
		int firstRefused = b3World_GetRefusedContactCount( worldId );
		ENSURE( firstRefused > 0 );

		for ( int i = 1; i < steps; ++i )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );
			ENSURE( b3World_GetCounters( worldId ).contactCount <= capacity );
		}

		for ( int i = 0; i < bodyCount; ++i )
		{
			finalPositions[run][i] = b3Body_GetPosition( bodies[i] );
		}

		// Moving proxies find their refused pairs again once there is room.
		b3World_SetContactCapacity( worldId, INT_MAX );
		for ( int i = 0; i < steps; ++i )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );
		}
		ENSURE( b3World_GetRefusedContactCount( worldId ) == 0 );
		ENSURE( b3World_GetCounters( worldId ).contactCount > capacity );

		b3DestroyWorld( worldId );
	}

	// The same pairs are refused in both runs.
	ENSURE( memcmp( finalPositions[0], finalPositions[1], sizeof( finalPositions[0] ) ) == 0 );
	return 0;
}

// ---- Directional contact joint ----------------------------------------------------------------------------------------

#define WHEEL_COUNT 4
#define CHASSIS_MASS 100.0f
#define GRAVITY 10.0f
#define WHEEL_RADIUS 0.3f
#define SETTLE_STEPS 180

typedef struct Car
{
	b3WorldId worldId;
	b3BodyId groundIds[2];
	b3BodyId chassisId;
	b3JointId wheels[WHEEL_COUNT];
	b3Vec3 hardPoints[WHEEL_COUNT];
} Car;

// A box chassis on four sprung wheels over a static ground of two slabs meeting at x = 0, starting at x = -3.
static Car CreateCar( void )
{
	Car car = { 0 };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -GRAVITY, 0.0f };
	worldDef.enableSleep = false;
	car.worldId = b3CreateWorld( &worldDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	for ( int i = 0; i < 2; ++i )
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.position = ( b3Pos ){ i == 0 ? -10.0f : 10.0f, -0.5f, 0.0f };
		car.groundIds[i] = b3CreateBody( car.worldId, &groundDef );
		b3BoxHull slab = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
		b3CreateHullShape( car.groundIds[i], &shapeDef, &slab.base );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ -3.0f, 1.0f, 0.0f };
	car.chassisId = b3CreateBody( car.worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 1.5f, 0.25f, 1.0f );
	b3ShapeDef chassisShape = b3DefaultShapeDef();
	chassisShape.density = CHASSIS_MASS / ( 3.0f * 0.5f * 2.0f );
	chassisShape.filter.maskBits = 0;
	b3CreateHullShape( car.chassisId, &chassisShape, &box.base );

	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		car.hardPoints[i] = ( b3Vec3 ){ ( i & 1 ) ? 1.2f : -1.2f, 0.0f, ( i & 2 ) ? 0.8f : -0.8f };
		b3DirectionalContactJointDef def = b3DefaultDirectionalContactJointDef();
		def.base.bodyIdA = car.chassisId;
		def.base.bodyIdB = car.groundIds[0];
		def.base.localFrameA.p = car.hardPoints[i];
		def.radius = WHEEL_RADIUS;
		def.minLength = 0.1f;
		def.maxLength = 0.6f;
		def.stiffness = 8000.0f;
		def.damping = 1500.0f;
		car.wheels[i] = b3CreateDirectionalContactJoint( car.worldId, &def );
	}
	return car;
}

// The probe: the ground under each hard point, on whichever slab that is, with the brakes on or off.
static void ProbeWheels( Car* car, float brakeTorque, bool retarget )
{
	b3WorldTransform xf = b3Body_GetTransform( car->chassisId );
	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		b3Pos hardPoint = b3TransformWorldPoint( xf, car->hardPoints[i] );
		if ( retarget )
		{
			b3DirectionalContactJoint_SetGround( car->wheels[i], car->groundIds[hardPoint.x < 0.0f ? 0 : 1] );
		}
		b3DirectionalContactInput input = { 0 };
		input.hasContact = true;
		input.point = ( b3Pos ){ hardPoint.x, 0.0f, hardPoint.z };
		input.normal = b3Vec3_axisY;
		input.longitudinalFriction = 1.0f;
		input.lateralFriction = 1.0f;
		input.brakeTorque = brakeTorque;
		b3DirectionalContactJoint_SetInput( car->wheels[i], &input );
	}
}

static b3Vec3 GroundForce( const Car* car )
{
	b3Vec3 sum = b3Vec3_zero;
	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		sum = b3Add( sum, b3Joint_GetConstraintForce( car->wheels[i] ) );
	}
	return sum;
}

static int ContactJointReportsTheGroundForce( void )
{
	Car car = CreateCar();
	const float brake = 1.0e4f;
	for ( int i = 0; i < SETTLE_STEPS; ++i )
	{
		ProbeWheels( &car, brake, false );
		b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
	}

	// At rest the wheels press the weight into the ground, at the contact points.
	float weight = CHASSIS_MASS * GRAVITY;
	b3Vec3 force = GroundForce( &car );
	ENSURE_SMALL( force.y + weight, 0.005f * weight );
	ENSURE_SMALL( force.x, 0.005f * weight );
	ENSURE_SMALL( force.z, 0.005f * weight );
	b3WorldTransform xf = b3Body_GetTransform( car.chassisId );
	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		ENSURE( b3Length( b3Joint_GetConstraintTorque( car.wheels[i] ) ) == 0.0f );
		b3Vec3 anchor = b3Joint_GetLocalFrameB( car.wheels[i] ).p;
		b3Pos hardPoint = b3TransformWorldPoint( xf, car.hardPoints[i] );
		ENSURE_SMALL( anchor.x - (float)( hardPoint.x + 10.0 ), 1.0e-5f );
		ENSURE_SMALL( anchor.y - 0.5f, 1.0e-5f );
	}

	// Pushed forward and sideways with the brakes on, both friction rows carry the push into the ground.
	const float push = 300.0f;
	for ( int i = 0; i < 60; ++i )
	{
		ProbeWheels( &car, brake, false );
		b3Body_ApplyForceToCenter( car.chassisId, ( b3Vec3 ){ push, 0.0f, 0.5f * push }, true );
		b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
	}
	force = GroundForce( &car );
	ENSURE_SMALL( force.x - push, 0.01f * push );
	ENSURE_SMALL( force.z - 0.5f * push, 0.01f * push );
	ENSURE_SMALL( force.y + weight, 0.01f * weight );

	// A threshold under one wheel's share reports every wheel as overloaded.
	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		b3Joint_SetForceThreshold( car.wheels[i], 0.2f * weight );
	}
	ProbeWheels( &car, brake, false );
	b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3World_GetJointEvents( car.worldId ).count == WHEEL_COUNT );

	b3DestroyWorld( car.worldId );
	return 0;
}

// Rolled across the seam between the slabs, each wheel moves to the slab under it and keeps its id. The warm start
// carries over, so the chassis does not feel the seams: re-created joints (impulses zeroed) gave a 4e-4 support dip and
// kicked the chassis 7.7e-5 m/s up and 1.8e-4 rad/s in pitch; carried, 0 and 1.2e-6 m/s.
#define SEAM_SUPPORT_TOLERANCE 1.0e-4f
#define SEAM_VERTICAL_SPEED 1.0e-5f

static int ContactJointKeepsItsIdAcrossGrounds( void )
{
	Car car = CreateCar();
	for ( int i = 0; i < SETTLE_STEPS; ++i )
	{
		ProbeWheels( &car, 1.0e4f, true );
		b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
	}

	b3JointId ids[WHEEL_COUNT];
	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		ids[i] = car.wheels[i];
		ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( car.wheels[i] ), car.groundIds[0] ) );
	}

	// Without a new probe the contact keeps its world position on the new ground.
	{
		b3WorldTransform from = b3Body_GetTransform( car.groundIds[0] );
		b3WorldTransform to = b3Body_GetTransform( car.groundIds[1] );
		b3Pos before = b3TransformWorldPoint( from, b3Joint_GetLocalFrameB( car.wheels[0] ).p );
		b3DirectionalContactJoint_SetGround( car.wheels[0], car.groundIds[1] );
		ENSURE( B3_ID_EQUALS( car.wheels[0], ids[0] ) );
		ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( car.wheels[0] ), car.groundIds[1] ) );
		b3Pos after = b3TransformWorldPoint( to, b3Joint_GetLocalFrameB( car.wheels[0] ).p );
		ENSURE_SMALL( (float)( after.x - before.x ), 1.0e-5f );
		ENSURE_SMALL( (float)( after.y - before.y ), 1.0e-5f );
		ENSURE( b3DirectionalContactJoint_GetState( car.wheels[0] ).hasContact );
		b3DirectionalContactJoint_SetGround( car.wheels[0], car.groundIds[0] );
	}

	float weight = CHASSIS_MASS * GRAVITY;
	float worst = 0.0f;
	float worstVy = 0.0f;
	for ( int i = 0; i < 240; ++i )
	{
		ProbeWheels( &car, 0.0f, true );
		b3Body_ApplyForceToCenter( car.chassisId, ( b3Vec3 ){ 120.0f, 0.0f, 0.0f }, true );
		b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
		float deviation = b3AbsFloat( GroundForce( &car ).y + weight ) / weight;
		float x = (float)b3Body_GetPosition( car.chassisId ).x;
		float vy = b3AbsFloat( b3Body_GetLinearVelocity( car.chassisId ).y );
		bool nearSeam = b3AbsFloat( x + 1.2f ) < 0.3f || b3AbsFloat( x - 1.2f ) < 0.3f;
		if ( nearSeam )
		{
			worst = b3MaxFloat( worst, deviation );
			worstVy = b3MaxFloat( worstVy, vy );
		}
	}

	ENSURE( b3Body_GetPosition( car.chassisId ).x > 2.0f );
	ENSURE( b3World_GetCounters( car.worldId ).jointCount == WHEEL_COUNT );
	for ( int i = 0; i < WHEEL_COUNT; ++i )
	{
		ENSURE( B3_ID_EQUALS( car.wheels[i], ids[i] ) );
		ENSURE( b3Joint_IsValid( ids[i] ) );
		ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( ids[i] ), car.groundIds[1] ) );
	}
	ENSURE( worst < SEAM_SUPPORT_TOLERANCE );
	ENSURE( worstVy < SEAM_VERTICAL_SPEED );

	b3DestroyWorld( car.worldId );
	return 0;
}

// Re-targeting onto a sleeping dynamic body wakes it and joins the carrier's island; back onto static ground the
// joint leaves it. Validation builds check the solver sets after every move.
static int ContactJointMovesBetweenStaticAndDynamicGround( void )
{
	Car car = CreateCar();
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BodyDef plankDef = b3DefaultBodyDef();
	plankDef.type = b3_dynamicBody;
	plankDef.position = ( b3Pos ){ 8.0f, 0.25f, 0.0f };
	b3BodyId plankId = b3CreateBody( car.worldId, &plankDef );
	b3BoxHull plank = b3MakeBoxHull( 1.0f, 0.25f, 1.0f );
	b3CreateHullShape( plankId, &shapeDef, &plank.base );
	b3Body_SetAwake( plankId, false );
	ENSURE( b3Body_IsAwake( plankId ) == false );

	for ( int i = 0; i < 30; ++i )
	{
		ProbeWheels( &car, 1.0e4f, false );
		b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
	}

	b3JointId id = car.wheels[0];
	b3DirectionalContactJoint_SetGround( id, plankId );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( id ), plankId ) );
	ENSURE( b3Body_IsAwake( plankId ) );
	ENSURE( b3Joint_IsAwake( id ) );
	b3World_Step( car.worldId, 1.0f / 60.0f, 4 );

	b3DirectionalContactJoint_SetGround( id, car.groundIds[1] );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( id ), car.groundIds[1] ) );
	b3DirectionalContactJoint_SetGround( id, car.groundIds[0] );
	ENSURE( B3_ID_EQUALS( car.wheels[0], id ) );
	for ( int i = 0; i < 30; ++i )
	{
		ProbeWheels( &car, 1.0e4f, false );
		b3World_Step( car.worldId, 1.0f / 60.0f, 4 );
	}
	ENSURE( b3World_GetCounters( car.worldId ).jointCount == WHEEL_COUNT );

	b3DestroyJoint( id, false );
	ENSURE( b3Joint_IsValid( id ) == false );
	b3DestroyWorld( car.worldId );
	return 0;
}

int M31Test( void )
{
	RUN_SUBTEST( CapacityGrowsInSixtyFourBits );
	RUN_SUBTEST( ByteSizesPassTwoGibibytes );
	RUN_SUBTEST( FailedAllocationIsReported );
	RUN_SUBTEST( ContactCapacityRefusesAndCounts );
	RUN_SUBTEST( ContactJointReportsTheGroundForce );
	RUN_SUBTEST( ContactJointKeepsItsIdAcrossGrounds );
	RUN_SUBTEST( ContactJointMovesBetweenStaticAndDynamicGround );
	return 0;
}
