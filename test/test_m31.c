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
#include <math.h>
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

// ---- Suspension joint -------------------------------------------------------------------------------------------------

#define SPRUNG_MASS 400.0f
#define SPRING_RATE 40000.0f
#define SEAT_OFFSET 0.6f
#define STEP_DT ( 1.0f / 60.0f )
#define SUB_STEPS 4

typedef struct Quarter
{
	b3WorldId worldId;
	b3BodyId mountId; // static: the unsprung side held still
	b3BodyId sprungId;
	b3JointId jointId;
} Quarter;

// A sprung box (body A) on a static unsprung mount (body B), joined at the box's origin; gravity along -y.
static Quarter CreateQuarter( bool solidAxle, float preload, float bumpDamping, float reboundDamping )
{
	Quarter q = { 0 };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -GRAVITY, 0.0f };
	worldDef.enableSleep = false;
	q.worldId = b3CreateWorld( &worldDef );

	b3BodyDef mountDef = b3DefaultBodyDef();
	q.mountId = b3CreateBody( q.worldId, &mountDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	q.sprungId = b3CreateBody( q.worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 1.0f, 0.2f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = SPRUNG_MASS / ( 2.0f * 0.4f * 2.0f );
	shapeDef.filter.maskBits = 0;
	b3CreateHullShape( q.sprungId, &shapeDef, &box.base );

	b3SuspensionJointDef def = b3DefaultSuspensionJointDef();
	def.base.bodyIdA = q.sprungId;
	def.base.bodyIdB = q.mountId;
	def.stiffness = SPRING_RATE;
	def.bumpDamping = bumpDamping;
	def.reboundDamping = reboundDamping;
	def.preload[0] = preload;
	def.preload[1] = preload;
	def.lowerTravel = -0.5f;
	def.upperTravel = 0.5f;
	def.solidAxle = solidAxle;
	def.seatOffset = SEAT_OFFSET;
	q.jointId = b3CreateSuspensionJoint( q.worldId, &def );
	return q;
}

static void StepQuarter( Quarter* q, int steps )
{
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( q->worldId, STEP_DT, SUB_STEPS );
	}
}

static int SuspensionSagsByWeightOverRate( void )
{
	float weight = SPRUNG_MASS * GRAVITY;
	Quarter q = CreateQuarter( false, 0.0f, 8000.0f, 8000.0f );
	StepQuarter( &q, 300 );
	ENSURE_SMALL( b3SuspensionJoint_GetTravel( q.jointId, 0 ) - weight / SPRING_RATE, 0.002f * weight / SPRING_RATE );
	ENSURE_SMALL( b3SuspensionJoint_GetSpringForce( q.jointId, 0 ) - weight, 0.002f * weight );
	b3DestroyWorld( q.worldId );

	// A preload of the weight holds it at the design position.
	q = CreateQuarter( false, weight, 8000.0f, 8000.0f );
	StepQuarter( &q, 300 );
	ENSURE_SMALL( b3SuspensionJoint_GetTravel( q.jointId, 0 ), 1.0e-4f );
	b3DestroyWorld( q.worldId );
	return 0;
}

// Undamped from the design position, the mass oscillates at sqrt(k / m); damped, it decays at the ratio the physical
// damping gives.
static int SuspensionSpringAndDampingAreInPhysicalUnits( void )
{
	float omega = sqrtf( SPRING_RATE / SPRUNG_MASS );
	float period = 2.0f * B3_PI / omega;
	Quarter q = CreateQuarter( false, 0.0f, 0.0f, 0.0f );
	float previous = 0.0f;
	float upCrossings[3] = { 0 };
	int crossings = 0;
	float settle = SPRUNG_MASS * GRAVITY / SPRING_RATE;
	for ( int i = 1; i < 600 && crossings < 3; ++i )
	{
		StepQuarter( &q, 1 );
		float x = b3SuspensionJoint_GetTravel( q.jointId, 0 ) - settle;
		if ( previous < 0.0f && x >= 0.0f )
		{
			upCrossings[crossings++] = ( (float)i - x / ( x - previous ) ) * STEP_DT;
		}
		previous = x;
	}
	ENSURE( crossings == 3 );
	float measured = 0.5f * ( upCrossings[2] - upCrossings[0] );
	ENSURE_SMALL( measured - period, 0.02f * period );
	b3DestroyWorld( q.worldId );

	// zeta 0.1: successive peaks fall by exp(-2 pi zeta / sqrt(1 - zeta^2))
	float zeta = 0.1f;
	float c = 2.0f * zeta * sqrtf( SPRING_RATE * SPRUNG_MASS );
	q = CreateQuarter( false, 0.0f, c, c );
	float peaks[2] = { 0 };
	int peakCount = 0;
	float last = 0.0f, lastSlope = 0.0f;
	for ( int i = 1; i < 600 && peakCount < 2; ++i )
	{
		StepQuarter( &q, 1 );
		float x = b3SuspensionJoint_GetTravel( q.jointId, 0 ) - settle;
		float slope = x - last;
		if ( i > 1 && lastSlope > 0.0f && slope <= 0.0f )
		{
			peaks[peakCount++] = last;
		}
		last = x;
		lastSlope = slope;
	}
	ENSURE( peakCount == 2 );
	float ratio = peaks[1] / peaks[0];
	float expected = expf( -2.0f * B3_PI * zeta / sqrtf( 1.0f - zeta * zeta ) );
	ENSURE_SMALL( ratio - expected, 0.01f * expected );
	b3DestroyWorld( q.worldId );
	return 0;
}

// Bump damping slows the compression stroke only, rebound damping the extension only.
static int SuspensionDampsEachDirection( void )
{
	float deepest[2];
	for ( int run = 0; run < 2; ++run )
	{
		float c = 2.0f * sqrtf( SPRING_RATE * SPRUNG_MASS );
		Quarter q = CreateQuarter( false, 0.0f, run == 0 ? c : 0.0f, run == 0 ? 0.0f : c );
		deepest[run] = 0.0f;
		for ( int i = 0; i < 120; ++i )
		{
			StepQuarter( &q, 1 );
			deepest[run] = b3MaxFloat( deepest[run], b3SuspensionJoint_GetTravel( q.jointId, 0 ) );
		}
		b3DestroyWorld( q.worldId );
	}
	float settle = SPRUNG_MASS * GRAVITY / SPRING_RATE;
	// Critically damped in compression it barely overshoots; undamped in compression it reaches twice the sag.
	ENSURE( deepest[0] < 1.1f * settle );
	ENSURE( deepest[1] > 1.8f * settle );
	return 0;
}

// Pushed sideways and twisted, an independent wheel's joint holds every direction but travel.
static int SuspensionLocksTheOtherDirections( void )
{
	Quarter q = CreateQuarter( false, SPRUNG_MASS * GRAVITY, 8000.0f, 8000.0f );
	for ( int i = 0; i < 60; ++i )
	{
		b3Body_ApplyForceToCenter( q.sprungId, ( b3Vec3 ){ 2000.0f, 0.0f, 2000.0f }, true );
		b3Body_ApplyTorque( q.sprungId, ( b3Vec3 ){ 2000.0f, 2000.0f, 2000.0f }, true );
		StepQuarter( &q, 1 );
	}
	b3Pos p = b3Body_GetPosition( q.sprungId );
	b3Quat r = b3Body_GetRotation( q.sprungId );
	ENSURE_SMALL( (float)p.x, 2.0e-3f );
	ENSURE_SMALL( (float)p.z, 2.0e-3f );
	ENSURE( b3Length( r.v ) < 1.0e-3f );
	b3DestroyWorld( q.worldId );
	return 0;
}

// A solid axle rolls against its two seats: roll stiffness 2 k s^2. Yaw, pitch and sideways motion stay locked.
static int SolidAxleRollsOnItsSeats( void )
{
	float rollTorque = 2000.0f;
	Quarter q = CreateQuarter( true, 0.5f * SPRUNG_MASS * GRAVITY, 4000.0f, 4000.0f );
	for ( int i = 0; i < 300; ++i )
	{
		b3Body_ApplyTorque( q.sprungId, ( b3Vec3 ){ rollTorque, 0.0f, 0.0f }, true );
		StepQuarter( &q, 1 );
	}
	float expected = rollTorque / ( 2.0f * SPRING_RATE * SEAT_OFFSET * SEAT_OFFSET );
	float roll = b3AbsFloat( b3SuspensionJoint_GetRoll( q.jointId ) );
	ENSURE_SMALL( roll - expected, 0.02f * expected );
	float seat0 = b3SuspensionJoint_GetTravel( q.jointId, 0 );
	float seat1 = b3SuspensionJoint_GetTravel( q.jointId, 1 );
	ENSURE( seat0 * seat1 < 0.0f );

	for ( int i = 0; i < 60; ++i )
	{
		b3Body_ApplyTorque( q.sprungId, ( b3Vec3 ){ rollTorque, 3000.0f, 3000.0f }, true );
		b3Body_ApplyForceToCenter( q.sprungId, ( b3Vec3 ){ 3000.0f, 0.0f, 3000.0f }, true );
		StepQuarter( &q, 1 );
	}
	b3Pos p = b3Body_GetPosition( q.sprungId );
	b3Vec3 forward = b3RotateVector( b3Body_GetRotation( q.sprungId ), b3Vec3_axisX );
	ENSURE_SMALL( (float)p.x, 2.0e-3f );
	ENSURE_SMALL( (float)p.z, 2.0e-3f );
	ENSURE( forward.x > 1.0f - 1.0e-5f );
	b3DestroyWorld( q.worldId );
	return 0;
}

// ---- Tire mode --------------------------------------------------------------------------------------------------------

#define HUB_MASS 200.0f
#define TIRE_RATE 200000.0f
#define TIRE_RADIUS 0.5f
#define RIM_DEFLECTION 0.08f

typedef struct TireRig
{
	b3WorldId worldId;
	b3BodyId groundId;
	b3BodyId hubId;
	b3JointId tireId;
} TireRig;

// A hub (a box, no collision) on a tire-mode joint over a static ground at y = 0, the tire touching unloaded.
static TireRig CreateTireRig( float hubMass )
{
	TireRig t = { 0 };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -GRAVITY, 0.0f };
	worldDef.enableSleep = false;
	t.worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = ( b3Pos ){ 0.0f, -0.5f, 0.0f };
	t.groundId = b3CreateBody( t.worldId, &groundDef );
	b3BoxHull slab = b3MakeBoxHull( 50.0f, 0.5f, 50.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( t.groundId, &shapeDef, &slab.base );

	b3BodyDef hubDef = b3DefaultBodyDef();
	hubDef.type = b3_dynamicBody;
	hubDef.position = ( b3Pos ){ 0.0f, TIRE_RADIUS, 0.0f };
	hubDef.motionLocks.angularX = true;
	hubDef.motionLocks.angularY = true;
	hubDef.motionLocks.angularZ = true;
	t.hubId = b3CreateBody( t.worldId, &hubDef );
	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.2f, 0.2f );
	b3ShapeDef hubShape = b3DefaultShapeDef();
	hubShape.density = hubMass / ( 0.4f * 0.4f * 0.4f );
	hubShape.filter.maskBits = 0;
	b3CreateHullShape( t.hubId, &hubShape, &box.base );

	b3DirectionalContactJointDef def = b3DefaultDirectionalContactJointDef();
	def.base.bodyIdA = t.hubId;
	def.base.bodyIdB = t.groundId;
	def.radius = TIRE_RADIUS;
	def.radialSpring = true;
	def.minLength = -RIM_DEFLECTION;
	def.maxLength = 0.0f;
	def.stiffness = TIRE_RATE;
	def.damping = 2000.0f;
	def.spinInertia = 2.0f;
	t.tireId = b3CreateDirectionalContactJoint( t.worldId, &def );
	return t;
}

static void ProbeTire( TireRig* t, float brake )
{
	b3Pos centre = b3Body_GetPosition( t->hubId );
	b3DirectionalContactInput input = { 0 };
	input.hasContact = true;
	input.point = ( b3Pos ){ centre.x, 0.0f, centre.z };
	input.normal = b3Vec3_axisY;
	input.longitudinalFriction = 1.0f;
	input.lateralFriction = 1.0f;
	input.brakeTorque = brake;
	b3DirectionalContactJoint_SetInput( t->tireId, &input );
}

static int TireDeflectsAndRollsOnTheEffectiveRadius( void )
{
	TireRig t = CreateTireRig( HUB_MASS );
	for ( int i = 0; i < 240; ++i )
	{
		ProbeTire( &t, 0.0f );
		b3World_Step( t.worldId, STEP_DT, SUB_STEPS );
	}
	float deflection = HUB_MASS * GRAVITY / TIRE_RATE;
	b3DirectionalContactState state = b3DirectionalContactJoint_GetState( t.tireId );
	ENSURE_SMALL( state.suspensionLength + deflection, 0.01f * deflection );
	ENSURE_SMALL( (float)b3Body_GetPosition( t.hubId ).y - ( TIRE_RADIUS - deflection ), 0.01f * deflection );

	// Rolling free at 5 m/s, the spin matches the speed over the effective radius.
	b3Body_SetLinearVelocity( t.hubId, ( b3Vec3 ){ 5.0f, 0.0f, 0.0f } );
	b3DirectionalContactJoint_SetSpin( t.tireId, 5.0f / ( TIRE_RADIUS - deflection / 3.0f ) );
	for ( int i = 0; i < 60; ++i )
	{
		ProbeTire( &t, 0.0f );
		b3World_Step( t.worldId, STEP_DT, SUB_STEPS );
	}
	float speed = b3Body_GetLinearVelocity( t.hubId ).x;
	float spin = b3DirectionalContactJoint_GetSpin( t.tireId );
	ENSURE_SMALL( spin * ( TIRE_RADIUS - deflection / 3.0f ) - speed, 1.0e-3f * speed );
	b3DestroyWorld( t.worldId );

	// Loaded past the sidewall, the rim stops the deflection.
	t = CreateTireRig( 4.0f * TIRE_RATE * RIM_DEFLECTION / GRAVITY );
	for ( int i = 0; i < 240; ++i )
	{
		ProbeTire( &t, 0.0f );
		b3World_Step( t.worldId, STEP_DT, SUB_STEPS );
	}
	state = b3DirectionalContactJoint_GetState( t.tireId );
	ENSURE_SMALL( state.suspensionLength + RIM_DEFLECTION, 0.05f * RIM_DEFLECTION );
	b3DestroyWorld( t.worldId );
	return 0;
}

// A hub bouncing on its tire decays at the tire's damping ratio: omega h is 0.13 here, where Box3D's soft row alone
// damps at zeta 0.07.
static int TireHopDecaysAtItsOwnDamping( void )
{
	float zeta = 0.15f;
	TireRig t = CreateTireRig( HUB_MASS );
	b3DirectionalContactJoint_SetSpring( t.tireId, TIRE_RATE, 2.0f * zeta * sqrtf( TIRE_RATE * HUB_MASS ), 0.0f );
	float settle = HUB_MASS * GRAVITY / TIRE_RATE;
	// From twice the static deflection the tire never leaves the ground.
	b3Body_SetTransform( t.hubId, ( b3Pos ){ 0.0f, TIRE_RADIUS - 2.0f * settle, 0.0f }, b3Quat_identity );
	float peaks[2] = { 0 };
	int peakCount = 0;
	float last = 0.0f, lastSlope = 0.0f;
	for ( int i = 1; i < 600 && peakCount < 2; ++i )
	{
		ProbeTire( &t, 0.0f );
		b3World_Step( t.worldId, STEP_DT / SUB_STEPS, 1 );
		float x = -b3DirectionalContactJoint_GetState( t.tireId ).suspensionLength - settle;
		float slope = x - last;
		if ( i > 1 && lastSlope > 0.0f && slope <= 0.0f )
		{
			peaks[peakCount++] = last;
		}
		last = x;
		lastSlope = slope;
	}
	ENSURE( peakCount == 2 );
	float expected = expf( -2.0f * B3_PI * zeta / sqrtf( 1.0f - zeta * zeta ) );
	ENSURE_SMALL( peaks[1] / peaks[0] - expected, 0.03f * expected );
	b3DestroyWorld( t.worldId );
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
	RUN_SUBTEST( SuspensionSagsByWeightOverRate );
	RUN_SUBTEST( SuspensionSpringAndDampingAreInPhysicalUnits );
	RUN_SUBTEST( SuspensionDampsEachDirection );
	RUN_SUBTEST( SuspensionLocksTheOtherDirections );
	RUN_SUBTEST( SolidAxleRollsOnItsSeats );
	RUN_SUBTEST( TireDeflectsAndRollsOnTheEffectiveRadius );
	RUN_SUBTEST( TireHopDecaysAtItsOwnDamping );
	return 0;
}
