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

int M31Test( void )
{
	RUN_SUBTEST( CapacityGrowsInSixtyFourBits );
	RUN_SUBTEST( ByteSizesPassTwoGibibytes );
	RUN_SUBTEST( FailedAllocationIsReported );
	RUN_SUBTEST( ContactCapacityRefusesAndCounts );
	return 0;
}
