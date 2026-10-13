// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

#include "core.h"
#include "physics_world.h"

// needed for dll export
#include "box3d/box3d.h"
#include "box3d/m31.h"

void b3World_SetContactCapacity( b3WorldId worldId, int capacity )
{
	B3_ASSERT( capacity >= 0 );
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->contactCapacity = b3MaxInt( capacity, 0 );
}

int b3World_GetContactCapacity( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->contactCapacity;
}

int b3World_GetRefusedContactCount( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->refusedContactCount;
}
