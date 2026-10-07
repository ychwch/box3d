// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"

#include "box3d/math_functions.h"

typedef struct b3JointSim b3JointSim;
typedef struct b3StepContext b3StepContext;
typedef struct b3World b3World;

// Member of the b3JointSim union; must not grow the union (checked in directional_contact_joint.c).
typedef struct b3DirectionalContactJoint
{
	// settings
	float radius;
	float minLength;
	float maxLength;
	float preloadLength;
	float stiffness;
	float damping;
	float invSpinInertia;

	// per-step inputs
	float steerAngle;
	float brakeTorque;
	float longitudinalFriction;
	float lateralFriction;
	b3Vec3 localPointB;
	b3Vec3 localNormalB;

	// state
	float spin;
	float springImpulse;
	float bumpImpulse;
	float longitudinalImpulse;
	float lateralImpulse;
	float brakeImpulse;

	// solver data, set in prepare
	int indexA;
	int indexB;
	b3Vec3 rA;
	b3Vec3 rB;
	b3Vec3 deltaCenter;
	b3Vec3 normal;
	b3Vec3 longitudinalAxis;
	b3Vec3 lateralAxis;
	float length0;
	float normalMass;
	float lateralMass;
	float longitudinalMassLocked; // bodies only (spin held by the brake)
	float longitudinalMassFree;	  // bodies plus spin
	b3Softness springSoftness;

	bool hasContact;
} b3DirectionalContactJoint;

void b3PrepareDirectionalContactJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartDirectionalContactJoint( b3JointSim* base, b3StepContext* context );
void b3SolveDirectionalContactJoint( b3JointSim* base, b3StepContext* context, bool useBias );

void b3RunSubStepCallback( b3StepContext* context, int subStepIndex );
