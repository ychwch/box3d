// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"

#include "box3d/math_functions.h"

typedef struct b3DebugDraw b3DebugDraw;
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
	float normalForce;
	float restLength;
	// The contact point is the joint's frame B origin, so the joint's force acts there and its torque is zero.
	b3Vec3 localNormalB;

	// state
	float spin;
	float rollingRadius; // radius, less a third of the tire's deflection in tire mode
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
	float springCos;
	b3Softness springSoftness;

	bool hasContact;
	bool radialSpring;
} b3DirectionalContactJoint;

// Member of the b3JointSim union; must not grow the union (checked in suspension_joint.c).
typedef struct b3SuspensionJoint
{
	// settings
	float stiffness;
	float bumpDamping;
	float reboundDamping;
	float preload[2];
	float lowerTravel;
	float upperTravel;
	float seatOffset;
	bool solidAxle;

	// state
	b3Vec2 perpImpulse;
	b3Vec3 angularImpulse; // independent: world; solid axle: y and z are the collinearity rows' impulses
	float springImpulse[2];
	float lowerImpulse[2];
	float upperImpulse[2];

	// solver data, set in prepare
	int indexA;
	int indexB;
	b3Transform frameA;
	b3Transform frameB;
	b3Vec3 deltaCenter;
	b3Vec3 perpAxisY;
	b3Vec3 perpAxisZ;
	b3Softness bumpSoftness[2];
	b3Softness reboundSoftness[2];
} b3SuspensionJoint;

// A physical spring (k, N/m) and damper (c, N s/m) on a row of effective mass m, as soft-constraint coefficients that
// decay at the spring's own rate. Box3D's soft row is a backward Euler step: per sub-step its amplitude falls by
// (1 + 2 zeta omega h + omega^2 h^2)^-1/2, which is damping the spring does not have (zeta 0.02 at 10 rad/s and
// 240 Hz, 0.1 at 50 rad/s). The input damping is chosen so that factor equals exp(-zeta omega h), and is 0 when the
// integrator alone already damps more. The row must run once per sub-step: in the solve pass, not again in relax.
b3Softness b3MakePhysicalSpring( float k, float c, float m, float h );

b3Vec3 b3GetDirectionalContactJointForce( b3World* world, b3JointSim* base );
float b3GetDirectionalContactJointImpulse( const b3JointSim* base );

void b3PrepareDirectionalContactJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartDirectionalContactJoint( b3JointSim* base, b3StepContext* context );
void b3SolveDirectionalContactJoint( b3JointSim* base, b3StepContext* context, bool useBias );

b3Vec3 b3GetSuspensionJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetSuspensionJointTorque( b3World* world, b3JointSim* base );
float b3GetSuspensionJointLinearImpulse( const b3JointSim* base );
float b3GetSuspensionJointAngularImpulse( const b3JointSim* base );
float b3GetSuspensionJointLinearSeparation( b3World* world, b3JointSim* base );
float b3GetSuspensionJointAngularSeparation( b3World* world, b3JointSim* base );
void b3PrepareSuspensionJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartSuspensionJoint( b3JointSim* base, b3StepContext* context );
void b3SolveSuspensionJoint( b3JointSim* base, b3StepContext* context, bool useBias );
void b3DrawSuspensionJoint( b3DebugDraw* draw, b3JointSim* base, b3WorldTransform transformA, b3WorldTransform transformB, float scale );

void b3RunSubStepCallback( b3StepContext* context, int subStepIndex );
b3Softness b3DirectionalSpringSoftness( const b3DirectionalContactJoint* joint, float h );
