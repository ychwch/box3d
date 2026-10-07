// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

// Fork additions to Box3D: a serial sub-step hook and the directional contact joint.
#pragma once

#include "box3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Opaque context passed to a sub-step callback. Valid only during the callback.
typedef struct b3SubStepContext b3SubStepContext;

/// Called once per sub-step on the stepping thread, after velocities are integrated (gravity, forces,
/// damping) and before constraints are warm started. Every worker is parked while it runs, so the
/// callback may read and write awake body velocities and directional contact joint state through the
/// b3SubStep_* functions. It must not call any other Box3D function.
typedef void b3SubStepFcn( b3SubStepContext* context, void* userContext );

/// Install or remove (NULL) the sub-step callback of a world.
B3_API void b3World_SetSubStepCallback( b3WorldId worldId, b3SubStepFcn* fcn, void* userContext );

/// Sub-step time step (s).
B3_API float b3SubStep_GetTimeStep( const b3SubStepContext* context );

/// Index of this sub-step within the step, 0 .. count - 1.
B3_API int b3SubStep_GetIndex( const b3SubStepContext* context );

/// Number of sub-steps in this step.
B3_API int b3SubStep_GetCount( const b3SubStepContext* context );

/// Velocity of the center of mass and angular velocity of an awake body. Returns false (and zero
/// velocities) for a body that is not in the awake set.
B3_API bool b3SubStep_GetBodyVelocity( const b3SubStepContext* context, b3BodyId bodyId, b3Vec3* linearVelocity,
									   b3Vec3* angularVelocity );

/// Apply a linear impulse at the center of mass and an angular impulse to an awake body (world frame).
B3_API void b3SubStep_ApplyBodyImpulse( b3SubStepContext* context, b3BodyId bodyId, b3Vec3 linearImpulse,
										b3Vec3 angularImpulse );

/// Wheel spin (rad/s, positive rolls the carrier forward) of a directional contact joint.
B3_API float b3SubStep_GetSpin( const b3SubStepContext* context, b3JointId jointId );
B3_API void b3SubStep_SetSpin( b3SubStepContext* context, b3JointId jointId, float spin );

/// Longitudinal impulse (N s) the joint applied in the previous sub-step; positive pushes the carrier forward.
B3_API float b3SubStep_GetLongitudinalImpulse( const b3SubStepContext* context, b3JointId jointId );

/// Spin and longitudinal impulse in one lookup; the impulse is 0 without contact.
B3_API void b3SubStep_GetSpinAndImpulse( const b3SubStepContext* context, b3JointId jointId, float* spin, float* impulse );

/// True when the joint has a ground contact this step.
B3_API bool b3SubStep_HasContact( const b3SubStepContext* context, b3JointId jointId );

/// Directional contact joint: a contact between a carrier (body A, e.g. a chassis) and a ground body
/// (body B, static or dynamic) found by a probe outside the solver, with
/// - a unilateral suspension spring along the contact normal (stiffness and damping in N/m and N s/m),
/// - a bump stop at the minimum suspension length,
/// - a longitudinal friction row coupled to a wheel spin degree of freedom held by the joint,
/// - a lateral friction row,
/// - a brake row on the spin degree of freedom.
/// The friction coefficients that bound the two friction rows are set per step from outside (a slip
/// curve, an edge-hold law, a constant), so the joint holds stick exactly at low speed.
/// localFrameA is the suspension hard point: x is the wheel forward direction at zero steer, y is up
/// (the suspension extends along -y), z is the axle. localFrameB is unused.
typedef struct b3DirectionalContactJointDef
{
	/// Base joint definition. bodyIdA is the carrier, bodyIdB the ground body.
	b3JointDef base;

	/// Wheel radius (m)
	float radius;

	/// Suspension length range (m), measured from the hard point to the wheel center along -y
	float minLength;
	float maxLength;

	/// Natural spring length is maxLength + preloadLength
	float preloadLength;

	/// Suspension spring stiffness (N/m) and damping (N s/m)
	float stiffness;
	float damping;

	/// Rotational inertia of the spin degree of freedom (kg m^2)
	float spinInertia;
} b3DirectionalContactJointDef;

/// Per-step state of a directional contact joint, for telemetry and the vehicle layer.
typedef struct b3DirectionalContactState
{
	bool hasContact;
	float suspensionLength;
	float spin;
	float normalImpulse;		// spring + bump stop, last sub-step (N s)
	float longitudinalImpulse;	// last sub-step (N s)
	float lateralImpulse;		// last sub-step (N s)
	float brakeImpulse;			// last sub-step (N s m)
	b3Vec3 longitudinalAxis;	// world, at the last prepare
	b3Vec3 lateralAxis;
	b3Vec3 normal;
} b3DirectionalContactState;

/// Everything the probe and the vehicle layer set for one step, in one call.
typedef struct b3DirectionalContactInput
{
	bool hasContact;
	b3Pos point;   // world, on body B
	b3Vec3 normal; // world, out of the ground
	float longitudinalFriction;
	float lateralFriction;
	float steerAngle;
	float brakeTorque;
} b3DirectionalContactInput;

B3_API b3DirectionalContactJointDef b3DefaultDirectionalContactJointDef( void );

/// Set the step input; same effect as SetContact or ClearContact, SetFriction, SetSteerAngle and SetBrakeTorque.
B3_API void b3DirectionalContactJoint_SetInput( b3JointId jointId, const b3DirectionalContactInput* input );

/// Create a directional contact joint. Body B may be any body (a static anchor works when the ground
/// is static); use b3DirectionalContactJoint_SetGround to re-target it.
B3_API b3JointId b3CreateDirectionalContactJoint( b3WorldId worldId, const b3DirectionalContactJointDef* def );

/// Set the probed contact for the coming step: world point and normal on body B (the normal points out
/// of the ground, toward the carrier).
B3_API void b3DirectionalContactJoint_SetContact( b3JointId jointId, b3Pos point, b3Vec3 normal );

/// No ground under the wheel this step: every row is skipped, the spin is kept.
B3_API void b3DirectionalContactJoint_ClearContact( b3JointId jointId );

/// Re-target the joint to another ground body. Returns the id to use from now on (the joint is
/// re-created when the body changes; settings, spin and impulses carry over).
B3_API b3JointId b3DirectionalContactJoint_SetGround( b3JointId jointId, b3BodyId groundId );

/// Friction coefficients bounding the longitudinal and lateral rows (impulse <= mu * normal impulse).
B3_API void b3DirectionalContactJoint_SetFriction( b3JointId jointId, float longitudinal, float lateral );

/// Steering angle (rad) about the hard point's y-axis, positive to the left.
B3_API void b3DirectionalContactJoint_SetSteerAngle( b3JointId jointId, float angle );

/// Brake torque capacity (N m) acting on the spin degree of freedom.
B3_API void b3DirectionalContactJoint_SetBrakeTorque( b3JointId jointId, float torque );

B3_API void b3DirectionalContactJoint_SetSpin( b3JointId jointId, float spin );
B3_API float b3DirectionalContactJoint_GetSpin( b3JointId jointId );

B3_API b3DirectionalContactState b3DirectionalContactJoint_GetState( b3JointId jointId );

#ifdef __cplusplus
}
#endif
