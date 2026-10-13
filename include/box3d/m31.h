// SPDX-FileCopyrightText: 2026 M31 fork additions
// SPDX-License-Identifier: MIT

// Fork additions to Box3D: a serial sub-step hook, the directional contact joint, and memory limits.
#pragma once

#include "box3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Called when Box3D cannot get memory: an allocation of `size` bytes failed, or an array would pass INT_MAX
/// elements. `byteCount` is what Box3D holds already. Box3D aborts when the function returns.
typedef void b3OutOfMemoryFcn( size_t size, int64_t byteCount );

/// Install or remove (NULL) the out-of-memory report. Without one, Box3D logs the size before aborting.
B3_API void b3SetOutOfMemoryFcn( b3OutOfMemoryFcn* fcn );

/// Most contacts the world holds (default INT_MAX). Past it a new pair gets no contact, so its shapes pass through
/// each other; the pair is counted and found again when either proxy next moves. Contacts already made are kept.
B3_API void b3World_SetContactCapacity( b3WorldId worldId, int capacity );
B3_API int b3World_GetContactCapacity( b3WorldId worldId );

/// Pairs refused a contact in the last step because the world was at its contact capacity.
B3_API int b3World_GetRefusedContactCount( b3WorldId worldId );

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
/// (the suspension extends along -y), z is the axle. localFrameB's origin is the contact point, set by SetContact
/// and SetInput.
///
/// Tire mode (radialSpring): body A is the wheel's unsprung body and the hard point is the wheel centre. The spring
/// row is the tire's radial spring, measured along the contact normal: radius is the unloaded radius, the rest
/// length is 0, and minLength (negative) is the deflection at which the rim strikes. The spin rolls on the
/// effective radius, radius minus a third of the deflection.
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

	/// Tire mode: the spring row is the tire's radial spring along the contact normal.
	bool radialSpring;
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
	float normalForce; // N, a normal load carried outside the joint (an explicit soil force); adds to the friction bound
} b3DirectionalContactInput;

B3_API b3DirectionalContactJointDef b3DefaultDirectionalContactJointDef( void );

/// Set the step input; same effect as SetContact or ClearContact, SetFriction, SetSteerAngle and SetBrakeTorque.
B3_API void b3DirectionalContactJoint_SetInput( b3JointId jointId, const b3DirectionalContactInput* input );

/// Create a directional contact joint. Body B may be any body (a static anchor works when the ground
/// is static); use b3DirectionalContactJoint_SetGround to re-target it. The contact point is the joint's frame B
/// origin: b3Joint_GetConstraintForce is the force on the ground there (N, world), b3Joint_GetConstraintTorque is
/// zero, and the force thresholds and joint events see it as for any joint.
B3_API b3JointId b3CreateDirectionalContactJoint( b3WorldId worldId, const b3DirectionalContactJointDef* def );

/// Set the probed contact for the coming step: world point and normal on body B (the normal points out
/// of the ground, toward the carrier).
B3_API void b3DirectionalContactJoint_SetContact( b3JointId jointId, b3Pos point, b3Vec3 normal );

/// No ground under the wheel this step: every row is skipped, the spin is kept.
B3_API void b3DirectionalContactJoint_ClearContact( b3JointId jointId );

/// Re-target the joint to another ground body. The joint keeps its id; its settings, spin, contact and warm-start
/// impulses carry over, and the contact keeps its world position until the next SetContact or SetInput.
B3_API void b3DirectionalContactJoint_SetGround( b3JointId jointId, b3BodyId groundId );

/// Friction coefficients bounding the longitudinal and lateral rows (impulse <= mu * normal impulse).
B3_API void b3DirectionalContactJoint_SetFriction( b3JointId jointId, float longitudinal, float lateral );

/// Slip damping (N s/m) of the longitudinal and lateral rows. 0, the default, is a rigid row: it drives the slip
/// velocity to zero within its bound. c > 0 makes the row a damper, impulse = -c h * slip velocity, solved implicitly
/// once per sub-step: a tire's force-slip curve as its secant, C / (r |ω| + v0), stable at any stiffness.
B3_API void b3DirectionalContactJoint_SetSlipDamping( b3JointId jointId, float longitudinal, float lateral );

/// Steering angle (rad) about the hard point's y-axis, positive to the left.
B3_API void b3DirectionalContactJoint_SetSteerAngle( b3JointId jointId, float angle );

/// Brake torque capacity (N m) acting on the spin degree of freedom.
B3_API void b3DirectionalContactJoint_SetBrakeTorque( b3JointId jointId, float torque );

B3_API void b3DirectionalContactJoint_SetSpin( b3JointId jointId, float spin );
B3_API float b3DirectionalContactJoint_GetSpin( b3JointId jointId );

B3_API b3DirectionalContactState b3DirectionalContactJoint_GetState( b3JointId jointId );

/// Suspension spring for the coming step: stiffness (N/m), damping (N s/m) and rest length (m, hard point to the
/// contact point minus the radius). Stiffness 0 removes the spring row; the bump stop stays.
B3_API void b3DirectionalContactJoint_SetSpring( b3JointId jointId, float stiffness, float damping, float restLength );

/// Current center of mass and rotation of a body during the sub-step (start of step plus the sub-step deltas).
/// Returns false for a body that is not awake (static and sleeping bodies return their stored pose).
B3_API bool b3SubStep_GetBodyPose( const b3SubStepContext* context, b3BodyId bodyId, b3Pos* center, b3Quat* rotation );

/// Current suspension length of a directional contact joint, measured to the contact point along the contact
/// normal as the solver measures it, and its normal impulse (spring plus bump stop) of the previous sub-step.
B3_API void b3SubStep_GetSuspension( const b3SubStepContext* context, b3JointId jointId, float* length, float* normalImpulse );

/// Spring of the suspension row for the coming sub-step (same units as b3DirectionalContactJoint_SetSpring).
B3_API void b3SubStep_SetSpring( b3SubStepContext* context, b3JointId jointId, float stiffness, float damping, float restLength );

/// Friction coefficients and external normal force (N) for the coming sub-step.
B3_API void b3SubStep_SetFriction( b3SubStepContext* context, b3JointId jointId, float longitudinal, float lateral );
B3_API void b3SubStep_SetNormalForce( b3SubStepContext* context, b3JointId jointId, float normalForce );
B3_API void b3SubStep_SetSlipDamping( b3SubStepContext* context, b3JointId jointId, float longitudinal, float lateral );

/// Brake torque capacity (N m) from the coming sub-step until the next step input: a brake modulated within the step.
B3_API void b3SubStep_SetBrakeTorque( b3SubStepContext* context, b3JointId jointId, float torque );

/// Suspension joint: a chassis (body A) and an unsprung body (body B), a wheel hub or a solid axle.
/// Frame A is the mount: travel along y, forward x, axle z. Frame B is the matching point on the unsprung body.
/// - Independent wheel: only travel along y is free.
/// - Solid axle: travel and roll about x are free; two spring seats sit at -/+ seatOffset along z.
/// Travel is positive in compression. Each seat's spring pushes the unsprung body down by
/// preload + stiffness * travel, with bump damping while compressing and rebound damping while extending, and
/// stops at the travel limits.
typedef struct b3SuspensionJointDef
{
	b3JointDef base;

	/// Spring stiffness per seat (N/m)
	float stiffness;

	/// Damping per seat (N s/m), compressing and extending
	float bumpDamping;
	float reboundDamping;

	/// Spring force at zero travel (N), per seat; seat 0 is at -seatOffset
	float preload[2];

	/// Travel limits (m): lowerTravel, negative, at full droop; upperTravel at the bump stop
	float lowerTravel;
	float upperTravel;

	/// Half the spring track (m), solid axles only
	float seatOffset;

	/// Roll about x is free and the axle sits on two seats
	bool solidAxle;
} b3SuspensionJointDef;

B3_API b3SuspensionJointDef b3DefaultSuspensionJointDef( void );
B3_API b3JointId b3CreateSuspensionJoint( b3WorldId worldId, const b3SuspensionJointDef* def );
B3_API void b3SuspensionJoint_SetSpring( b3JointId jointId, float stiffness, float bumpDamping, float reboundDamping );
B3_API void b3SuspensionJoint_SetPreload( b3JointId jointId, float seat0, float seat1 );
B3_API void b3SuspensionJoint_SetTravelLimits( b3JointId jointId, float lower, float upper );

/// Travel of a seat (m, positive in compression), from the bodies' current transforms
B3_API float b3SuspensionJoint_GetTravel( b3JointId jointId, int seat );

/// Spring and stop force of a seat in the last sub-step (N, positive pushing the unsprung body down)
B3_API float b3SuspensionJoint_GetSpringForce( b3JointId jointId, int seat );

/// Roll of a solid axle relative to the chassis (rad); zero for an independent wheel
B3_API float b3SuspensionJoint_GetRoll( b3JointId jointId );

#ifdef __cplusplus
}
#endif
