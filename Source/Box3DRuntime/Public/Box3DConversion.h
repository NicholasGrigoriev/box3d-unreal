#pragma once

#include "CoreMinimal.h"
#include "box3d/math_functions.h"

/// Conversion between UE and Box3D spaces.
///
/// Axes pass through unchanged: Box3D imposes no up-axis or handedness convention
/// (gravity is a plain vector, quaternions are standard Hamilton), so a simulation fed
/// consistently with UE's Z-up coordinates is internally consistent. Only units differ:
/// UE works in centimeters, Box3D expects meters. See docs/DESIGN.md.
namespace Box3D
{
	/// Fixed world scale: 100 unreal units per meter (UE default).
	constexpr float UEToMeters = 0.01f;
	constexpr float MetersToUE = 100.0f;

	/// Position/length: cm -> m
	inline b3Vec3 ToB3(const FVector& V)
	{
		return b3Vec3{ float(V.X) * UEToMeters, float(V.Y) * UEToMeters, float(V.Z) * UEToMeters };
	}

	/// Position/length: m -> cm
	inline FVector ToUE(const b3Vec3& V)
	{
		return FVector(V.x * MetersToUE, V.y * MetersToUE, V.z * MetersToUE);
	}

	/// World position: cm -> m. In single precision builds b3Pos aliases b3Vec3; this
	/// seam is where double precision (large world mode) would slot in.
	inline b3Pos ToB3Pos(const FVector& V)
	{
		return b3Pos{ float(V.X) * UEToMeters, float(V.Y) * UEToMeters, float(V.Z) * UEToMeters };
	}

	inline FVector ToUEPos(const b3Pos& P)
	{
		return FVector(P.x * MetersToUE, P.y * MetersToUE, P.z * MetersToUE);
	}

	/// Unitless direction / angular velocity (rad/s): no scaling.
	inline b3Vec3 ToB3Dir(const FVector& V)
	{
		return b3Vec3{ float(V.X), float(V.Y), float(V.Z) };
	}

	inline FVector ToUEDir(const b3Vec3& V)
	{
		return FVector(V.x, V.y, V.z);
	}

	/// Rotation: direct component copy, both are (x, y, z, w) Hamilton quaternions.
	inline b3Quat ToB3(const FQuat& Q)
	{
		return b3Quat{ { float(Q.X), float(Q.Y), float(Q.Z) }, float(Q.W) };
	}

	inline FQuat ToUE(const b3Quat& Q)
	{
		return FQuat(Q.v.x, Q.v.y, Q.v.z, Q.s);
	}

	/// Acceleration (gravity): cm/s^2 -> m/s^2, same scale as length.
	inline b3Vec3 ToB3Accel(const FVector& V)
	{
		return ToB3(V);
	}
}
