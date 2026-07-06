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

	/// Position/length: cm -> m. Scaled in double and narrowed once, matching the
	/// b3Pos seam bit-for-bit in single precision.
	inline b3Vec3 ToB3(const FVector& V)
	{
		return b3Vec3{ float(V.X * double(UEToMeters)), float(V.Y * double(UEToMeters)), float(V.Z * double(UEToMeters)) };
	}

	/// Position/length: m -> cm
	inline FVector ToUE(const b3Vec3& V)
	{
		return FVector(V.x * MetersToUE, V.y * MetersToUE, V.z * MetersToUE);
	}

	/// World position: cm -> m. The math runs in double and narrows to whatever
	/// b3Pos holds, so BOX3D_DOUBLE_PRECISION (large world mode) keeps UE's full
	/// LWC precision through this seam while single precision truncates as before.
	inline b3Pos ToB3Pos(const FVector& V)
	{
		b3Pos P;
		P.x = decltype(P.x)(V.X * double(UEToMeters));
		P.y = decltype(P.y)(V.Y * double(UEToMeters));
		P.z = decltype(P.z)(V.Z * double(UEToMeters));
		return P;
	}

	inline FVector ToUEPos(const b3Pos& P)
	{
		return FVector(P.x * double(MetersToUE), P.y * double(MetersToUE), P.z * double(MetersToUE));
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
