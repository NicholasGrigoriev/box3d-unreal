// Unit tests for the UE <-> Box3D conversion layer: unit scaling, direction
// passthrough, quaternion equivalence, and filter bit widening.

#include "Box3DConversion.h"
#include "Box3DTypes.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Tests/Box3DTestHelpers.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DConversionLengthScaleTest,
	"Box3DUnreal.Conversion.LengthScale", BOX3D_TEST_FLAGS)
bool FBox3DConversionLengthScaleTest::RunTest(const FString& Parameters)
{
	const FVector V(100.0, 250.0, -300.0); // cm

	const b3Vec3 B3 = Box3D::ToB3(V);
	TestEqual(TEXT("X cm->m"), B3.x, 1.0f, KINDA_SMALL_NUMBER);
	TestEqual(TEXT("Y cm->m"), B3.y, 2.5f, KINDA_SMALL_NUMBER);
	TestEqual(TEXT("Z cm->m"), B3.z, -3.0f, KINDA_SMALL_NUMBER);

	const FVector RoundTrip = Box3D::ToUE(B3);
	TestTrue(TEXT("cm->m->cm round trip"), RoundTrip.Equals(V, 0.001));

	const b3Pos Pos = Box3D::ToB3Pos(V);
	TestTrue(TEXT("b3Pos seam matches b3Vec3 in single precision"),
		FMath::IsNearlyEqual(Pos.x, B3.x) && FMath::IsNearlyEqual(Pos.y, B3.y) && FMath::IsNearlyEqual(Pos.z, B3.z));
	TestTrue(TEXT("pos round trip"), Box3D::ToUEPos(Pos).Equals(V, 0.001));

	const b3Vec3 Accel = Box3D::ToB3Accel(FVector(0.0, 0.0, -980.0));
	TestEqual(TEXT("gravity cm/s^2 -> m/s^2"), Accel.z, -9.8f, KINDA_SMALL_NUMBER);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DConversionDirectionTest,
	"Box3DUnreal.Conversion.DirectionUnscaled", BOX3D_TEST_FLAGS)
bool FBox3DConversionDirectionTest::RunTest(const FString& Parameters)
{
	const FVector V(1.0, -2.0, 3.0);
	const b3Vec3 B3 = Box3D::ToB3Dir(V);
	TestTrue(TEXT("directions pass through unscaled"),
		FMath::IsNearlyEqual(B3.x, 1.0f) && FMath::IsNearlyEqual(B3.y, -2.0f) && FMath::IsNearlyEqual(B3.z, 3.0f));
	TestTrue(TEXT("direction round trip"), Box3D::ToUEDir(B3).Equals(V, 0.0001));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DConversionQuaternionTest,
	"Box3DUnreal.Conversion.QuaternionEquivalence", BOX3D_TEST_FLAGS)
bool FBox3DConversionQuaternionTest::RunTest(const FString& Parameters)
{
	// A rotation is converted correctly iff rotating a vector on either side of
	// the boundary produces the same result.
	const FQuat Q(FVector(1.0, 2.0, 3.0).GetSafeNormal(), 1.234);
	const FVector V(10.0, -20.0, 30.0);

	const FVector RotatedUE = Q.RotateVector(V);
	const FVector RotatedB3 = Box3D::ToUEDir(b3RotateVector(Box3D::ToB3(Q), Box3D::ToB3Dir(V)));
	TestTrue(FString::Printf(TEXT("b3RotateVector matches FQuat::RotateVector (UE=%s b3=%s)"),
		*RotatedUE.ToCompactString(), *RotatedB3.ToCompactString()),
		RotatedB3.Equals(RotatedUE, 0.01));

	const FQuat RoundTrip = Box3D::ToUE(Box3D::ToB3(Q));
	TestTrue(TEXT("quat round trip"), RoundTrip.Equals(Q, 0.0001f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DConversionFilterBitsTest,
	"Box3DUnreal.Conversion.FilterBitsWidening", BOX3D_TEST_FLAGS)
bool FBox3DConversionFilterBitsTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("-1 widens to all 64 bits"), Box3D::ToB3Bits(-1) == UINT64_MAX);
	TestTrue(TEXT("0 stays 0"), Box3D::ToB3Bits(0) == 0ull);
	TestTrue(TEXT("single channel bit"), Box3D::ToB3Bits(1 << 5) == (1ull << 5));
	// The high bit of an int32 must zero-extend, not sign-extend, into 64 bits.
	TestTrue(TEXT("int32 sign bit zero-extends"), Box3D::ToB3Bits(INT32_MIN) == 0x80000000ull);
	TestTrue(TEXT("mixed mask zero-extends"),
		Box3D::ToB3Bits(static_cast<int32>(0xFFFFFFFE)) == 0x00000000FFFFFFFEull);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
