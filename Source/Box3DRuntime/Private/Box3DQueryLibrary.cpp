#include "Box3DQueryLibrary.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	b3WorldId GetB3World(UObject* WorldContextObject)
	{
		const UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
		const UBox3DWorldSubsystem* Subsystem = World ? World->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
		return Subsystem ? Subsystem->GetBox3DWorldId() : b3WorldId{};
	}

	b3QueryFilter MakeB3QueryFilter(const FBox3DQueryFilter& Filter)
	{
		b3QueryFilter Result = b3DefaultQueryFilter();
		Result.categoryBits = Box3D::ToB3Bits(Filter.CategoryBits);
		Result.maskBits = Box3D::ToB3Bits(Filter.MaskBits);
		return Result;
	}

	FBox3DHitResult MakeHit(b3ShapeId ShapeId, b3Pos Point, b3Vec3 Normal, float Fraction,
		uint64_t UserMaterialId, int TriangleIndex)
	{
		FBox3DHitResult Hit;
		Hit.bHit = true;
		Hit.Location = Box3D::ToUEPos(Point);
		Hit.Normal = Box3D::ToUEDir(Normal);
		Hit.Fraction = Fraction;
		Hit.UserMaterialId = static_cast<int64>(UserMaterialId);
		Hit.TriangleIndex = TriangleIndex;
		Hit.Component = Box3D::ResolveComponent(ShapeId);
		Hit.Actor = Hit.Component ? Hit.Component->GetOwner() : nullptr;
		return Hit;
	}

	/// Closest-hit collector for b3World_CastShape style callbacks.
	struct FClosestHitContext
	{
		FBox3DHitResult Best;
	};

	float ClosestHitCallback(b3ShapeId ShapeId, b3Pos Point, b3Vec3 Normal, float Fraction,
		uint64_t UserMaterialId, int TriangleIndex, int /*ChildIndex*/, void* Context)
	{
		FClosestHitContext* Ctx = static_cast<FClosestHitContext*>(Context);
		if (!Ctx->Best.bHit || Fraction < Ctx->Best.Fraction)
		{
			Ctx->Best = MakeHit(ShapeId, Point, Normal, Fraction, UserMaterialId, TriangleIndex);
		}
		// Clip the cast to the hit: anything farther can be skipped by the tree.
		return Fraction;
	}

	float AllHitsCallback(b3ShapeId ShapeId, b3Pos Point, b3Vec3 Normal, float Fraction,
		uint64_t UserMaterialId, int TriangleIndex, int /*ChildIndex*/, void* Context)
	{
		TArray<FBox3DHitResult>* Hits = static_cast<TArray<FBox3DHitResult>*>(Context);
		Hits->Add(MakeHit(ShapeId, Point, Normal, Fraction, UserMaterialId, TriangleIndex));
		return 1.0f; // don't clip, keep collecting
	}
}

bool UBox3DQueryLibrary::Box3DRayCast(UObject* WorldContextObject, FVector Start, FVector End,
	const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit)
{
	OutHit = FBox3DHitResult();

	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (!b3World_IsValid(WorldId))
	{
		return false;
	}

	const b3RayResult Result = b3World_CastRayClosest(WorldId,
		Box3D::ToB3Pos(Start), Box3D::ToB3(End - Start), MakeB3QueryFilter(Filter));
	if (!Result.hit)
	{
		return false;
	}

	OutHit = MakeHit(Result.shapeId, Result.point, Result.normal, Result.fraction,
		Result.userMaterialId, Result.triangleIndex);
	return true;
}

TArray<FBox3DHitResult> UBox3DQueryLibrary::Box3DRayCastMulti(UObject* WorldContextObject, FVector Start, FVector End,
	const FBox3DQueryFilter& Filter)
{
	TArray<FBox3DHitResult> Hits;

	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (b3World_IsValid(WorldId))
	{
		b3World_CastRay(WorldId, Box3D::ToB3Pos(Start), Box3D::ToB3(End - Start),
			MakeB3QueryFilter(Filter), &AllHitsCallback, &Hits);
		Hits.Sort([](const FBox3DHitResult& A, const FBox3DHitResult& B) { return A.Fraction < B.Fraction; });
	}
	return Hits;
}

bool UBox3DQueryLibrary::Box3DSphereCast(UObject* WorldContextObject, FVector Start, FVector End, float Radius,
	const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit)
{
	OutHit = FBox3DHitResult();

	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (!b3World_IsValid(WorldId))
	{
		return false;
	}

	const b3Vec3 Point = b3Vec3{ 0.0f, 0.0f, 0.0f };
	const b3ShapeProxy Proxy{ &Point, 1, Radius * Box3D::UEToMeters };

	FClosestHitContext Context;
	b3World_CastShape(WorldId, Box3D::ToB3Pos(Start), &Proxy, Box3D::ToB3(End - Start),
		MakeB3QueryFilter(Filter), &ClosestHitCallback, &Context);

	OutHit = Context.Best;
	return OutHit.bHit;
}

bool UBox3DQueryLibrary::Box3DCapsuleCast(UObject* WorldContextObject, FVector Start, FVector End, float Radius,
	float HalfHeight, const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit)
{
	OutHit = FBox3DHitResult();

	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (!b3World_IsValid(WorldId))
	{
		return false;
	}

	const float RadiusM = Radius * Box3D::UEToMeters;
	const float SegmentHalfM = FMath::Max(HalfHeight - Radius, 0.0f) * Box3D::UEToMeters;
	const b3Vec3 Points[2] = { { 0.0f, 0.0f, -SegmentHalfM }, { 0.0f, 0.0f, SegmentHalfM } };
	const b3ShapeProxy Proxy{ Points, 2, RadiusM };

	FClosestHitContext Context;
	b3World_CastShape(WorldId, Box3D::ToB3Pos(Start), &Proxy, Box3D::ToB3(End - Start),
		MakeB3QueryFilter(Filter), &ClosestHitCallback, &Context);

	OutHit = Context.Best;
	return OutHit.bHit;
}

TArray<UBox3DBodyComponent*> UBox3DQueryLibrary::Box3DOverlapSphere(UObject* WorldContextObject, FVector Center,
	float Radius, const FBox3DQueryFilter& Filter)
{
	TArray<UBox3DBodyComponent*> Components;

	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (!b3World_IsValid(WorldId))
	{
		return Components;
	}

	const b3Vec3 Point = b3Vec3{ 0.0f, 0.0f, 0.0f };
	const b3ShapeProxy Proxy{ &Point, 1, Radius * Box3D::UEToMeters };

	b3World_OverlapShape(WorldId, Box3D::ToB3Pos(Center), &Proxy, MakeB3QueryFilter(Filter),
		[](b3ShapeId ShapeId, void* Context) -> bool
		{
			TArray<UBox3DBodyComponent*>* Out = static_cast<TArray<UBox3DBodyComponent*>*>(Context);
			if (UBox3DBodyComponent* Component = Box3D::ResolveComponent(ShapeId))
			{
				Out->AddUnique(Component);
			}
			return true; // keep searching
		},
		&Components);

	return Components;
}

namespace
{
	b3Capsule MakeMoverCapsule(float RadiusCm, float HalfHeightCm)
	{
		const float RadiusM = RadiusCm * Box3D::UEToMeters;
		const float SegmentHalfM = FMath::Max(HalfHeightCm - RadiusCm, 0.0f) * Box3D::UEToMeters;
		return b3Capsule{ { 0.0f, 0.0f, -SegmentHalfM }, { 0.0f, 0.0f, SegmentHalfM }, RadiusM };
	}
}

float UBox3DQueryLibrary::Box3DCastMover(UObject* WorldContextObject, FVector Position, FVector Translation,
	float Radius, float HalfHeight, const FBox3DQueryFilter& Filter)
{
	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (!b3World_IsValid(WorldId))
	{
		return 1.0f;
	}

	const b3Capsule Mover = MakeMoverCapsule(Radius, HalfHeight);
	return b3World_CastMover(WorldId, Box3D::ToB3Pos(Position), &Mover, Box3D::ToB3(Translation),
		MakeB3QueryFilter(Filter), nullptr, nullptr);
}

FVector UBox3DQueryLibrary::Box3DSolveMoverDelta(UObject* WorldContextObject, FVector Position, float Radius,
	float HalfHeight, FVector DesiredDelta, const FBox3DQueryFilter& Filter, int32& OutPlaneCount)
{
	OutPlaneCount = 0;

	const b3WorldId WorldId = GetB3World(WorldContextObject);
	if (!b3World_IsValid(WorldId))
	{
		return DesiredDelta;
	}

	// Gather contact planes around the capsule, then let box3d's plane solver
	// produce a delta that slides along them.
	TArray<b3CollisionPlane, TInlineAllocator<32>> Planes;
	const b3Capsule Mover = MakeMoverCapsule(Radius, HalfHeight);
	b3World_CollideMover(WorldId, Box3D::ToB3Pos(Position), &Mover, MakeB3QueryFilter(Filter),
		[](b3ShapeId /*ShapeId*/, const b3PlaneResult* PlaneResults, int PlaneCount, void* Context) -> bool
		{
			auto* Out = static_cast<TArray<b3CollisionPlane, TInlineAllocator<32>>*>(Context);
			for (int Index = 0; Index < PlaneCount && Out->Num() < 32; ++Index)
			{
				Out->Add(b3CollisionPlane{ PlaneResults[Index].plane, FLT_MAX, 0.0f, true });
			}
			return Out->Num() < 32;
		},
		&Planes);

	OutPlaneCount = Planes.Num();
	if (Planes.IsEmpty())
	{
		return DesiredDelta;
	}

	const b3PlaneSolverResult Result = b3SolvePlanes(Box3D::ToB3(DesiredDelta), Planes.GetData(), Planes.Num());
	return Box3D::ToUE(Result.delta);
}
