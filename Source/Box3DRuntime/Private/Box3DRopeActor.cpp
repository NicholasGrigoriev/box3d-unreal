#include "Box3DRopeActor.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/SplineComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"
#include "box3d/box3d.h"

ABox3DRopeActor::ABox3DRopeActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Root->SetMobility(EComponentMobility::Movable);

	// The rope hangs from this: kinematic so moving the actor drags the rope
	// through the subsystem's per-step target push. Zero filter bits — it is an
	// attachment point, not a collider.
	AnchorBody = CreateDefaultSubobject<UBox3DBodyComponent>(TEXT("RopeAnchor"));
	AnchorBody->SetupAttachment(Root);
	AnchorBody->BodyType = EBox3DBodyType::Kinematic;
	AnchorBody->ShapeType = EBox3DShapeType::Sphere;
	AnchorBody->bAutoFitShape = false;
	AnchorBody->SphereRadius = 2.0f;
	AnchorBody->Filter.CategoryBits = 0;
	AnchorBody->Filter.MaskBits = 0;

	Spline = CreateDefaultSubobject<USplineComponent>(TEXT("RopeSpline"));
	Spline->SetupAttachment(Root);
	Spline->SetMobility(EComponentMobility::Movable);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(
		TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	RopeMesh = CylinderFinder.Object;
}

void ABox3DRopeActor::BeginPlay()
{
	Super::BeginPlay();
	BuildRope();
}

void ABox3DRopeActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyRope();
	Super::EndPlay(EndPlayReason);
}

void ABox3DRopeActor::BuildRope()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}
	if (AnchorBody == nullptr || !AnchorBody->IsSimulating())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: rope anchor has no Box3D body; rope not built"), *GetPathName());
		return;
	}
	const b3WorldId WorldId = Subsystem->GetBox3DWorldId();

	const int32 Count = FMath::Clamp(NumSegments, 2, 64);
	SegmentLength = FMath::Max(RopeLength, 10.0f) / Count;
	const float HalfLen = SegmentLength * 0.5f;

	const FVector Start = GetActorLocation();
	FVector Dir = FVector::DownVector;
	if (bPinEnd)
	{
		Dir = (GetActorTransform().TransformPosition(EndPinLocation) - Start)
			.GetSafeNormal(UE_SMALL_NUMBER, FVector::DownVector);
	}
	// Capsules run along local Z; this rotation lays that axis down the rope.
	const FQuat SegmentRot = FQuat::FindBetweenNormals(FVector::UpVector, Dir);

	const int32 SelfGroup = Box3D::AllocateSelfCollisionGroup();

	Bodies.Reserve(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_dynamicBody;
		BodyDef.position = Box3D::ToB3Pos(Start + Dir * (SegmentLength * (Index + 0.5f)));
		BodyDef.rotation = Box3D::ToB3(SegmentRot);
		BodyDef.linearDamping = LinearDamping;
		BodyDef.angularDamping = AngularDamping;
		BodyDef.name = "Box3DRope";
		// userData stays null: Box3D::ResolveComponent casts it to UObject on
		// every query hit, so raw bodies must never carry anything else.
		const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.density = FMath::Max(Density, 1.0f);
		ShapeDef.filter.categoryBits = 1ull << static_cast<int32>(EBox3DChannel::Debris);
		ShapeDef.filter.maskBits = UINT64_MAX;
		ShapeDef.filter.groupIndex = SelfGroup;

		// Adjacent links overlap at the joint pivots; the negative group keeps
		// the chain from fighting itself.
		b3Capsule Capsule;
		const float CylinderHalf = FMath::Max(HalfLen - RopeRadius, 0.1f);
		Capsule.center1 = Box3D::ToB3(FVector(0.0, 0.0, +CylinderHalf));
		Capsule.center2 = Box3D::ToB3(FVector(0.0, 0.0, -CylinderHalf));
		Capsule.radius = RopeRadius * Box3D::UEToMeters;
		b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);

		Bodies.Add(BodyId);
	}

	const auto MakeLink = [WorldId](b3BodyId BodyA, b3BodyId BodyB, const FVector& LocalA, const FVector& LocalB)
	{
		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = BodyA;
		Def.base.bodyIdB = BodyB;
		Def.base.localFrameA = b3Transform{ Box3D::ToB3(LocalA), Box3D::IdentityQuat };
		Def.base.localFrameB = b3Transform{ Box3D::ToB3(LocalB), Box3D::IdentityQuat };
		b3CreateSphericalJoint(WorldId, &Def);
	};

	// Anchor sits exactly at the rope start; link the first segment's top tip to it.
	MakeLink(AnchorBody->GetBodyId(), Bodies[0], FVector::ZeroVector, FVector(0.0, 0.0, -HalfLen));
	for (int32 Index = 0; Index + 1 < Count; ++Index)
	{
		MakeLink(Bodies[Index], Bodies[Index + 1], FVector(0.0, 0.0, +HalfLen), FVector(0.0, 0.0, -HalfLen));
	}

	if (bPinEnd)
	{
		b3BodyDef PinDef = b3DefaultBodyDef();
		PinDef.type = b3_staticBody;
		PinDef.position = Box3D::ToB3Pos(GetActorTransform().TransformPosition(EndPinLocation));
		PinDef.name = "Box3DRopePin";
		EndPinBodyId = b3CreateBody(WorldId, &PinDef);
		MakeLink(EndPinBodyId, Bodies.Last(), FVector::ZeroVector, FVector(0.0, 0.0, +HalfLen));
	}

	Interp.SetNum(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const b3WorldTransform T = b3Body_GetTransform(Bodies[Index]);
		Interp[Index].P0 = Interp[Index].P1 = Box3D::ToUEPos(T.p);
		Interp[Index].Q0 = Interp[Index].Q1 = Box3D::ToUE(T.q);
	}
	LastStep = Subsystem->GetStepCount();
	bAllAsleep = false;

	// Visual skin: one spline mesh per segment, refit to the chain every frame.
	const float MeshRadius = RopeMesh ? FMath::Max<float>(RopeMesh->GetBounds().BoxExtent.X, 1.0f) : 50.0f;
	const FVector2D SectionScale(RopeRadius / MeshRadius);
	SegmentMeshes.Reserve(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		USplineMeshComponent* Segment = NewObject<USplineMeshComponent>(this);
		Segment->SetMobility(EComponentMobility::Movable);
		Segment->SetupAttachment(GetRootComponent());
		Segment->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Segment->SetCastShadow(bCastShadow);
		Segment->SetForwardAxis(RopeMeshAxis, false);
		Segment->SetStaticMesh(RopeMesh);
		if (RopeMaterial != nullptr)
		{
			Segment->SetMaterial(0, RopeMaterial);
		}
		Segment->SetStartScale(SectionScale, false);
		Segment->SetEndScale(SectionScale, false);
		Segment->RegisterComponent();
		SegmentMeshes.Add(Segment);
	}
	UpdateRopeVisual();
}

void ABox3DRopeActor::DestroyRope()
{
	DetachEnd();
	// Destroying a body destroys its joints, so the links (and the anchor/pin
	// joints) go down with the segments.
	for (const b3BodyId BodyId : Bodies)
	{
		if (b3Body_IsValid(BodyId))
		{
			b3DestroyBody(BodyId);
		}
	}
	Bodies.Empty();
	if (b3Body_IsValid(EndPinBodyId))
	{
		b3DestroyBody(EndPinBodyId);
	}
	EndPinBodyId = b3BodyId{};
	Interp.Empty();
}

void ABox3DRopeActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	UpdateRopeVisual();
}

void ABox3DRopeActor::UpdateRopeVisual()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || Bodies.Num() == 0 || Spline == nullptr)
	{
		return;
	}

	const uint64 Step = Subsystem->GetStepCount();
	if (Step != LastStep)
	{
		// A skipped step (hitch dropped >1 step this frame) has no valid previous
		// pose; snap that segment instead of interpolating across the gap.
		const bool bConsecutive = Step == LastStep + 1;
		bAllAsleep = true;
		for (int32 Index = 0; Index < Bodies.Num(); ++Index)
		{
			const b3WorldTransform T = b3Body_GetTransform(Bodies[Index]);
			FBodyInterp& Segment = Interp[Index];
			Segment.P0 = bConsecutive ? Segment.P1 : Box3D::ToUEPos(T.p);
			Segment.Q0 = bConsecutive ? Segment.Q1 : Box3D::ToUE(T.q);
			Segment.P1 = Box3D::ToUEPos(T.p);
			Segment.Q1 = Box3D::ToUE(T.q);
			bAllAsleep &= !b3Body_IsAwake(Bodies[Index]);
		}
		LastStep = Step;
	}
	else if (bAllAsleep)
	{
		return; // settled and already rendered
	}

	const float Alpha = Subsystem->GetFixedStepAlpha();
	const float HalfLen = SegmentLength * 0.5f;
	const int32 Count = Bodies.Num();

	TArray<FVector, TInlineAllocator<65>> Positions;
	TArray<FQuat, TInlineAllocator<65>> Rotations;
	Positions.SetNum(Count);
	Rotations.SetNum(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FBodyInterp& Segment = Interp[Index];
		Positions[Index] = FMath::Lerp(Segment.P0, Segment.P1, Alpha);
		Rotations[Index] = FQuat::Slerp(Segment.Q0, Segment.Q1, Alpha);
	}

	// Control points: segment tips, averaged where two links meet so tiny solver
	// separation never shows as a kink.
	TArray<FVector, TInlineAllocator<66>> Points;
	Points.SetNum(Count + 1);
	Points[0] = Positions[0] + Rotations[0].RotateVector(FVector(0.0, 0.0, -HalfLen));
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FVector TipDown = Positions[Index] + Rotations[Index].RotateVector(FVector(0.0, 0.0, +HalfLen));
		if (Index + 1 < Count)
		{
			const FVector TipUp = Positions[Index + 1] + Rotations[Index + 1].RotateVector(FVector(0.0, 0.0, -HalfLen));
			Points[Index + 1] = (TipDown + TipUp) * 0.5;
		}
		else
		{
			Points[Count] = TipDown;
		}
	}

	const FTransform ToLocal = GetActorTransform();
	TArray<FVector> LocalPoints;
	LocalPoints.Reserve(Points.Num());
	for (const FVector& Point : Points)
	{
		LocalPoints.Add(ToLocal.InverseTransformPosition(Point));
	}
	Spline->SetSplinePoints(LocalPoints, ESplineCoordinateSpace::Local, true);

	for (int32 Index = 0; Index < SegmentMeshes.Num() && Index + 1 < LocalPoints.Num(); ++Index)
	{
		SegmentMeshes[Index]->SetStartAndEnd(
			Spline->GetLocationAtSplinePoint(Index, ESplineCoordinateSpace::Local),
			Spline->GetTangentAtSplinePoint(Index, ESplineCoordinateSpace::Local),
			Spline->GetLocationAtSplinePoint(Index + 1, ESplineCoordinateSpace::Local),
			Spline->GetTangentAtSplinePoint(Index + 1, ESplineCoordinateSpace::Local),
			true);
	}
}

FVector ABox3DRopeActor::GetEndLocation() const
{
	if (Bodies.Num() == 0 || !b3Body_IsValid(Bodies.Last()))
	{
		return GetActorLocation();
	}
	const b3WorldTransform T = b3Body_GetTransform(Bodies.Last());
	return Box3D::ToUEPos(T.p) + Box3D::ToUE(T.q).RotateVector(FVector(0.0, 0.0, SegmentLength * 0.5f));
}

FVector ABox3DRopeActor::GetSegmentLocation(int32 SegmentIndex) const
{
	if (!Bodies.IsValidIndex(SegmentIndex) || !b3Body_IsValid(Bodies[SegmentIndex]))
	{
		return GetActorLocation();
	}
	return Box3D::ToUEPos(b3Body_GetPosition(Bodies[SegmentIndex]));
}

bool ABox3DRopeActor::AttachActorToEnd(AActor* ActorToAttach)
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	UBox3DBodyComponent* Other = ActorToAttach ? ActorToAttach->FindComponentByClass<UBox3DBodyComponent>() : nullptr;
	if (Subsystem == nullptr || Bodies.Num() == 0 || Other == nullptr || !Other->IsSimulating())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: AttachActorToEnd needs a live UBox3DBodyComponent on %s"),
			*GetPathName(), *GetNameSafe(ActorToAttach));
		return false;
	}
	DetachEnd();

	const b3WorldTransform OtherTransform = b3Body_GetTransform(Other->GetBodyId());
	const FTransform OtherWorld(Box3D::ToUE(OtherTransform.q), Box3D::ToUEPos(OtherTransform.p));

	b3SphericalJointDef Def = b3DefaultSphericalJointDef();
	Def.base.bodyIdA = Bodies.Last();
	Def.base.bodyIdB = Other->GetBodyId();
	Def.base.localFrameA = b3Transform{ Box3D::ToB3(FVector(0.0, 0.0, SegmentLength * 0.5f)), Box3D::IdentityQuat };
	Def.base.localFrameB = b3Transform{ Box3D::ToB3(OtherWorld.InverseTransformPosition(GetEndLocation())), Box3D::IdentityQuat };
	EndAttachJointId = b3CreateSphericalJoint(Subsystem->GetBox3DWorldId(), &Def);
	b3Joint_WakeBodies(EndAttachJointId);
	return true;
}

void ABox3DRopeActor::DetachEnd()
{
	if (b3Joint_IsValid(EndAttachJointId))
	{
		b3DestroyJoint(EndAttachJointId, /*wakeAttached*/ true);
	}
	EndAttachJointId = b3JointId{};
}

void ABox3DRopeActor::AddImpulseAtEnd(FVector Impulse)
{
	if (Bodies.Num() > 0 && b3Body_IsValid(Bodies.Last()))
	{
		b3Body_ApplyLinearImpulse(Bodies.Last(), Box3D::ToB3Dir(Impulse * Box3D::UEToMeters),
			Box3D::ToB3Pos(GetEndLocation()), /*wake*/ true);
	}
}
