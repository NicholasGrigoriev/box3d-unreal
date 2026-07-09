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

void ABox3DRopeActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	// Editor preview: the straight rest shape, no physics. In game, BuildRope
	// rebuilds the skin itself and the simulation takes over.
	if (HasActorBegunPlay())
	{
		return;
	}

	const int32 Count = FMath::Clamp(NumSegments, 2, 64);
	SegmentLength = FMath::Max(RopeLength, 10.0f) / Count;
	BuildSkin();

	const FVector Start = GetActorLocation();
	FVector Dir = BuildDirection.GetSafeNormal(UE_SMALL_NUMBER, FVector::DownVector);
	if (bPinEnd)
	{
		Dir = (GetActorTransform().TransformPosition(EndPinLocation) - Start)
			.GetSafeNormal(UE_SMALL_NUMBER, FVector::DownVector);
	}
	TArray<FVector> Points;
	Points.Reserve(Count + 1);
	for (int32 Index = 0; Index <= Count; ++Index)
	{
		Points.Add(Start + Dir * (SegmentLength * Index));
	}
	ApplyPointsToSkin(Points);
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

void ABox3DRopeActor::BuildSkin()
{
	for (USplineMeshComponent* Segment : SegmentMeshes)
	{
		if (Segment != nullptr)
		{
			Segment->DestroyComponent();
		}
	}
	SegmentMeshes.Reset();

	const int32 Count = FMath::Clamp(NumSegments, 2, 64);
	const float MeshRadius = RopeMesh ? FMath::Max<float>(RopeMesh->GetBounds().BoxExtent.X, 1.0f) : 50.0f;
	const FVector2D SectionScale(RopeRadius / MeshRadius);
	SegmentMeshes.Reserve(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		USplineMeshComponent* Segment = NewObject<USplineMeshComponent>(this, NAME_None, RF_Transient);
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
}

void ABox3DRopeActor::ApplyPointsToSkin(const TArray<FVector>& WorldPoints)
{
	if (Spline == nullptr || WorldPoints.Num() < 2)
	{
		return;
	}
	const FTransform ToLocal = GetActorTransform();
	TArray<FVector> LocalPoints;
	LocalPoints.Reserve(WorldPoints.Num());
	for (const FVector& Point : WorldPoints)
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
	FVector Dir = BuildDirection.GetSafeNormal(UE_SMALL_NUMBER, FVector::DownVector);
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
		ShapeDef.filter.maskBits = bCollideWithPawns
			? UINT64_MAX
			: UINT64_MAX & ~(1ull << static_cast<int32>(EBox3DChannel::Pawn));
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

	int32 NextLinkIndex = 0;
	const auto MakeLink = [WorldId, this, &NextLinkIndex](
		b3BodyId BodyA, b3BodyId BodyB, const FVector& LocalA, const FVector& LocalB)
	{
		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = BodyA;
		Def.base.bodyIdB = BodyB;
		Def.base.localFrameA = b3Transform{ Box3D::ToB3(LocalA), Box3D::IdentityQuat };
		Def.base.localFrameB = b3Transform{ Box3D::ToB3(LocalB), Box3D::IdentityQuat };
		LinkJoints.Add({ b3CreateSphericalJoint(WorldId, &Def), NextLinkIndex++ });
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

	ActiveSegments = Count;

	Interp.SetNum(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const b3WorldTransform T = b3Body_GetTransform(Bodies[Index]);
		Interp[Index].P0 = Interp[Index].P1 = Box3D::ToUEPos(T.p);
		Interp[Index].Q0 = Interp[Index].Q1 = Box3D::ToUE(T.q);
	}
	LastStep = Subsystem->GetStepCount();
	bAllAsleep = false;
	bChainBroken = false;

	BuildSkin();
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
	EndAttachBodyId = b3BodyId{};
	LinkJoints.Empty();
	bChainBroken = false;
	Interp.Empty();
	ActiveSegments = 0;
}

void ABox3DRopeActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	PollLinkBreaks();
	UpdateRopeVisual();
}

void ABox3DRopeActor::PollLinkBreaks()
{
	if (LinkBreakForce <= 0.0f || LinkJoints.Num() == 0)
	{
		return;
	}
	for (int32 Index = LinkJoints.Num() - 1; Index >= 0; --Index)
	{
		const FRopeLink& Link = LinkJoints[Index];
		if (!b3Joint_IsValid(Link.Joint))
		{
			LinkJoints.RemoveAtSwap(Index);
			continue;
		}
		if (Box3D::ToUEDir(b3Joint_GetConstraintForce(Link.Joint)).Size() > LinkBreakForce)
		{
			const int32 CutIndex = Link.Index;
			b3DestroyJoint(Link.Joint, /*wakeAttached*/ true);
			LinkJoints.RemoveAtSwap(Index);
			bChainBroken = true;
			OnRopeCut.Broadcast(CutIndex);
		}
	}
}

void ABox3DRopeActor::CutLink(int32 LinkIndex)
{
	for (int32 Index = 0; Index < LinkJoints.Num(); ++Index)
	{
		if (LinkJoints[Index].Index == LinkIndex)
		{
			if (b3Joint_IsValid(LinkJoints[Index].Joint))
			{
				b3DestroyJoint(LinkJoints[Index].Joint, /*wakeAttached*/ true);
			}
			LinkJoints.RemoveAtSwap(Index);
			bChainBroken = true;
			OnRopeCut.Broadcast(LinkIndex);
			return;
		}
	}
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
		for (int32 Index = 0; Index < ActiveSegments; ++Index)
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
	// Spooled tail links (beyond ActiveSegments) are disabled and hidden.
	const int32 Count = ActiveSegments;

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

	if (bChainBroken)
	{
		// A cut chain is no longer one polyline: skin each segment straight
		// between its own capsule tips so severed ends separate visibly.
		const FTransform ToLocal = GetActorTransform();
		for (int32 Index = 0; Index < SegmentMeshes.Num() && Index < Count; ++Index)
		{
			const FVector TipUp = ToLocal.InverseTransformPosition(
				Positions[Index] + Rotations[Index].RotateVector(FVector(0.0, 0.0, -HalfLen)));
			const FVector TipDown = ToLocal.InverseTransformPosition(
				Positions[Index] + Rotations[Index].RotateVector(FVector(0.0, 0.0, +HalfLen)));
			SegmentMeshes[Index]->SetStartAndEnd(TipUp, TipDown - TipUp, TipDown, TipDown - TipUp, true);
		}
		return;
	}

	// Control points: segment tips, averaged where two links meet so tiny solver
	// separation never shows as a kink.
	TArray<FVector> Points;
	Points.Reserve(Count + 1);
	Points.Add(Positions[0] + Rotations[0].RotateVector(FVector(0.0, 0.0, -HalfLen)));
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FVector TipDown = Positions[Index] + Rotations[Index].RotateVector(FVector(0.0, 0.0, +HalfLen));
		if (Index + 1 < Count)
		{
			const FVector TipUp = Positions[Index + 1] + Rotations[Index + 1].RotateVector(FVector(0.0, 0.0, -HalfLen));
			Points.Add((TipDown + TipUp) * 0.5);
		}
		else
		{
			Points.Add(TipDown);
		}
	}
	ApplyPointsToSkin(Points);
}

FVector ABox3DRopeActor::GetEndLocation() const
{
	if (ActiveSegments <= 0 || !b3Body_IsValid(Bodies[ActiveSegments - 1]))
	{
		return GetActorLocation();
	}
	const b3WorldTransform T = b3Body_GetTransform(Bodies[ActiveSegments - 1]);
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
	UBox3DBodyComponent* Other = ActorToAttach ? ActorToAttach->FindComponentByClass<UBox3DBodyComponent>() : nullptr;
	if (Other == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: AttachActorToEnd needs a live UBox3DBodyComponent on %s"),
			*GetPathName(), *GetNameSafe(ActorToAttach));
		return false;
	}
	return AttachBodyToEnd(Other);
}

bool ABox3DRopeActor::AttachBodyToEnd(UBox3DBodyComponent* Body)
{
	if (ActiveSegments <= 0 || Body == nullptr || !Body->IsSimulating())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: AttachBodyToEnd needs a built rope and a live body (%s)"),
			*GetPathName(), *GetNameSafe(Body));
		return false;
	}
	DetachEnd();

	const b3WorldTransform OtherTransform = b3Body_GetTransform(Body->GetBodyId());
	const FTransform OtherWorld(Box3D::ToUE(OtherTransform.q), Box3D::ToUEPos(OtherTransform.p));
	EndAttachBodyId = Body->GetBodyId();
	EndAttachLocalPoint = OtherWorld.InverseTransformPosition(GetEndLocation());
	CreateEndAttachJoint();
	return b3Joint_IsValid(EndAttachJointId);
}

void ABox3DRopeActor::CreateEndAttachJoint()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || ActiveSegments <= 0 || !b3Body_IsValid(EndAttachBodyId))
	{
		return;
	}
	b3SphericalJointDef Def = b3DefaultSphericalJointDef();
	Def.base.bodyIdA = Bodies[ActiveSegments - 1];
	Def.base.bodyIdB = EndAttachBodyId;
	Def.base.localFrameA = b3Transform{ Box3D::ToB3(FVector(0.0, 0.0, SegmentLength * 0.5f)), Box3D::IdentityQuat };
	Def.base.localFrameB = b3Transform{ Box3D::ToB3(EndAttachLocalPoint), Box3D::IdentityQuat };
	EndAttachJointId = b3CreateSphericalJoint(Subsystem->GetBox3DWorldId(), &Def);
	b3Joint_WakeBodies(EndAttachJointId);
}

void ABox3DRopeActor::DetachEnd()
{
	if (b3Joint_IsValid(EndAttachJointId))
	{
		b3DestroyJoint(EndAttachJointId, /*wakeAttached*/ true);
	}
	EndAttachJointId = b3JointId{};
	EndAttachBodyId = b3BodyId{};
}

void ABox3DRopeActor::SetDeployedLength(float LengthCm)
{
	// A cut chain has no single deployed length left to re-rig.
	if (Bodies.Num() == 0 || bChainBroken || SegmentLength <= 0.0f)
	{
		return;
	}
	// Floor, never round up: the deployed chain must not exceed the requested
	// length, or a freshly connected grapple rope reads as slack.
	const int32 NewActive = FMath::Clamp(FMath::FloorToInt(LengthCm / SegmentLength), 1, Bodies.Num());
	if (NewActive == ActiveSegments)
	{
		return;
	}

	// The end joint is tied to the end link, which is about to change identity.
	const bool bReattach = b3Joint_IsValid(EndAttachJointId);
	if (bReattach)
	{
		b3DestroyJoint(EndAttachJointId, /*wakeAttached*/ true);
		EndAttachJointId = b3JointId{};
	}

	if (NewActive < ActiveSegments)
	{
		// Reel in: spool tail links into the winch.
		for (int32 Index = NewActive; Index < ActiveSegments; ++Index)
		{
			if (b3Body_IsValid(Bodies[Index]))
			{
				b3Body_Disable(Bodies[Index]);
			}
		}
		ActiveSegments = NewActive;
	}
	else
	{
		// Pay out: feed links back one at a time at the current end tip so they
		// unspool from the winch instead of teleporting in from where they parked.
		// Their chain joints were only deactivated by b3Body_Disable and revive
		// with the body.
		while (ActiveSegments < NewActive)
		{
			const b3BodyId Next = Bodies[ActiveSegments];
			if (!b3Body_IsValid(Next))
			{
				break;
			}
			const b3WorldTransform PrevT = b3Body_GetTransform(Bodies[ActiveSegments - 1]);
			const FQuat PrevRot = Box3D::ToUE(PrevT.q);
			const FVector NextCenter = Box3D::ToUEPos(PrevT.p) + PrevRot.RotateVector(FVector(0.0, 0.0, SegmentLength));
			b3Body_SetTransform(Next, Box3D::ToB3Pos(NextCenter), PrevT.q);
			b3Body_Enable(Next);
			if (Interp.IsValidIndex(ActiveSegments))
			{
				Interp[ActiveSegments].P0 = Interp[ActiveSegments].P1 = NextCenter;
				Interp[ActiveSegments].Q0 = Interp[ActiveSegments].Q1 = PrevRot;
			}
			++ActiveSegments;
		}
	}

	if (bReattach)
	{
		CreateEndAttachJoint();
	}
	for (int32 Index = 0; Index < SegmentMeshes.Num(); ++Index)
	{
		if (SegmentMeshes[Index] != nullptr)
		{
			SegmentMeshes[Index]->SetVisibility(Index < ActiveSegments);
		}
	}
	bAllAsleep = false;
}

float ABox3DRopeActor::GetDeployedLength() const
{
	return ActiveSegments * SegmentLength;
}

FVector ABox3DRopeActor::GetEndConstraintForce() const
{
	if (!b3Joint_IsValid(EndAttachJointId))
	{
		return FVector::ZeroVector;
	}
	return Box3D::ToUEDir(b3Joint_GetConstraintForce(EndAttachJointId));
}

bool ABox3DRopeActor::HasEndAttachment() const
{
	return b3Joint_IsValid(EndAttachJointId);
}

void ABox3DRopeActor::AddImpulseAtEnd(FVector Impulse)
{
	if (ActiveSegments > 0 && b3Body_IsValid(Bodies[ActiveSegments - 1]))
	{
		b3Body_ApplyLinearImpulse(Bodies[ActiveSegments - 1], Box3D::ToB3Dir(Impulse * Box3D::UEToMeters),
			Box3D::ToB3Pos(GetEndLocation()), /*wake*/ true);
	}
}
