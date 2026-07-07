#include "Box3DClothActor.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "box3d/box3d.h"

ABox3DClothActor::ABox3DClothActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Root->SetMobility(EComponentMobility::Movable);

	// Pinned particles hang from this: kinematic so moving the actor drags the
	// cloth through the subsystem's per-step target push. Zero filter bits — an
	// attachment point, not a collider.
	AnchorBody = CreateDefaultSubobject<UBox3DBodyComponent>(TEXT("ClothAnchor"));
	AnchorBody->SetupAttachment(Root);
	AnchorBody->BodyType = EBox3DBodyType::Kinematic;
	AnchorBody->ShapeType = EBox3DShapeType::Sphere;
	AnchorBody->bAutoFitShape = false;
	AnchorBody->SphereRadius = 2.0f;
	AnchorBody->Filter.CategoryBits = 0;
	AnchorBody->Filter.MaskBits = 0;

	ClothMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ClothMesh"));
	ClothMesh->SetupAttachment(Root);
	ClothMesh->SetMobility(EComponentMobility::Movable);
	ClothMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> MaterialFinder(
		TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	ClothMaterial = MaterialFinder.Object;
}

void ABox3DClothActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	// Editor preview: the flat sheet with its material, no physics. In game,
	// BuildCloth rebuilds the skin itself right before the lattice.
	if (!HasActorBegunPlay())
	{
		BuildSkin();
	}
}

void ABox3DClothActor::BeginPlay()
{
	Super::BeginPlay();
	BuildCloth();
}

void ABox3DClothActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyCloth();
	Super::EndPlay(EndPlayReason);
}

void ABox3DClothActor::BuildSkin()
{
	GridRows = FMath::Clamp(Rows, 2, 40);
	GridColumns = FMath::Clamp(Columns, 2, 40);
	GridSpacingX = FMath::Max(Width, 20.0f) / (GridColumns - 1);
	GridSpacingZ = FMath::Max(Height, 20.0f) / (GridRows - 1);
	GridHalfWidth = 0.5f * FMath::Max(Width, 20.0f);
	const int32 VertexCount = GridRows * GridColumns;

	// Winding puts the front face on local -Y; the per-frame normals match.
	VertexBuffer.SetNum(VertexCount);
	NormalBuffer.SetNum(VertexCount);
	BackNormalBuffer.SetNum(VertexCount);
	TArray<FVector2D> UVs;
	UVs.Reserve(VertexCount);
	for (int32 Row = 0; Row < GridRows; ++Row)
	{
		for (int32 Column = 0; Column < GridColumns; ++Column)
		{
			const int32 Index = ParticleIndex(Row, Column);
			VertexBuffer[Index] = LocalGridPosition(Row, Column);
			NormalBuffer[Index] = FVector(0.0, -1.0, 0.0);
			BackNormalBuffer[Index] = FVector(0.0, 1.0, 0.0);
			UVs.Emplace(Column / static_cast<float>(GridColumns - 1), Row / static_cast<float>(GridRows - 1));
		}
	}

	Triangles.Reset();
	BackTriangles.Reset();
	Triangles.Reserve((GridRows - 1) * (GridColumns - 1) * 6);
	BackTriangles.Reserve(Triangles.Max());
	for (int32 Row = 0; Row + 1 < GridRows; ++Row)
	{
		for (int32 Column = 0; Column + 1 < GridColumns; ++Column)
		{
			const int32 I00 = ParticleIndex(Row, Column);
			const int32 I01 = ParticleIndex(Row, Column + 1);
			const int32 I10 = ParticleIndex(Row + 1, Column);
			const int32 I11 = ParticleIndex(Row + 1, Column + 1);
			Triangles.Append({ I00, I01, I11 });
			Triangles.Append({ I00, I11, I10 });
			BackTriangles.Append({ I00, I11, I01 });
			BackTriangles.Append({ I00, I10, I11 });
		}
	}

	ClothMesh->ClearAllMeshSections();
	ClothMesh->CreateMeshSection_LinearColor(0, VertexBuffer, Triangles, NormalBuffer, UVs,
		TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision*/ false);
	if (bDoubleSided)
	{
		ClothMesh->CreateMeshSection_LinearColor(1, VertexBuffer, BackTriangles, BackNormalBuffer, UVs,
			TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision*/ false);
	}
	SetClothMaterial(ClothMaterial);
	ClothMesh->SetCastShadow(bCastShadow);
}

void ABox3DClothActor::SetClothMaterial(UMaterialInterface* Material)
{
	ClothMaterial = Material;
	if (ClothMesh != nullptr)
	{
		ClothMesh->SetMaterial(0, Material);
		if (bDoubleSided)
		{
			ClothMesh->SetMaterial(1, Material);
		}
	}
}

void ABox3DClothActor::BuildCloth()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}
	if (AnchorBody == nullptr || !AnchorBody->IsSimulating())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: cloth anchor has no Box3D body; cloth not built"), *GetPathName());
		return;
	}
	const b3WorldId WorldId = Subsystem->GetBox3DWorldId();

	BuildSkin();

	// Small enough that resting neighbors never touch; large enough that the
	// sheet still catches on props and pawns.
	const float ParticleRadius = 0.35f * FMath::Min(GridSpacingX, GridSpacingZ);
	const FTransform ActorTransform = GetActorTransform();
	const int32 SelfGroup = Box3D::AllocateSelfCollisionGroup();

	Bodies.Reserve(GridRows * GridColumns);
	for (int32 Row = 0; Row < GridRows; ++Row)
	{
		for (int32 Column = 0; Column < GridColumns; ++Column)
		{
			b3BodyDef BodyDef = b3DefaultBodyDef();
			BodyDef.type = b3_dynamicBody;
			BodyDef.position = Box3D::ToB3Pos(ActorTransform.TransformPosition(LocalGridPosition(Row, Column)));
			BodyDef.linearDamping = LinearDamping;
			// Point masses: rotation carries no meaning for a cloth vertex and
			// unlocked spin only feeds jitter into the lattice.
			BodyDef.motionLocks.angularX = true;
			BodyDef.motionLocks.angularY = true;
			BodyDef.motionLocks.angularZ = true;
			BodyDef.name = "Box3DCloth";
			// userData stays null: Box3D::ResolveComponent casts it to UObject on
			// every query hit, so raw bodies must never carry anything else.
			const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);

			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			ShapeDef.density = FMath::Max(Density, 1.0f);
			ShapeDef.filter.categoryBits = 1ull << static_cast<int32>(EBox3DChannel::Debris);
			ShapeDef.filter.maskBits = UINT64_MAX;
			ShapeDef.filter.groupIndex = SelfGroup;

			b3Sphere Sphere;
			Sphere.center = b3Vec3{ 0.0f, 0.0f, 0.0f };
			Sphere.radius = ParticleRadius * Box3D::UEToMeters;
			b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);

			Bodies.Add(BodyId);
		}
	}

	const auto MakeStitch = [WorldId](b3BodyId BodyA, b3BodyId BodyB, float LengthCm,
		bool bSpring, float Hertz, float DampingRatio)
	{
		b3DistanceJointDef Def = b3DefaultDistanceJointDef();
		Def.base.bodyIdA = BodyA;
		Def.base.bodyIdB = BodyB;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, Box3D::IdentityQuat };
		Def.base.localFrameB = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, Box3D::IdentityQuat };
		Def.length = LengthCm * Box3D::UEToMeters;
		Def.enableSpring = bSpring;
		if (bSpring)
		{
			Def.hertz = Hertz;
			Def.dampingRatio = DampingRatio;
		}
		b3CreateDistanceJoint(WorldId, &Def);
	};

	const float DiagonalLength = FMath::Sqrt(GridSpacingX * GridSpacingX + GridSpacingZ * GridSpacingZ);
	for (int32 Row = 0; Row < GridRows; ++Row)
	{
		for (int32 Column = 0; Column < GridColumns; ++Column)
		{
			const b3BodyId Here = Bodies[ParticleIndex(Row, Column)];
			// Structural warp/weft: rigid rods. The cloth still folds freely —
			// nothing constrains the angle between neighboring links.
			if (Column + 1 < GridColumns)
			{
				MakeStitch(Here, Bodies[ParticleIndex(Row, Column + 1)], GridSpacingX, false, 0.0f, 0.0f);
			}
			if (Row + 1 < GridRows)
			{
				MakeStitch(Here, Bodies[ParticleIndex(Row + 1, Column)], GridSpacingZ, false, 0.0f, 0.0f);
			}
			if (bShearConstraints && Row + 1 < GridRows && Column + 1 < GridColumns)
			{
				MakeStitch(Here, Bodies[ParticleIndex(Row + 1, Column + 1)], DiagonalLength,
					true, ShearHertz, ShearDampingRatio);
				MakeStitch(Bodies[ParticleIndex(Row, Column + 1)], Bodies[ParticleIndex(Row + 1, Column)],
					DiagonalLength, true, ShearHertz, ShearDampingRatio);
			}
		}
	}

	// Pins: ball joints into the kinematic anchor (which sits at the actor
	// origin — the middle of the top edge's row).
	const auto PinParticle = [&](int32 Row, int32 Column)
	{
		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = AnchorBody->GetBodyId();
		Def.base.bodyIdB = Bodies[ParticleIndex(Row, Column)];
		Def.base.localFrameA = b3Transform{ Box3D::ToB3(LocalGridPosition(Row, Column)), Box3D::IdentityQuat };
		Def.base.localFrameB = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, Box3D::IdentityQuat };
		b3CreateSphericalJoint(WorldId, &Def);
	};
	switch (PinMode)
	{
	case EBox3DClothPin::TopEdge:
		for (int32 Column = 0; Column < GridColumns; ++Column)
		{
			PinParticle(0, Column);
		}
		break;
	case EBox3DClothPin::TopCorners:
		PinParticle(0, 0);
		PinParticle(0, GridColumns - 1);
		break;
	case EBox3DClothPin::None:
		break;
	}

	Interp.SetNum(Bodies.Num());
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		Interp[Index].P0 = Interp[Index].P1 = Box3D::ToUEPos(b3Body_GetPosition(Bodies[Index]));
	}
	LastStep = Subsystem->GetStepCount();
	bAllAsleep = false;
}

void ABox3DClothActor::DestroyCloth()
{
	// Destroying a body destroys its joints, so stitches and pins go down with
	// the particles.
	for (const b3BodyId BodyId : Bodies)
	{
		if (b3Body_IsValid(BodyId))
		{
			b3DestroyBody(BodyId);
		}
	}
	Bodies.Empty();
	Interp.Empty();
}

void ABox3DClothActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	UpdateClothVisual();
}

void ABox3DClothActor::UpdateClothVisual()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || Bodies.Num() == 0 || ClothMesh == nullptr)
	{
		return;
	}

	const uint64 Step = Subsystem->GetStepCount();
	if (Step != LastStep)
	{
		// A skipped step (hitch dropped >1 step this frame) has no valid previous
		// pose; snap instead of interpolating across the gap.
		const bool bConsecutive = Step == LastStep + 1;
		bAllAsleep = true;
		for (int32 Index = 0; Index < Bodies.Num(); ++Index)
		{
			const FVector Position = Box3D::ToUEPos(b3Body_GetPosition(Bodies[Index]));
			FParticleInterp& Particle = Interp[Index];
			Particle.P0 = bConsecutive ? Particle.P1 : Position;
			Particle.P1 = Position;
			bAllAsleep &= !b3Body_IsAwake(Bodies[Index]);
		}
		LastStep = Step;
	}
	else if (bAllAsleep)
	{
		return; // settled and already rendered
	}

	const float Alpha = Subsystem->GetFixedStepAlpha();
	const FTransform ToLocal = ClothMesh->GetComponentTransform();
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		VertexBuffer[Index] = ToLocal.InverseTransformPosition(
			FMath::Lerp(Interp[Index].P0, Interp[Index].P1, Alpha));
	}

	// Area-weighted vertex normals. UE front faces satisfy N = (C-A) x (B-A)
	// for triangle (A, B, C), matching the winding chosen in BuildSkin.
	FMemory::Memzero(NormalBuffer.GetData(), NormalBuffer.Num() * sizeof(FVector));
	for (int32 Index = 0; Index + 2 < Triangles.Num(); Index += 3)
	{
		const FVector& A = VertexBuffer[Triangles[Index]];
		const FVector& B = VertexBuffer[Triangles[Index + 1]];
		const FVector& C = VertexBuffer[Triangles[Index + 2]];
		const FVector FaceNormal = FVector::CrossProduct(C - A, B - A);
		NormalBuffer[Triangles[Index]] += FaceNormal;
		NormalBuffer[Triangles[Index + 1]] += FaceNormal;
		NormalBuffer[Triangles[Index + 2]] += FaceNormal;
	}
	for (FVector& Normal : NormalBuffer)
	{
		Normal = Normal.GetSafeNormal(UE_SMALL_NUMBER, FVector(0.0, -1.0, 0.0));
	}

	ClothMesh->UpdateMeshSection_LinearColor(0, VertexBuffer, NormalBuffer,
		TArray<FVector2D>(), TArray<FLinearColor>(), TArray<FProcMeshTangent>());
	if (bDoubleSided)
	{
		for (int32 Index = 0; Index < NormalBuffer.Num(); ++Index)
		{
			BackNormalBuffer[Index] = -NormalBuffer[Index];
		}
		ClothMesh->UpdateMeshSection_LinearColor(1, VertexBuffer, BackNormalBuffer,
			TArray<FVector2D>(), TArray<FLinearColor>(), TArray<FProcMeshTangent>());
	}
}

FVector ABox3DClothActor::GetParticleLocation(int32 Row, int32 Column) const
{
	if (Row < 0 || Row >= GridRows || Column < 0 || Column >= GridColumns || Bodies.Num() == 0)
	{
		return GetActorLocation();
	}
	const b3BodyId BodyId = Bodies[Row * GridColumns + Column];
	return b3Body_IsValid(BodyId) ? Box3D::ToUEPos(b3Body_GetPosition(BodyId)) : GetActorLocation();
}

void ABox3DClothActor::AddImpulseAtNearestParticle(FVector Location, FVector Impulse)
{
	int32 Nearest = INDEX_NONE;
	double BestDistSq = TNumericLimits<double>::Max();
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		if (!b3Body_IsValid(Bodies[Index]))
		{
			continue;
		}
		const double DistSq = FVector::DistSquared(Location, Box3D::ToUEPos(b3Body_GetPosition(Bodies[Index])));
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			Nearest = Index;
		}
	}
	if (Nearest != INDEX_NONE)
	{
		b3Body_ApplyLinearImpulseToCenter(Bodies[Nearest], Box3D::ToB3Dir(Impulse * Box3D::UEToMeters), /*wake*/ true);
	}
}
