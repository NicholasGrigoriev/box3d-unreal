#include "Box3DBreakableActor.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "box3d/box3d.h"

ABox3DBreakableActor::ABox3DBreakableActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Root->SetMobility(EComponentMobility::Movable);
}

UStaticMeshComponent* ABox3DBreakableActor::AddChunk(UStaticMesh* Mesh, const FTransform& RelativeTransform)
{
	if (HasActorBegunPlay())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: AddChunk must run before BeginPlay"), *GetPathName());
		return nullptr;
	}
	UStaticMeshComponent* Chunk = NewObject<UStaticMeshComponent>(this);
	Chunk->SetupAttachment(GetRootComponent());
	Chunk->SetRelativeTransform(RelativeTransform);
	Chunk->SetStaticMesh(Mesh);
	if (HasActorRegisteredAllComponents())
	{
		Chunk->RegisterComponent();
	}
	return Chunk;
}

void ABox3DBreakableActor::BeginPlay()
{
	Super::BeginPlay();
	BuildChunks();
}

void ABox3DBreakableActor::BuildChunks()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}
	const b3WorldId WorldId = Subsystem->GetBox3DWorldId();

	TInlineComponentArray<UStaticMeshComponent*> Found(this);
	TArray<UStaticMeshComponent*> ChunkMeshes;
	for (UStaticMeshComponent* Mesh : Found)
	{
		if (Mesh != nullptr && Mesh->GetStaticMesh() != nullptr)
		{
			ChunkMeshes.Add(Mesh);
		}
	}
	if (ChunkMeshes.Num() == 0)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: no chunk meshes; add StaticMeshComponents (or AddChunk) for the pieces"),
			*GetPathName());
		return;
	}

	ChunkBodies.Reserve(ChunkMeshes.Num());
	for (UStaticMeshComponent* Mesh : ChunkMeshes)
	{
		// Same split as ABox3DPropActor: Box3D owns motion, Chaos keeps
		// query-only collision so traces and character movement still see it.
		Mesh->SetMobility(EComponentMobility::Movable);
		Mesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		Mesh->SetCollisionObjectType(ECC_WorldDynamic);
		Mesh->SetCollisionResponseToAllChannels(ECR_Block);
		Mesh->SetGenerateOverlapEvents(false);

		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(this);
		Body->BodyType = EBox3DBodyType::Dynamic;
		Body->ShapeType = EBox3DShapeType::CollisionAsset;
		Body->Density = ChunkDensity;
		Body->bStartAwake = !bStartAsleep;
		Body->SetupAttachment(GetRootComponent());
		Body->SetWorldTransform(Mesh->GetComponentTransform());
		// The mesh becomes the body's child (at identity) before registration,
		// so CollisionAsset cooking sees it as the shape source.
		Mesh->AttachToComponent(Body, FAttachmentTransformRules::KeepWorldTransform);
		Body->RegisterComponent();
		ChunkBodies.Add(Body);
	}

	// Weld every pair of touching chunks at the midpoint between their origins.
	for (int32 IndexA = 0; IndexA < ChunkMeshes.Num(); ++IndexA)
	{
		const FBox BoundsA = ChunkMeshes[IndexA]->Bounds.GetBox().ExpandBy(WeldTolerance);
		for (int32 IndexB = IndexA + 1; IndexB < ChunkMeshes.Num(); ++IndexB)
		{
			if (!BoundsA.Intersect(ChunkMeshes[IndexB]->Bounds.GetBox()))
			{
				continue;
			}
			const UBox3DBodyComponent* BodyA = ChunkBodies[IndexA];
			const UBox3DBodyComponent* BodyB = ChunkBodies[IndexB];
			if (!BodyA->IsSimulating() || !BodyB->IsSimulating())
			{
				continue;
			}
			// Joint frames come from the b3 bodies, not the components: component
			// transforms carry render scale, which would stretch the local offsets.
			const b3WorldTransform TransformA = b3Body_GetTransform(BodyA->GetBodyId());
			const b3WorldTransform TransformB = b3Body_GetTransform(BodyB->GetBodyId());
			const FTransform WorldA(Box3D::ToUE(TransformA.q), Box3D::ToUEPos(TransformA.p));
			const FTransform WorldB(Box3D::ToUE(TransformB.q), Box3D::ToUEPos(TransformB.p));
			const FVector Midpoint = (WorldA.GetLocation() + WorldB.GetLocation()) * 0.5;

			b3WeldJointDef Def = b3DefaultWeldJointDef();
			Def.base.bodyIdA = BodyA->GetBodyId();
			Def.base.bodyIdB = BodyB->GetBodyId();
			Def.base.localFrameA = b3Transform{
				Box3D::ToB3(WorldA.InverseTransformPosition(Midpoint)),
				Box3D::ToB3(WorldA.GetRotation().Inverse()) };
			Def.base.localFrameB = b3Transform{
				Box3D::ToB3(WorldB.InverseTransformPosition(Midpoint)),
				Box3D::ToB3(WorldB.GetRotation().Inverse()) };
			Welds.Add(b3CreateWeldJoint(WorldId, &Def));
		}
	}
	UE_LOG(LogBox3D, Log, TEXT("%s: %d chunks, %d welds"), *GetNameSafe(this), ChunkBodies.Num(), Welds.Num());
}

void ABox3DBreakableActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	for (const b3JointId Weld : Welds)
	{
		if (b3Joint_IsValid(Weld))
		{
			b3DestroyJoint(Weld, /*wakeAttached*/ false);
		}
	}
	Welds.Empty();
	Super::EndPlay(EndPlayReason);
}

void ABox3DBreakableActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	CheckWelds();
}

void ABox3DBreakableActor::CheckWelds()
{
	if (BreakForce <= 0.0f || Welds.Num() == 0)
	{
		return;
	}
	bool bAnyBroke = false;
	for (int32 Index = Welds.Num() - 1; Index >= 0; --Index)
	{
		if (!b3Joint_IsValid(Welds[Index]))
		{
			Welds.RemoveAtSwap(Index);
			continue;
		}
		if (Box3D::ToUEDir(b3Joint_GetConstraintForce(Welds[Index])).Size() > BreakForce)
		{
			b3DestroyJoint(Welds[Index], /*wakeAttached*/ true);
			Welds.RemoveAtSwap(Index);
			bAnyBroke = true;
		}
	}
	if (bAnyBroke)
	{
		OnWeldBroken.Broadcast();
	}
	if (Welds.Num() == 0)
	{
		SetActorTickEnabled(false);
	}
}

void ABox3DBreakableActor::BreakAllWelds()
{
	if (Welds.Num() == 0)
	{
		return;
	}
	for (const b3JointId Weld : Welds)
	{
		if (b3Joint_IsValid(Weld))
		{
			b3DestroyJoint(Weld, /*wakeAttached*/ true);
		}
	}
	Welds.Empty();
	OnWeldBroken.Broadcast();
	SetActorTickEnabled(false);
}
