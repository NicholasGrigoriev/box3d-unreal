#include "Box3DPropActor.h"

#include "Box3DBodyComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

ABox3DPropActor::ABox3DPropActor()
{
	PrimaryActorTick.bCanEverTick = false;

	Body = CreateDefaultSubobject<UBox3DBodyComponent>(TEXT("Body"));
	Body->BodyType = EBox3DBodyType::Dynamic;
	Body->ShapeType = EBox3DShapeType::CollisionAsset;
	SetRootComponent(Body);

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	Mesh->SetupAttachment(Body);
	Mesh->SetMobility(EComponentMobility::Movable);
	// Chaos sees the prop for queries only: traces, sweeps, and character movement
	// keep working against it while Box3D owns the simulation. Block everything by
	// default (game collision profiles can override per instance).
	Mesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Mesh->SetCollisionObjectType(ECC_WorldDynamic);
	Mesh->SetCollisionResponseToAllChannels(ECR_Block);
	Mesh->SetGenerateOverlapEvents(false);
}

void ABox3DPropActor::SetStaticMesh(UStaticMesh* InMesh)
{
	Mesh->SetStaticMesh(InMesh);
}
