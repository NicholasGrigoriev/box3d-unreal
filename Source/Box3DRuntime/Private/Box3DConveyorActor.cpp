#include "Box3DConveyorActor.h"

#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DTypes.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"
#include "box3d/box3d.h"

ABox3DConveyorActor::ABox3DConveyorActor()
{
	PrimaryActorTick.bCanEverTick = false;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Belt"));
	SetRootComponent(Mesh);
	// Movable so the static scene mirror never cooks a second (velocity-less)
	// body for the same surface. Chaos collision stays default: pawns walk on
	// the belt via Chaos while Box3D drags the props.
	Mesh->SetMobility(EComponentMobility::Movable);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	Mesh->SetStaticMesh(CubeFinder.Object);
	Mesh->SetRelativeScale3D(FVector(4.0, 1.5, 0.2));
}

void ABox3DConveyorActor::BeginPlay()
{
	Super::BeginPlay();

	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()) ||
		Mesh == nullptr || Mesh->GetStaticMesh() == nullptr)
	{
		return;
	}

	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = b3_staticBody;
	BodyDef.position = Box3D::ToB3Pos(Mesh->GetComponentLocation());
	BodyDef.rotation = Box3D::ToB3(Mesh->GetComponentQuat());
	BodyDef.name = "Box3DConveyor";
	// userData stays null: Box3D::ResolveComponent casts it to UObject on
	// every query hit, so raw bodies must never carry anything else.
	BodyId = b3CreateBody(Subsystem->GetBox3DWorldId(), &BodyDef);
	CreateBeltShape();
}

void ABox3DConveyorActor::CreateBeltShape()
{
	if (!b3Body_IsValid(BodyId))
	{
		return;
	}
	// One shape per belt: recreate it wholesale on speed changes.
	b3ShapeId Shapes[1];
	if (b3Body_GetShapes(BodyId, Shapes, 1) > 0)
	{
		b3DestroyShape(Shapes[0], /*updateBodyMass*/ false);
	}

	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	ShapeDef.filter.categoryBits = 1ull << static_cast<int32>(EBox3DChannel::WorldStatic);
	ShapeDef.filter.maskBits = UINT64_MAX;
	ShapeDef.baseMaterial.friction = Friction;
	// box3d's conveyor feature: the tangent velocity is local to the shape and
	// projected onto the contact surface, dragging whatever rests on it.
	ShapeDef.baseMaterial.tangentVelocity = Box3D::ToB3(FVector(BeltSpeed, 0.0, 0.0));

	const FBoxSphereBounds LocalBounds = Mesh->GetStaticMesh()->GetBounds();
	const FVector Scale = Mesh->GetComponentScale().GetAbs();
	const b3Vec3 HalfExtents = Box3D::ToB3(LocalBounds.BoxExtent * Scale);
	const b3BoxHull Hull = b3MakeOffsetBoxHull(HalfExtents.x, HalfExtents.y, HalfExtents.z,
		Box3D::ToB3(FVector(LocalBounds.Origin) * Scale));
	b3CreateHullShape(BodyId, &ShapeDef, &Hull.base);
}

void ABox3DConveyorActor::SetBeltSpeed(float CmPerSec)
{
	BeltSpeed = CmPerSec;
	CreateBeltShape();
}

void ABox3DConveyorActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (b3Body_IsValid(BodyId))
	{
		b3DestroyBody(BodyId);
	}
	BodyId = b3BodyId{};
	Super::EndPlay(EndPlayReason);
}
