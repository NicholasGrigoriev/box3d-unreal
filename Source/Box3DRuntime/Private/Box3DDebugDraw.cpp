#include "Box3DDebugDraw.h"

#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "box3d/box3d.h"

static TAutoConsoleVariable<int32> CVarBox3DDebugDraw(
	TEXT("box3d.DebugDraw"), 0,
	TEXT("Draw the Box3D world with debug lines. 0=off, 1=shapes (+joints).\n")
	TEXT("Refine with box3d.DebugDraw.Bounds/Contacts/Mass/Islands/Distance."));

static TAutoConsoleVariable<int32> CVarBox3DDrawJoints(
	TEXT("box3d.DebugDraw.Joints"), 1, TEXT("Include joints in Box3D debug draw."));

static TAutoConsoleVariable<int32> CVarBox3DDrawBounds(
	TEXT("box3d.DebugDraw.Bounds"), 0, TEXT("Include shape AABBs in Box3D debug draw."));

static TAutoConsoleVariable<int32> CVarBox3DDrawContacts(
	TEXT("box3d.DebugDraw.Contacts"), 0, TEXT("Include contact points and normals in Box3D debug draw."));

static TAutoConsoleVariable<int32> CVarBox3DDrawMass(
	TEXT("box3d.DebugDraw.Mass"), 0, TEXT("Include center of mass markers in Box3D debug draw."));

static TAutoConsoleVariable<int32> CVarBox3DDrawIslands(
	TEXT("box3d.DebugDraw.Islands"), 0, TEXT("Include island bounding boxes in Box3D debug draw."));

static TAutoConsoleVariable<float> CVarBox3DDrawDistance(
	TEXT("box3d.DebugDraw.Distance"), 10000.0f,
	TEXT("Half-extent of the debug draw volume around the view, in cm."));

namespace
{
	constexpr int32 CircleSegments = 16;
	constexpr int32 ArcSegments = 8;

	FColor ToFColor(b3HexColor Color)
	{
		// The high byte may carry a b3DebugMaterial preset; only the low 24 bits are RGB.
		const uint32 C = static_cast<uint32>(Color);
		return FColor((C >> 16) & 0xFF, (C >> 8) & 0xFF, C & 0xFF);
	}

	void AddLine(TArray<FVector3f>& Out, const FVector3f& A, const FVector3f& B)
	{
		Out.Add(A);
		Out.Add(B);
	}

	void AddCircle(TArray<FVector3f>& Out, const FVector3f& Center, const FVector3f& U, const FVector3f& V, float Radius)
	{
		for (int32 Index = 0; Index < CircleSegments; ++Index)
		{
			const float A0 = 2.0f * PI * Index / CircleSegments;
			const float A1 = 2.0f * PI * (Index + 1) / CircleSegments;
			AddLine(Out,
				Center + Radius * (U * FMath::Cos(A0) + V * FMath::Sin(A0)),
				Center + Radius * (U * FMath::Cos(A1) + V * FMath::Sin(A1)));
		}
	}

	/// Half circle from +U through +V to -U.
	void AddArc(TArray<FVector3f>& Out, const FVector3f& Center, const FVector3f& U, const FVector3f& V, float Radius)
	{
		for (int32 Index = 0; Index < ArcSegments; ++Index)
		{
			const float A0 = PI * Index / ArcSegments;
			const float A1 = PI * (Index + 1) / ArcSegments;
			AddLine(Out,
				Center + Radius * (U * FMath::Cos(A0) + V * FMath::Sin(A0)),
				Center + Radius * (U * FMath::Cos(A1) + V * FMath::Sin(A1)));
		}
	}

	FVector3f ToF3(const b3Vec3& V)
	{
		return FVector3f(V.x, V.y, V.z);
	}

	/// Byte-offset array access into box3d's blob-style hull/mesh data.
	template <typename T, typename TBlob>
	const T* BlobArray(const TBlob* Blob, int32 ByteOffset)
	{
		return reinterpret_cast<const T*>(reinterpret_cast<const uint8*>(Blob) + ByteOffset);
	}

	/// Draw-pass context handed through b3DebugDraw::context.
	struct FDrawCtx
	{
		const UWorld* World = nullptr;
	};
}

FBox3DDebugDrawer::~FBox3DDebugDrawer()
{
	// b3DestroyWorld destroys every shape allocation, which fires DestroyShapeThunk,
	// so by the time the subsystem releases the drawer the cache must have drained.
	if (GetAliveCount() != 0)
	{
		UE_LOG(LogBox3D, Warning, TEXT("Box3D debug shape cache leaked %d entries"), GetAliveCount());
	}
}

void* FBox3DDebugDrawer::CreateShapeThunk(const b3DebugShape* DebugShape, void* Context)
{
	FBox3DDebugDrawer* Drawer = static_cast<FBox3DDebugDrawer*>(Context);
	++Drawer->CreatedCount;
	return Drawer->BuildWireShape(*DebugShape);
}

void FBox3DDebugDrawer::DestroyShapeThunk(void* UserShape, void* Context)
{
	FBox3DDebugDrawer* Drawer = static_cast<FBox3DDebugDrawer*>(Context);
	++Drawer->DestroyedCount;
	delete static_cast<FWireShape*>(UserShape);
}

FBox3DDebugDrawer::FWireShape* FBox3DDebugDrawer::BuildWireShape(const b3DebugShape& DebugShape)
{
	FWireShape* Wire = new FWireShape();

	switch (DebugShape.type)
	{
		case b3_sphereShape:
		{
			const FVector3f Center = ToF3(DebugShape.sphere->center);
			const float Radius = DebugShape.sphere->radius;
			AddCircle(Wire->Points, Center, FVector3f::XAxisVector, FVector3f::YAxisVector, Radius);
			AddCircle(Wire->Points, Center, FVector3f::XAxisVector, FVector3f::ZAxisVector, Radius);
			AddCircle(Wire->Points, Center, FVector3f::YAxisVector, FVector3f::ZAxisVector, Radius);
			break;
		}

		case b3_capsuleShape:
		{
			const FVector3f P1 = ToF3(DebugShape.capsule->center1);
			const FVector3f P2 = ToF3(DebugShape.capsule->center2);
			const float Radius = DebugShape.capsule->radius;

			FVector3f Axis = P2 - P1;
			if (!Axis.Normalize())
			{
				Axis = FVector3f::ZAxisVector;
			}
			FVector3f U, V;
			Axis.FindBestAxisVectors(U, V);

			AddCircle(Wire->Points, P1, U, V, Radius);
			AddCircle(Wire->Points, P2, U, V, Radius);
			for (const FVector3f& Side : { U, V, -U, -V })
			{
				AddLine(Wire->Points, P1 + Radius * Side, P2 + Radius * Side);
			}
			AddArc(Wire->Points, P1, U, -Axis, Radius);
			AddArc(Wire->Points, P1, V, -Axis, Radius);
			AddArc(Wire->Points, P2, U, Axis, Radius);
			AddArc(Wire->Points, P2, V, Axis, Radius);
			break;
		}

		case b3_hullShape:
		{
			const b3HullData* Hull = DebugShape.hull;
			const b3Vec3* Points = BlobArray<b3Vec3>(Hull, Hull->pointOffset);
			const b3HullHalfEdge* Edges = BlobArray<b3HullHalfEdge>(Hull, Hull->edgeOffset);
			for (int32 Index = 0; Index < Hull->edgeCount; ++Index)
			{
				// Each physical edge appears as two half-edges; draw it once.
				if (Index < Edges[Index].twin)
				{
					AddLine(Wire->Points, ToF3(Points[Edges[Index].origin]), ToF3(Points[Edges[Edges[Index].twin].origin]));
				}
			}
			break;
		}

		case b3_meshShape:
		{
			const b3MeshData* Data = DebugShape.mesh->data;
			const FVector3f Scale = ToF3(DebugShape.mesh->scale);
			const b3Vec3* Vertices = BlobArray<b3Vec3>(Data, Data->vertexOffset);
			const b3MeshTriangle* Triangles = BlobArray<b3MeshTriangle>(Data, Data->triangleOffset);

			TSet<uint64> SeenEdges;
			SeenEdges.Reserve(Data->triangleCount * 3);
			auto AddEdge = [&](int32 I0, int32 I1)
			{
				const uint64 Key = (uint64(FMath::Min(I0, I1)) << 32) | uint64(FMath::Max(I0, I1));
				bool bAlreadySeen = false;
				SeenEdges.Add(Key, &bAlreadySeen);
				if (!bAlreadySeen)
				{
					AddLine(Wire->Points, ToF3(Vertices[I0]) * Scale, ToF3(Vertices[I1]) * Scale);
				}
			};
			for (int32 Index = 0; Index < Data->triangleCount; ++Index)
			{
				const b3MeshTriangle& Tri = Triangles[Index];
				AddEdge(Tri.index1, Tri.index2);
				AddEdge(Tri.index2, Tri.index3);
				AddEdge(Tri.index3, Tri.index1);
			}
			break;
		}

		default:
			// Compound and height field shapes are not created by this plugin yet
			// (deferred features); cache an empty wire so box3d does not re-ask.
			break;
	}

	return Wire;
}

bool FBox3DDebugDrawer::IsDrawEnabled()
{
#if ENABLE_DRAW_DEBUG
	return CVarBox3DDebugDraw.GetValueOnGameThread() != 0;
#else
	return false;
#endif
}

void FBox3DDebugDrawer::Draw(const UWorld* World, b3WorldId WorldId) const
{
#if ENABLE_DRAW_DEBUG
	if (World == nullptr || !b3World_IsValid(WorldId))
	{
		return;
	}

	FDrawCtx Ctx{ World };

	b3DebugDraw Draw = b3DefaultDebugDraw();
	Draw.context = &Ctx;
	Draw.drawShapes = true;
	Draw.drawJoints = CVarBox3DDrawJoints.GetValueOnGameThread() != 0;
	Draw.drawBounds = CVarBox3DDrawBounds.GetValueOnGameThread() != 0;
	Draw.drawContacts = CVarBox3DDrawContacts.GetValueOnGameThread() != 0;
	Draw.drawContactNormals = Draw.drawContacts;
	Draw.drawMass = CVarBox3DDrawMass.GetValueOnGameThread() != 0;
	Draw.drawIslands = CVarBox3DDrawIslands.GetValueOnGameThread() != 0;

	// Limit drawing to a volume around the local view.
	FVector ViewLocation = FVector::ZeroVector;
	FRotator ViewRotation = FRotator::ZeroRotator;
	if (const APlayerController* PC = World->GetFirstPlayerController())
	{
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
	}
	const FVector Extent(FMath::Max(100.0f, CVarBox3DDrawDistance.GetValueOnGameThread()));
	Draw.drawingBounds = b3AABB{ Box3D::ToB3(ViewLocation - Extent), Box3D::ToB3(ViewLocation + Extent) };

	Draw.DrawShapeFcn = [](void* UserShape, b3WorldTransform Transform, b3HexColor Color, void* Context) -> bool
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		const FWireShape* Wire = static_cast<const FWireShape*>(UserShape);
		const FVector Origin = Box3D::ToUEPos(Transform.p);
		const FQuat Rotation = Box3D::ToUE(Transform.q);
		const FColor DrawColor = ToFColor(Color);
		for (int32 Index = 0; Index + 1 < Wire->Points.Num(); Index += 2)
		{
			DrawDebugLine(Ctx->World,
				Origin + Rotation.RotateVector(FVector(Wire->Points[Index]) * Box3D::MetersToUE),
				Origin + Rotation.RotateVector(FVector(Wire->Points[Index + 1]) * Box3D::MetersToUE),
				DrawColor);
		}
		return true;
	};

	Draw.DrawSegmentFcn = [](b3Pos P1, b3Pos P2, b3HexColor Color, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		DrawDebugLine(Ctx->World, Box3D::ToUEPos(P1), Box3D::ToUEPos(P2), ToFColor(Color));
	};

	Draw.DrawTransformFcn = [](b3WorldTransform Transform, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		DrawDebugCoordinateSystem(Ctx->World, Box3D::ToUEPos(Transform.p),
			Box3D::ToUE(Transform.q).Rotator(), 30.0f);
	};

	Draw.DrawPointFcn = [](b3Pos P, float Size, b3HexColor Color, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		DrawDebugPoint(Ctx->World, Box3D::ToUEPos(P), Size, ToFColor(Color));
	};

	Draw.DrawSphereFcn = [](b3Pos P, float Radius, b3HexColor Color, float /*Alpha*/, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		DrawDebugSphere(Ctx->World, Box3D::ToUEPos(P), Radius * Box3D::MetersToUE, 12, ToFColor(Color));
	};

	Draw.DrawCapsuleFcn = [](b3Pos P1, b3Pos P2, float Radius, b3HexColor Color, float /*Alpha*/, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		const FVector A = Box3D::ToUEPos(P1);
		const FVector B = Box3D::ToUEPos(P2);
		const float RadiusUE = Radius * Box3D::MetersToUE;
		const FVector Axis = B - A;
		const FQuat Rotation = Axis.IsNearlyZero() ? FQuat::Identity : FRotationMatrix::MakeFromZ(Axis).ToQuat();
		DrawDebugCapsule(Ctx->World, (A + B) * 0.5, Axis.Size() * 0.5 + RadiusUE, RadiusUE, Rotation, ToFColor(Color));
	};

	Draw.DrawBoundsFcn = [](b3AABB Bounds, b3HexColor Color, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		const FVector Lower = Box3D::ToUE(Bounds.lowerBound);
		const FVector Upper = Box3D::ToUE(Bounds.upperBound);
		DrawDebugBox(Ctx->World, (Lower + Upper) * 0.5, (Upper - Lower) * 0.5, ToFColor(Color));
	};

	Draw.DrawBoxFcn = [](b3Vec3 Extents, b3WorldTransform Transform, b3HexColor Color, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		DrawDebugBox(Ctx->World, Box3D::ToUEPos(Transform.p), Box3D::ToUE(Extents),
			Box3D::ToUE(Transform.q), ToFColor(Color));
	};

	Draw.DrawStringFcn = [](b3Pos P, const char* Text, b3HexColor Color, void* Context)
	{
		const FDrawCtx* Ctx = static_cast<const FDrawCtx*>(Context);
		DrawDebugString(Ctx->World, Box3D::ToUEPos(P), FString(ANSI_TO_TCHAR(Text)), nullptr, ToFColor(Color), 0.0f);
	};

	b3World_Draw(WorldId, &Draw, UINT64_MAX);
#endif // ENABLE_DRAW_DEBUG
}
