#include "Box3DFracture.h"

#include "Box3DRuntime.h"
#include "Math/RandomStream.h"
#include "box3d/base.h"

namespace Box3D::Fracture
{
	namespace
	{
		/// Clipping tolerance (cm). Well below QuantizeStep so quantized planes
		/// never straddle the epsilon band by construction.
		constexpr double ClipEpsilon = 1e-4;

		/// Working polytope during clipping. Face tags carry the site index whose
		/// bisector created the face (INDEX_NONE = inherited proxy face). Vertices
		/// may go unreferenced as faces are clipped away; compaction happens once
		/// in FinalizeFragment.
		struct FPolyFace
		{
			TArray<int32> Verts;
			int32 Tag = INDEX_NONE;
		};

		struct FPoly
		{
			TArray<FVector> Verts;
			TArray<FPolyFace> Faces;
		};

		FPoly MakePoly(const FFractureProxy& Proxy)
		{
			FPoly Poly;
			Poly.Verts = Proxy.Vertices;
			Poly.Faces.Reserve(Proxy.Faces.Num());
			for (const TArray<int32>& Face : Proxy.Faces)
			{
				Poly.Faces.Add(FPolyFace{ Face, INDEX_NONE });
			}
			return Poly;
		}

		/// Clip Poly to the half-space (PlaneN | P) - PlaneD <= 0. The cap face
		/// sealing the cut gets Tag. Returns false when nothing volumetric remains.
		///
		/// No transcendentals: distances, lerps and integer bookkeeping only, so
		/// identical inputs clip bit-identically on every run.
		bool ClipByPlane(FPoly& Poly, const FVector& PlaneN, double PlaneD, int32 Tag)
		{
			const int32 NumVerts = Poly.Verts.Num();
			TArray<double> Dist;
			Dist.SetNumUninitialized(NumVerts);
			bool bAnyOutside = false;
			bool bAnyInside = false;
			for (int32 Index = 0; Index < NumVerts; ++Index)
			{
				const double D = (PlaneN | Poly.Verts[Index]) - PlaneD;
				Dist[Index] = D;
				bAnyOutside |= D > ClipEpsilon;
				bAnyInside |= D < -ClipEpsilon;
			}
			if (!bAnyOutside)
			{
				return true;
			}
			if (!bAnyInside)
			{
				Poly.Verts.Reset();
				Poly.Faces.Reset();
				return false;
			}

			// One cut vertex per unique polytope edge, computed in canonical index
			// order so the two faces sharing the edge get the identical vertex and
			// the mesh stays watertight.
			TMap<int64, int32> CutVerts;
			auto GetCutVertex = [&](int32 A, int32 B) -> int32
			{
				if (A > B)
				{
					Swap(A, B);
				}
				const int64 Key = (int64(A) << 32) | int64(B);
				if (const int32* Found = CutVerts.Find(Key))
				{
					return *Found;
				}
				const double T = Dist[A] / (Dist[A] - Dist[B]);
				const int32 Index = Poly.Verts.Add(Poly.Verts[A] + T * (Poly.Verts[B] - Poly.Verts[A]));
				CutVerts.Add(Key, Index);
				return Index;
			};
			// Cut vertices (Index >= NumVerts) lie on the plane by construction.
			auto IsOnPlane = [&](int32 Index)
			{
				return Index >= NumVerts || FMath::Abs(Dist[Index]) <= ClipEpsilon;
			};

			TArray<FPolyFace> NewFaces;
			NewFaces.Reserve(Poly.Faces.Num() + 1);
			// Directed edges of the cap face: each clipped face contributes its
			// on-plane segment reversed, so chaining them yields a loop wound CCW
			// seen from outside the kept half (along +PlaneN).
			TArray<TPair<int32, int32>> CapEdges;

			for (const FPolyFace& Face : Poly.Faces)
			{
				FPolyFace NewFace;
				NewFace.Tag = Face.Tag;
				const int32 Count = Face.Verts.Num();
				for (int32 Index = 0; Index < Count; ++Index)
				{
					const int32 A = Face.Verts[Index];
					const int32 B = Face.Verts[(Index + 1) % Count];
					if (Dist[A] <= ClipEpsilon)
					{
						NewFace.Verts.Add(A);
					}
					if ((Dist[A] < -ClipEpsilon && Dist[B] > ClipEpsilon) ||
						(Dist[A] > ClipEpsilon && Dist[B] < -ClipEpsilon))
					{
						NewFace.Verts.Add(GetCutVertex(A, B));
					}
				}
				if (NewFace.Verts.Num() < 3)
				{
					continue;
				}
				const int32 NewCount = NewFace.Verts.Num();
				for (int32 Index = 0; Index < NewCount; ++Index)
				{
					const int32 U = NewFace.Verts[Index];
					const int32 V = NewFace.Verts[(Index + 1) % NewCount];
					if (IsOnPlane(U) && IsOnPlane(V))
					{
						CapEdges.Add({ V, U });
					}
				}
				NewFaces.Add(MoveTemp(NewFace));
			}

			if (CapEdges.Num() >= 3)
			{
				TMap<int32, int32> NextVert;
				NextVert.Reserve(CapEdges.Num());
				for (const TPair<int32, int32>& Edge : CapEdges)
				{
					NextVert.Add(Edge.Key, Edge.Value);
				}

				FPolyFace Cap;
				Cap.Tag = Tag;
				const int32 Start = CapEdges[0].Key;
				int32 Current = Start;
				bool bClosed = false;
				for (int32 Guard = 0; Guard < CapEdges.Num(); ++Guard)
				{
					Cap.Verts.Add(Current);
					const int32* Next = NextVert.Find(Current);
					if (Next == nullptr)
					{
						break;
					}
					Current = *Next;
					if (Current == Start)
					{
						bClosed = true;
						break;
					}
				}
				if (bClosed && Cap.Verts.Num() >= 3)
				{
					NewFaces.Add(MoveTemp(Cap));
				}
				else
				{
					UE_LOG(LogBox3D, Warning, TEXT("Box3D fracture: cap face failed to close (%d edges)"),
						CapEdges.Num());
				}
			}

			Poly.Faces = MoveTemp(NewFaces);
			// A closed 3D polytope needs at least 4 faces.
			return Poly.Faces.Num() >= 4;
		}

		double PolygonArea(const TArray<FVector>& Verts, const TArray<int32>& Face)
		{
			const FVector& V0 = Verts[Face[0]];
			FVector CrossSum = FVector::ZeroVector;
			for (int32 Index = 1; Index + 1 < Face.Num(); ++Index)
			{
				CrossSum += (Verts[Face[Index]] - V0) ^ (Verts[Face[Index + 1]] - V0);
			}
			return 0.5 * CrossSum.Length();
		}

		/// Compact vertices, compute face areas, and integrate volume + centroid
		/// via signed origin tetrahedra (divergence theorem — faces wind CCW
		/// outward, so the signed sum is the enclosed volume regardless of where
		/// the origin sits). Face tags stay site indices; the caller remaps them to
		/// fragment indices.
		FBox3DFragmentData FinalizeFragment(const FPoly& Poly)
		{
			FBox3DFragmentData Fragment;
			TArray<int32> Remap;
			Remap.Init(INDEX_NONE, Poly.Verts.Num());

			double Vol6 = 0.0;
			FVector CentroidSum = FVector::ZeroVector;
			for (const FPolyFace& Face : Poly.Faces)
			{
				FBox3DFragmentFace& OutFace = Fragment.Faces.AddDefaulted_GetRef();
				OutFace.NeighborIndex = Face.Tag;
				OutFace.Area = PolygonArea(Poly.Verts, Face.Verts);
				OutFace.VertexIndices.Reserve(Face.Verts.Num());
				for (const int32 Index : Face.Verts)
				{
					if (Remap[Index] == INDEX_NONE)
					{
						Remap[Index] = Fragment.Vertices.Add(Poly.Verts[Index]);
					}
					OutFace.VertexIndices.Add(Remap[Index]);
				}

				const FVector& V0 = Poly.Verts[Face.Verts[0]];
				for (int32 Index = 1; Index + 1 < Face.Verts.Num(); ++Index)
				{
					const FVector& V1 = Poly.Verts[Face.Verts[Index]];
					const FVector& V2 = Poly.Verts[Face.Verts[Index + 1]];
					const double Tet6 = V0 | (V1 ^ V2);
					Vol6 += Tet6;
					CentroidSum += Tet6 * (V0 + V1 + V2);
				}
			}

			Fragment.Volume = Vol6 / 6.0;
			if (Vol6 > UE_DOUBLE_SMALL_NUMBER)
			{
				Fragment.Centroid = CentroidSum / (4.0 * Vol6);
			}
			return Fragment;
		}

		/// Rebuild a fragment's vertex array to only the vertices its faces
		/// reference, in face-iteration first-seen order (the same order
		/// FinalizeFragment produces, so untouched fragments are unchanged).
		/// Merging drops the shared faces of a pair, orphaning their vertices.
		void CompactFragmentVertices(FBox3DFragmentData& Fragment)
		{
			TArray<int32> Remap;
			Remap.Init(INDEX_NONE, Fragment.Vertices.Num());
			TArray<FVector> Compacted;
			Compacted.Reserve(Fragment.Vertices.Num());
			for (FBox3DFragmentFace& Face : Fragment.Faces)
			{
				for (int32& Index : Face.VertexIndices)
				{
					if (Remap[Index] == INDEX_NONE)
					{
						Remap[Index] = Compacted.Add(Fragment.Vertices[Index]);
					}
					Index = Remap[Index];
				}
			}
			Fragment.Vertices = MoveTemp(Compacted);
		}

		FBox3DFragmentNeighbor* FindNeighborLink(FBox3DFragmentData& Fragment, int32 NeighborIndex)
		{
			return Fragment.Neighbors.FindByPredicate([NeighborIndex](const FBox3DFragmentNeighbor& Link)
			{
				return Link.FragmentIndex == NeighborIndex;
			});
		}

		/// Absorb fragment Small into fragment Target: union the surfaces (both
		/// cells' faces minus the shared pair), add volumes exactly, volume-weight
		/// the centroid, and redirect all adjacency from Small to Target with
		/// shared-face areas summed on both sides (keeps the canonical-per-pair
		/// symmetry invariant).
		void MergeFragmentInto(TArray<FBox3DFragmentData>& Fragments, int32 Small, int32 Target)
		{
			FBox3DFragmentData& T = Fragments[Target];
			FBox3DFragmentData& S = Fragments[Small];

			const double TotalVolume = T.Volume + S.Volume;
			T.Centroid = (T.Centroid * T.Volume + S.Centroid * S.Volume) / TotalVolume;
			T.Volume = TotalVolume;

			for (int32 Index = T.Faces.Num() - 1; Index >= 0; --Index)
			{
				if (T.Faces[Index].NeighborIndex == Small)
				{
					T.Faces.RemoveAt(Index);
				}
			}
			const int32 VertexOffset = T.Vertices.Num();
			T.Vertices.Append(S.Vertices);
			for (FBox3DFragmentFace& Face : S.Faces)
			{
				if (Face.NeighborIndex == Target)
				{
					continue;
				}
				for (int32& Index : Face.VertexIndices)
				{
					Index += VertexOffset;
				}
				T.Faces.Add(MoveTemp(Face));
			}

			T.Neighbors.RemoveAll([Small](const FBox3DFragmentNeighbor& Link)
			{
				return Link.FragmentIndex == Small;
			});
			for (const FBox3DFragmentNeighbor& Link : S.Neighbors)
			{
				if (Link.FragmentIndex == Target)
				{
					continue;
				}
				FBox3DFragmentData& N = Fragments[Link.FragmentIndex];
				for (FBox3DFragmentFace& Face : N.Faces)
				{
					if (Face.NeighborIndex == Small)
					{
						Face.NeighborIndex = Target;
					}
				}
				FBox3DFragmentNeighbor* BackToSmall = FindNeighborLink(N, Small);
				check(BackToSmall != nullptr); // adjacency was symmetric before merging
				if (FBox3DFragmentNeighbor* BackToTarget = FindNeighborLink(N, Target))
				{
					BackToTarget->SharedFaceArea += BackToSmall->SharedFaceArea;
					N.Neighbors.RemoveAll([Small](const FBox3DFragmentNeighbor& Candidate)
					{
						return Candidate.FragmentIndex == Small;
					});
				}
				else
				{
					BackToSmall->FragmentIndex = Target;
				}
				N.Neighbors.Sort([](const FBox3DFragmentNeighbor& A, const FBox3DFragmentNeighbor& B)
				{
					return A.FragmentIndex < B.FragmentIndex;
				});

				if (FBox3DFragmentNeighbor* Forward = FindNeighborLink(T, Link.FragmentIndex))
				{
					Forward->SharedFaceArea += Link.SharedFaceArea;
				}
				else
				{
					T.Neighbors.Add({ Link.FragmentIndex, Link.SharedFaceArea });
				}
			}
			T.Neighbors.Sort([](const FBox3DFragmentNeighbor& A, const FBox3DFragmentNeighbor& B)
			{
				return A.FragmentIndex < B.FragmentIndex;
			});

			S = FBox3DFragmentData();
		}

		/// Merge fragments below MinVolume into a neighbor until every fragment
		/// with neighbors meets the threshold. Deterministic: fragments scanned in
		/// ascending index order, target = neighbor with the largest shared-face
		/// area (Neighbors is sorted ascending and the comparison is strict, so
		/// ties pick the lower index). Dead slots are compacted at the end and all
		/// indices remapped (monotonic remap keeps neighbor lists sorted).
		void MergeSmallFragments(TArray<FBox3DFragmentData>& Fragments, double MinVolume)
		{
			TArray<bool> Dead;
			Dead.Init(false, Fragments.Num());

			bool bMergedAny = true;
			while (bMergedAny)
			{
				bMergedAny = false;
				for (int32 Small = 0; Small < Fragments.Num(); ++Small)
				{
					if (Dead[Small] || Fragments[Small].Volume >= MinVolume ||
						Fragments[Small].Neighbors.IsEmpty())
					{
						continue;
					}
					int32 Target = INDEX_NONE;
					double BestArea = -1.0;
					for (const FBox3DFragmentNeighbor& Link : Fragments[Small].Neighbors)
					{
						if (Link.SharedFaceArea > BestArea)
						{
							BestArea = Link.SharedFaceArea;
							Target = Link.FragmentIndex;
						}
					}
					MergeFragmentInto(Fragments, Small, Target);
					Dead[Small] = true;
					bMergedAny = true;
				}
			}

			TArray<int32> Remap;
			Remap.Init(INDEX_NONE, Fragments.Num());
			int32 AliveCount = 0;
			for (int32 Index = 0; Index < Fragments.Num(); ++Index)
			{
				if (!Dead[Index])
				{
					Remap[Index] = AliveCount++;
				}
			}
			if (AliveCount == Fragments.Num())
			{
				return;
			}

			TArray<FBox3DFragmentData> Compacted;
			Compacted.Reserve(AliveCount);
			for (int32 Index = 0; Index < Fragments.Num(); ++Index)
			{
				if (Dead[Index])
				{
					continue;
				}
				FBox3DFragmentData& Fragment = Fragments[Index];
				for (FBox3DFragmentFace& Face : Fragment.Faces)
				{
					if (Face.NeighborIndex != INDEX_NONE)
					{
						Face.NeighborIndex = Remap[Face.NeighborIndex];
					}
				}
				for (FBox3DFragmentNeighbor& Link : Fragment.Neighbors)
				{
					Link.FragmentIndex = Remap[Link.FragmentIndex];
				}
				CompactFragmentVertices(Fragment);
				Compacted.Add(MoveTemp(Fragment));
			}
			Fragments = MoveTemp(Compacted);
		}

		/// Outward face plane from a proxy face via Newell's method.
		bool MakeFacePlane(const TArray<FVector>& Verts, const TArray<int32>& Face, FVector& OutN, double& OutD)
		{
			FVector Normal = FVector::ZeroVector;
			for (int32 Index = 0; Index < Face.Num(); ++Index)
			{
				const FVector& A = Verts[Face[Index]];
				const FVector& B = Verts[Face[(Index + 1) % Face.Num()]];
				Normal.X += (A.Y - B.Y) * (A.Z + B.Z);
				Normal.Y += (A.Z - B.Z) * (A.X + B.X);
				Normal.Z += (A.X - B.X) * (A.Y + B.Y);
			}
			if (!Normal.Normalize(UE_DOUBLE_SMALL_NUMBER))
			{
				return false;
			}
			OutN = Normal;
			OutD = Normal | Verts[Face[0]];
			return true;
		}

		/// Seeded sites inside the proxy, quantized to the grid and deduplicated.
		/// Deterministic: FRandomStream sequence + fixed draw order per attempt.
		/// With RadialBias > 0 that fraction of candidates is drawn uniformly from
		/// the ImpactRadius ball around ImpactPoint (rejected like any candidate
		/// when outside the proxy); the RadialBias == 0 path draws exactly as the
		/// uniform-only generator did.
		TArray<FVector> GenerateSites(const FFractureProxy& Proxy, const FFractureParams& Params,
			const TArray<TPair<FVector, double>>& ProxyPlanes)
		{
			FBox Bounds(Proxy.Vertices.GetData(), Proxy.Vertices.Num());

			TArray<FVector> Sites;
			TSet<FIntVector> SeenGrid;
			const auto TryAddSite = [&](FVector Candidate)
			{
				if (Params.FlattenAxis >= 0 && Params.FlattenAxis <= 2)
				{
					Candidate[Params.FlattenAxis] = Bounds.GetCenter()[Params.FlattenAxis];
				}
				for (const TPair<FVector, double>& Plane : ProxyPlanes)
				{
					if ((Plane.Key | Candidate) - Plane.Value > ClipEpsilon)
					{
						return;
					}
				}
				const FVector Site = Quantize(Candidate);
				const FIntVector GridKey(
					int32(FMath::RoundToDouble(Site.X / QuantizeStep)),
					int32(FMath::RoundToDouble(Site.Y / QuantizeStep)),
					int32(FMath::RoundToDouble(Site.Z / QuantizeStep)));
				bool bAlreadySeen = false;
				SeenGrid.Add(GridKey, &bAlreadySeen);
				if (!bAlreadySeen)
				{
					Sites.Add(Site);
				}
			};

			if (!Params.Sites.IsEmpty())
			{
				Sites.Reserve(Params.Sites.Num());
				for (const FVector& Candidate : Params.Sites)
				{
					TryAddSite(Candidate);
				}
				return Sites;
			}

			FRandomStream Stream(Params.Seed);
			const double Bias = FMath::Clamp(Params.RadialBias, 0.0, 1.0);
			const double ImpactRadius = FMath::Max(Params.ImpactRadius, QuantizeStep);
			Sites.Reserve(Params.CellCount);

			const int32 MaxAttempts = Params.CellCount * 256;
			for (int32 Attempt = 0; Attempt < MaxAttempts && Sites.Num() < Params.CellCount; ++Attempt)
			{
				FVector Candidate;
				if (Bias > 0.0 && double(Stream.FRand()) < Bias)
				{
					// Uniform in the impact ball: cube-root radius distribution.
					const double Radius = ImpactRadius * FMath::Pow(double(Stream.FRand()), 1.0 / 3.0);
					Candidate = Params.ImpactPoint + FVector(Stream.GetUnitVector()) * Radius;
				}
				else
				{
					Candidate = FVector(
						Stream.FRandRange(float(Bounds.Min.X), float(Bounds.Max.X)),
						Stream.FRandRange(float(Bounds.Min.Y), float(Bounds.Max.Y)),
						Stream.FRandRange(float(Bounds.Min.Z), float(Bounds.Max.Z)));
				}
				TryAddSite(Candidate);
			}
			return Sites;
		}

		int64 ToGrid(double Value)
		{
			return int64(FMath::RoundToDouble(Value / QuantizeStep));
		}

		uint32 HashBytes(uint32 Hash, const void* Data, int32 Count)
		{
			return b3Hash(Hash, static_cast<const uint8_t*>(Data), Count);
		}

		uint32 HashInt32(uint32 Hash, int32 Value)
		{
			return HashBytes(Hash, &Value, sizeof(Value));
		}

		uint32 HashInt64(uint32 Hash, int64 Value)
		{
			return HashBytes(Hash, &Value, sizeof(Value));
		}

		uint32 HashVectorQuantized(uint32 Hash, const FVector& V)
		{
			Hash = HashInt64(Hash, ToGrid(V.X));
			Hash = HashInt64(Hash, ToGrid(V.Y));
			return HashInt64(Hash, ToGrid(V.Z));
		}
	}

	FFractureProxy MakeBoxProxy(const FVector& Center, const FVector& HalfExtents)
	{
		const FVector& H = HalfExtents;
		FFractureProxy Proxy;
		Proxy.Vertices = {
			Center + FVector(-H.X, -H.Y, -H.Z), Center + FVector(+H.X, -H.Y, -H.Z),
			Center + FVector(+H.X, +H.Y, -H.Z), Center + FVector(-H.X, +H.Y, -H.Z),
			Center + FVector(-H.X, -H.Y, +H.Z), Center + FVector(+H.X, -H.Y, +H.Z),
			Center + FVector(+H.X, +H.Y, +H.Z), Center + FVector(-H.X, +H.Y, +H.Z),
		};
		Proxy.Faces = {
			{ 0, 3, 2, 1 }, // -Z
			{ 4, 5, 6, 7 }, // +Z
			{ 0, 1, 5, 4 }, // -Y
			{ 2, 3, 7, 6 }, // +Y
			{ 0, 4, 7, 3 }, // -X
			{ 1, 2, 6, 5 }, // +X
		};
		return Proxy;
	}

	bool Fracture(const FFractureProxy& Proxy, const FFractureParams& Params,
		TArray<FBox3DFragmentData>& OutFragments)
	{
		OutFragments.Reset();
		if (Params.CellCount < 1 || Proxy.Vertices.Num() < 4 || Proxy.Faces.Num() < 4)
		{
			return false;
		}

		TArray<TPair<FVector, double>> ProxyPlanes;
		ProxyPlanes.Reserve(Proxy.Faces.Num());
		for (const TArray<int32>& Face : Proxy.Faces)
		{
			FVector PlaneN;
			double PlaneD;
			if (Face.Num() < 3 || !MakeFacePlane(Proxy.Vertices, Face, PlaneN, PlaneD))
			{
				return false;
			}
			ProxyPlanes.Add({ PlaneN, PlaneD });
		}

		const TArray<FVector> Sites = GenerateSites(Proxy, Params, ProxyPlanes);
		if (Sites.IsEmpty())
		{
			return false;
		}

		// Clip every site's Voronoi cell out of the proxy. The bisector plane per
		// site pair is computed once, canonically from the lower site index, and
		// used exactly complementarily by the pair — no gaps or overlaps, so the
		// fragment volumes sum to the proxy volume even with quantized offsets.
		TArray<FBox3DFragmentData> Cells;
		TArray<int32> CellToFragment;
		Cells.Reserve(Sites.Num());
		CellToFragment.Init(INDEX_NONE, Sites.Num());
		int32 FragmentCount = 0;

		for (int32 SiteIndex = 0; SiteIndex < Sites.Num(); ++SiteIndex)
		{
			FPoly Cell = MakePoly(Proxy);
			bool bAlive = true;
			for (int32 Other = 0; Other < Sites.Num() && bAlive; ++Other)
			{
				if (Other == SiteIndex)
				{
					continue;
				}
				const int32 Low = FMath::Min(SiteIndex, Other);
				const int32 High = FMath::Max(SiteIndex, Other);
				const FVector Delta = Sites[High] - Sites[Low];
				const double Length = Delta.Length();
				if (Length <= UE_DOUBLE_SMALL_NUMBER)
				{
					continue; // deduplicated sites make this unreachable
				}
				const FVector PlaneN = Delta / Length;
				const double PlaneD = Quantize(PlaneN | ((Sites[Low] + Sites[High]) * 0.5));
				// The cell keeps the side closer to its own site.
				bAlive = (SiteIndex == Low)
					? ClipByPlane(Cell, PlaneN, PlaneD, Other)
					: ClipByPlane(Cell, -PlaneN, -PlaneD, Other);
			}
			if (!bAlive)
			{
				Cells.AddDefaulted();
				continue;
			}

			FBox3DFragmentData Fragment = FinalizeFragment(Cell);
			if (Fragment.Volume <= UE_DOUBLE_KINDA_SMALL_NUMBER || Fragment.Vertices.Num() < 4)
			{
				Cells.AddDefaulted();
				continue;
			}
			CellToFragment[SiteIndex] = FragmentCount++;
			Cells.Add(MoveTemp(Fragment));
		}

		// Remap face tags from site indices to output fragment indices and derive
		// the sorted neighbor list. A face whose neighbor cell collapsed becomes
		// exterior.
		OutFragments.Reserve(FragmentCount);
		for (int32 SiteIndex = 0; SiteIndex < Sites.Num(); ++SiteIndex)
		{
			if (CellToFragment[SiteIndex] == INDEX_NONE)
			{
				continue;
			}
			FBox3DFragmentData& Fragment = Cells[SiteIndex];
			for (FBox3DFragmentFace& Face : Fragment.Faces)
			{
				if (Face.NeighborIndex != INDEX_NONE)
				{
					Face.NeighborIndex = CellToFragment[Face.NeighborIndex];
				}
				if (Face.NeighborIndex != INDEX_NONE)
				{
					Fragment.Neighbors.Add({ Face.NeighborIndex, Face.Area });
				}
			}
			Fragment.Neighbors.Sort([](const FBox3DFragmentNeighbor& A, const FBox3DFragmentNeighbor& B)
			{
				return A.FragmentIndex < B.FragmentIndex;
			});
			OutFragments.Add(MoveTemp(Fragment));
		}

		// Symmetrize adjacency. Quantized bisector offsets make the cells a near-
		// Voronoi partition: on the shared plane of pair (i, j) the third-party
		// planes (i, k) and (j, k) cut at slightly different lines, so the two
		// cells' shared-face polygons differ by sub-QuantizeStep slivers. The
		// lower-indexed fragment's area is canonical for the pair; an orphaned
		// sliver whose counterpart collapsed entirely is demoted to exterior.
		for (int32 Index = 0; Index < OutFragments.Num(); ++Index)
		{
			FBox3DFragmentData& Fragment = OutFragments[Index];
			for (int32 Slot = Fragment.Neighbors.Num() - 1; Slot >= 0; --Slot)
			{
				const FBox3DFragmentNeighbor& Link = Fragment.Neighbors[Slot];
				if (Link.FragmentIndex < Index)
				{
					continue; // handled from the lower-indexed side
				}
				FBox3DFragmentNeighbor* Back = OutFragments[Link.FragmentIndex].Neighbors.FindByPredicate(
					[Index](const FBox3DFragmentNeighbor& Candidate)
					{
						return Candidate.FragmentIndex == Index;
					});
				if (Back != nullptr)
				{
					Back->SharedFaceArea = Link.SharedFaceArea;
				}
				else
				{
					for (FBox3DFragmentFace& Face : Fragment.Faces)
					{
						if (Face.NeighborIndex == Link.FragmentIndex)
						{
							Face.NeighborIndex = INDEX_NONE;
							break;
						}
					}
					Fragment.Neighbors.RemoveAt(Slot);
				}
			}
		}

		if (Params.MinFragmentVolume > 0.0 && OutFragments.Num() > 1)
		{
			MergeSmallFragments(OutFragments, Params.MinFragmentVolume);
		}

		return !OutFragments.IsEmpty();
	}

	uint32 FractureLayoutHash(const TArray<FBox3DFragmentData>& Fragments)
	{
		uint32 Hash = B3_HASH_INIT;
		Hash = HashInt32(Hash, Fragments.Num());
		for (const FBox3DFragmentData& Fragment : Fragments)
		{
			Hash = HashInt32(Hash, Fragment.Vertices.Num());
			for (const FVector& Vertex : Fragment.Vertices)
			{
				Hash = HashVectorQuantized(Hash, Vertex);
			}
			Hash = HashInt32(Hash, Fragment.Faces.Num());
			for (const FBox3DFragmentFace& Face : Fragment.Faces)
			{
				Hash = HashInt32(Hash, Face.VertexIndices.Num());
				Hash = HashBytes(Hash, Face.VertexIndices.GetData(),
					Face.VertexIndices.Num() * sizeof(int32));
				Hash = HashInt32(Hash, Face.NeighborIndex);
				Hash = HashInt64(Hash, ToGrid(Face.Area));
			}
			Hash = HashInt64(Hash, ToGrid(Fragment.Volume));
			Hash = HashVectorQuantized(Hash, Fragment.Centroid);
			Hash = HashInt32(Hash, Fragment.Neighbors.Num());
			for (const FBox3DFragmentNeighbor& Neighbor : Fragment.Neighbors)
			{
				Hash = HashInt32(Hash, Neighbor.FragmentIndex);
				Hash = HashInt64(Hash, ToGrid(Neighbor.SharedFaceArea));
			}
		}
		return Hash;
	}
}
