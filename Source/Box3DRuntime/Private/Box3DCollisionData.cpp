#include "Box3DCollisionData.h"

#include "Misc/PackageName.h"
#include "box3d/base.h"

#if WITH_EDITOR
#include "Engine/Level.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#endif

FString UBox3DCollisionData::DeriveAssetPackageName(const FString& MapPackageName)
{
	const FString Path = FPackageName::GetLongPackagePath(MapPackageName);
	const FString Name = FPackageName::GetShortName(MapPackageName);
	return FString::Printf(TEXT("%s/BC_%s"), *Path, *Name);
}

#if WITH_EDITOR

FString UBox3DCollisionData::ComputeSourceFingerprint(const FString& MapPackageName)
{
	FString MapFilename;
	if (!FPackageName::TryConvertLongPackageNameToFilename(
			MapPackageName, MapFilename, FPackageName::GetMapPackageExtension()))
	{
		return FString();
	}

	IFileManager& FileManager = IFileManager::Get();
	if (!FileManager.FileExists(*MapFilename))
	{
		return FString();
	}

	FDateTime NewestTime = FileManager.GetTimeStamp(*MapFilename);
	int64 TotalSize = FileManager.FileSize(*MapFilename);
	int32 FileCount = 1;

	// One File Per Actor: every actor lives in its own package under
	// __ExternalActors__/<map path>; edits there never touch the .umap.
	const FString ExternalActorsPath = ULevel::GetExternalActorsPath(MapPackageName);
	FString ExternalActorsDir;
	if (FPackageName::TryConvertLongPackageNameToFilename(ExternalActorsPath, ExternalActorsDir) &&
		FileManager.DirectoryExists(*ExternalActorsDir))
	{
		FileManager.IterateDirectoryRecursively(*ExternalActorsDir,
			[&](const TCHAR* Path, bool bIsDirectory)
			{
				if (!bIsDirectory)
				{
					NewestTime = FMath::Max(NewestTime, FileManager.GetTimeStamp(Path));
					TotalSize += FileManager.FileSize(Path);
					++FileCount;
				}
				return true;
			});
	}

	return FString::Printf(TEXT("%s|%lld|%d"), *NewestTime.ToIso8601(), TotalSize, FileCount);
}

bool UBox3DCollisionData::IsStale(FString& OutReason) const
{
	const b3Version Version = b3GetVersion();
	const FString Current = FString::Printf(TEXT("%d.%d.%d"), Version.major, Version.minor, Version.revision);
	if (Box3DVersion != Current)
	{
		OutReason = FString::Printf(TEXT("baked with box3d %s, running %s"), *Box3DVersion, *Current);
		return true;
	}

	if (SourceFingerprint.IsEmpty())
	{
		OutReason = TEXT("bake predates source fingerprinting");
		return true;
	}

	const FString Now = ComputeSourceFingerprint(SourceLevel);
	if (!Now.IsEmpty() && Now != SourceFingerprint)
	{
		OutReason = FString::Printf(TEXT("source level %s changed since the bake"), *SourceLevel);
		return true;
	}

	return false;
}

#endif // WITH_EDITOR
