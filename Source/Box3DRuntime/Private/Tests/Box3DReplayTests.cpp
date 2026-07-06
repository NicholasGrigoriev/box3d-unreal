// M6 determinism & replay: box3d records every world mutation with embedded
// per-step state hashes; validation replays in a scratch world and compares them.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "HAL/FileManager.h"
#include "Misc/Paths.h"

namespace
{
	void SpawnStack(UWorld* World, const FVector& Base, int32 Count)
	{
		for (int32 Index = 0; Index < Count; ++Index)
		{
			Box3DTest::SpawnBody(World, Base + FVector(0, 0, 60.0f + Index * 110.0f), [](UBox3DBodyComponent& Body)
			{
				Body.BodyType = EBox3DBodyType::Dynamic;
				Body.ShapeType = EBox3DShapeType::Box;
				Body.BoxHalfExtent = FVector(50.0);
			});
		}
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DReplayRecordAndValidateTest,
	"Box3DUnreal.Replay.RecordAndValidate", BOX3D_TEST_FLAGS)
bool FBox3DReplayRecordAndValidateTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UBox3DWorldSubsystem& Subsystem = Test.Subsystem();

	// Pre-recording content lands in the seed snapshot...
	Box3DTest::SpawnGround(Test.World);
	SpawnStack(Test.World, FVector::ZeroVector, 3);

	TestFalse(TEXT("not recording initially"), Subsystem.IsRecording());
	TestTrue(TEXT("StartRecording"), Subsystem.StartRecording());
	TestTrue(TEXT("IsRecording while active"), Subsystem.IsRecording());

	// ...mid-recording mutations land in the op stream.
	Test.Step(20);
	Box3DTest::SpawnBody(Test.World, FVector(200, 0, 300), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Sphere;
		Body.SphereRadius = 25.0f;
	});
	Test.Step(40);

	TestTrue(TEXT("StopRecording"), Subsystem.StopRecording());
	TestFalse(TEXT("not recording after stop"), Subsystem.IsRecording());

	const b3Recording* Recording = Subsystem.GetRecording();
	TestNotNull(TEXT("recording buffer exists"), Recording);
	const int32 Size = b3Recording_GetSize(Recording);
	TestTrue(TEXT("recording captured bytes"), Size > 0);

	// The determinism check: replay in a scratch world, compare embedded hashes.
	TestTrue(TEXT("replay reproduces identical state hashes"), Subsystem.ValidateLastRecording());

	// File roundtrip: save, reload, and validate the loaded copy too.
	const FString Path = FPaths::ProjectSavedDir() / TEXT("Box3D") / TEXT("automation_roundtrip.b3r");
	TestTrue(TEXT("SaveRecordingToFile"), Subsystem.SaveRecordingToFile(Path));

	b3Recording* Loaded = b3LoadRecordingFromFile(TCHAR_TO_UTF8(*Path));
	TestNotNull(TEXT("recording loads back from disk"), Loaded);
	if (Loaded != nullptr)
	{
		TestEqual(TEXT("loaded size matches"), b3Recording_GetSize(Loaded), Size);
		TestTrue(TEXT("loaded recording validates"),
			b3ValidateReplay(b3Recording_GetData(Loaded), b3Recording_GetSize(Loaded), 1));
		b3DestroyRecording(Loaded);
	}
	IFileManager::Get().Delete(*Path);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DReplayCrossWorkerDeterminismTest,
	"Box3DUnreal.Replay.CrossWorkerDeterminism", BOX3D_TEST_FLAGS)
bool FBox3DReplayCrossWorkerDeterminismTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UBox3DWorldSubsystem& Subsystem = Test.Subsystem();

	// Contact-heavy content so the replay world's constraint graph has real work
	// to partition differently at a different worker count.
	Box3DTest::SpawnGround(Test.World);
	SpawnStack(Test.World, FVector(-150, -150, 0), 3);
	SpawnStack(Test.World, FVector(150, -150, 0), 3);
	SpawnStack(Test.World, FVector(-150, 150, 0), 3);

	TestTrue(TEXT("StartRecording"), Subsystem.StartRecording());
	SpawnStack(Test.World, FVector(150, 150, 0), 3); // recorded creation ops
	Test.Step(60);
	TestTrue(TEXT("StopRecording"), Subsystem.StopRecording());

	// Replay at 4 workers what was recorded single-threaded. box3d re-partitions
	// the constraint graph, so the per-step hash comparison IS the cross-thread
	// determinism test.
	const b3Recording* Recording = Subsystem.GetRecording();
	b3RecPlayer* Player = b3RecPlayer_Create(b3Recording_GetData(Recording), b3Recording_GetSize(Recording), 4);
	TestNotNull(TEXT("player created over the recording"), Player);
	if (Player == nullptr)
	{
		return false;
	}

	const b3RecPlayerInfo Info = b3RecPlayer_GetInfo(Player);
	TestEqual(TEXT("recorded frame count"), Info.frameCount, 60);
	TestEqual(TEXT("recorded sub-steps"), Info.subStepCount, GetDefault<UBox3DSettings>()->SubStepCount);
	TestTrue(TEXT("recorded dt matches the fixed step"),
		FMath::IsNearlyEqual(Info.timeStep, Box3DTest::FTestWorld::FixedDt(), 1.0e-6f));

	const b3WorldId ReplayWorld = b3RecPlayer_GetWorldId(Player);
	TestTrue(TEXT("player drives its own world"), b3World_IsValid(ReplayWorld));

	int32 FramesStepped = 0;
	while (b3RecPlayer_StepFrame(Player))
	{
		++FramesStepped;
	}

	TestEqual(TEXT("player stepped every recorded frame"), FramesStepped, 60);
	TestTrue(TEXT("op stream exhausted"), b3RecPlayer_IsAtEnd(Player));
	TestEqual(TEXT("bodies tracked (ground + 12 boxes)"), b3RecPlayer_GetBodyCount(Player), 13);
	TestFalse(TEXT("replay at 4 workers never diverged from the serial recording"),
		b3RecPlayer_HasDiverged(Player));
	TestEqual(TEXT("no diverge frame"), b3RecPlayer_GetDivergeFrame(Player), -1);

	b3RecPlayer_Destroy(Player);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
