#include "Box3DRuntime.h"

#include "Modules/ModuleManager.h"
#include "box3d/box3d.h"

DEFINE_LOG_CATEGORY(LogBox3D);

namespace
{
	void* Box3DMalloc(int32_t Size, int32_t Alignment)
	{
		return FMemory::Malloc(static_cast<SIZE_T>(Size), static_cast<uint32>(Alignment));
	}

	void Box3DFree(void* Mem)
	{
		FMemory::Free(Mem);
	}

	int Box3DAssert(const char* Condition, const char* FileName, int LineNumber)
	{
		UE_LOG(LogBox3D, Error, TEXT("Box3D assertion failed: %hs (%hs:%d)"), Condition, FileName, LineNumber);
		// Non-zero requests B3_BREAKPOINT; only useful when a debugger is attached.
		return FPlatformMisc::IsDebuggerPresent() ? 1 : 0;
	}

	void Box3DLog(const char* Message)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%hs"), Message);
	}
}

class FBox3DRuntimeModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		// Route the library's global hooks into UE before any world exists.
		b3SetAllocator(&Box3DMalloc, &Box3DFree);
		b3SetAssertFcn(&Box3DAssert);
		b3SetLogFcn(&Box3DLog);

		const b3Version Version = b3GetVersion();
		UE_LOG(LogBox3D, Log, TEXT("Box3D %d.%d.%d initialized (%s precision)"),
			Version.major, Version.minor, Version.revision,
			b3IsDoublePrecision() ? TEXT("double") : TEXT("single"));
	}

	virtual void ShutdownModule() override
	{
		if (const int32 LeakedBytes = b3GetByteCount(); LeakedBytes != 0)
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D shutdown with %d bytes still allocated"), LeakedBytes);
		}
	}
};

IMPLEMENT_MODULE(FBox3DRuntimeModule, Box3DRuntime)
