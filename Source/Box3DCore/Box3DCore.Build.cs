// Box3D physics library (https://github.com/erincatto/box3d) compiled from
// vendored source. Upstream files are unmodified; see UPSTREAM.md.

using UnrealBuildTool;

public class Box3DCore : ModuleRules
{
	public Box3DCore(ReadOnlyTargetRules Target) : base(Target)
	{
		// Upstream is C17 ("_Static_assert and anonymous unions"); the C files
		// build standalone, so PCH/unity machinery is disabled.
		CStandard = CStandardVersion.C17;
		PCHUsage = PCHUsageMode.NoPCHs;
		bUseUnity = false;

		// Third-party code compiled at /W4 by UE; keep upstream warnings non-fatal.
		bWarningsAsErrors = false;

		// Large world mode: double-precision world positions (b3Pos/b3WorldTransform
		// split translation, like Jolt's DMat44). ABI-affecting, which is why it is
		// a PublicDefinition — UBT propagates it to every dependent module so both
		// sides of the API always agree. Flip and rebuild; the Box3DRuntime
		// conversion seam (Box3DConversion.h) keeps UE's LWC precision through it.
		bool bDoublePrecision = false;
		if (bDoublePrecision)
		{
			PublicDefinitions.Add("BOX3D_DOUBLE_PRECISION=1");
		}

		PrivateDependencyModuleNames.Add("Core");

		// In modular (editor) builds this module is its own DLL, so the C API must
		// cross the DLL boundary. UBT's BOX3DCORE_API can't be used: it expands to
		// UE's DLLEXPORT macro from Platform.h, which the C sources never include.
		// Use box3d's native scheme instead (base.h checks box3d_EXPORTS before
		// BOX3D_DLL, so the module itself exports while consumers import).
		if (Target.LinkType == TargetLinkType.Modular)
		{
			PrivateDefinitions.Add("box3d_EXPORTS");
			PublicDefinitions.Add("BOX3D_DLL");
		}

		// UE defines NDEBUG outside Debug configs, which would strip B3_ASSERT and
		// the exported b3InternalAssert. Keep asserts (routed to our hook) everywhere
		// but Shipping. Public: consumers must see the same B3_ASSERT/b3InternalAssert
		// state as the library.
		if (Target.Configuration != UnrealTargetConfiguration.Shipping)
		{
			PublicDefinitions.Add("B3_ENABLE_ASSERT");
		}
	}
}
