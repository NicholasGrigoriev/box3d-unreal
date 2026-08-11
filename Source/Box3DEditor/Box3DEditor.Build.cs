using UnrealBuildTool;

public class Box3DEditor : ModuleRules
{
	public Box3DEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"AssetRegistry",
			"Box3DCore",
			"Box3DRuntime",
			"CoreUObject",
			"Engine",
			"PhysicsCore",
			"RenderCore",
			"RHI",
			"UnrealEd",
		});
	}
}
