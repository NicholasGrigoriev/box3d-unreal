using UnrealBuildTool;

public class Box3DRuntime : ModuleRules
{
	public Box3DRuntime(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"Box3DCore",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"CoreUObject",
			"Engine",
			"DeveloperSettings",
			"PhysicsCore",
			"ProceduralMeshComponent",
			"MeshDescription",
			"StaticMeshDescription",
			"Niagara",
			"RenderCore",
			"RHI",
		});
	}
}
