// Copyright QLwin. All Rights Reserved.

using UnrealBuildTool;

public class MixedTrafficEditor : ModuleRules
{
	public MixedTrafficEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"MixedTraffic",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"UnrealEd",
			"AssetTools",
			"PropertyEditor",
			"EditorFramework",
		});
	}
}
