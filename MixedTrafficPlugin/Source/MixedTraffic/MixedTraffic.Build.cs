// Copyright QLwin. All Rights Reserved.

using UnrealBuildTool;

public class MixedTraffic : ModuleRules
{
	public MixedTraffic(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"DeveloperSettings",   // UDeveloperSettings base class
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			// Add private-only dependencies here
		});

		// Automation tests are compiled only in editor/development builds
		if (Target.bBuildEditor || Target.Configuration != UnrealTargetConfiguration.Shipping)
		{
			PrivateDependencyModuleNames.Add("AutomationController");
		}
	}
}
