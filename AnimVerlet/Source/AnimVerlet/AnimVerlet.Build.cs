using UnrealBuildTool;

public class AnimVerlet : ModuleRules
{
	public AnimVerlet(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PrivateDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "AnimGraphRuntime" });
	}
}
