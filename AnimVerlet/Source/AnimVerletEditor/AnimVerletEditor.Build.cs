using UnrealBuildTool;

public class AnimVerletEditor : ModuleRules
{
	public AnimVerletEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PrivateDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "AnimGraphRuntime", "AnimGraph", "BlueprintGraph", "Slate", "SlateCore", "Persona", "UnrealEd", "AnimVerlet" });
	}
}
