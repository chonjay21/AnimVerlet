using UnrealBuildTool;

public class AnimVerletEditor : ModuleRules
{
	public AnimVerletEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PrivateDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "AnimGraphRuntime", "AnimGraph", "BlueprintGraph", "Slate", "SlateCore", "Persona", "UnrealEd", "AnimVerlet" });
		
		/// UE5 compatibility
		BuildVersion Version;
        if (BuildVersion.TryRead(BuildVersion.GetDefaultFileName(), out Version))
        {
            if (Version.MajorVersion == 5)
				PrivateDependencyModuleNames.AddRange(new string[] { "EditorFramework" });
        }
	}
}
