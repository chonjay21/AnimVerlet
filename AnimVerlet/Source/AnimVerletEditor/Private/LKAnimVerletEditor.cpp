#include "LKAnimVerletEditor.h"

#include <EditorModeRegistry.h>
#include <Modules/ModuleManager.h>
#include <Textures/SlateIcon.h>
#include "LKAnimVerletEditMode.h"

IMPLEMENT_MODULE(FLKAnimVerletEditorModule, AnimVerletEditor)

#define LOCTEXT_NAMESPACE "FLKAnimVerletEditorModule"
void FLKAnimVerletEditorModule::StartupModule()
{
	FEditorModeRegistry::Get().RegisterMode<FLKAnimVerletEditMode>("AnimGraph.SkeletalControl.AnimVerlet", LOCTEXT("FLKAnimVerletEditMode", "AnimVerlet"), FSlateIcon(), false);
}



void FLKAnimVerletEditorModule::ShutdownModule()
{
	FEditorModeRegistry::Get().UnregisterMode("AnimGraph.SkeletalControl.AnimVerlet");
}
#undef LOCTEXT_NAMESPACE