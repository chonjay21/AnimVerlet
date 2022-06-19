#include "LKAnimVerletEditMode.h"

#include <AnimationRuntime.h>
///#include "SceneManagement.h"
///#include "EngineUtils.h"
#include "IPersonaPreviewScene.h"
#include "Animation/DebugSkelMeshComponent.h"
#include "AssetEditorModeManager.h"
///#include "EditorModeManager.h"
///#include "CanvasItem.h"
///#include "CanvasTypes.h"
///#include "Materials/MaterialInstanceDynamic.h"

void FLKAnimVerletEditMode::EnterMode(UAnimGraphNode_Base* InEditorNode, FAnimNode_Base* InRuntimeNode)
{
	AnimNode = InEditorNode;
	RuntimeAnimNode = InRuntimeNode;
}

void FLKAnimVerletEditMode::ExitMode()
{
	AnimNode = nullptr;
	RuntimeAnimNode = nullptr;
}

void FLKAnimVerletEditMode::Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI)
{
	if (AnimNode != nullptr)
	{
		AnimNode->Draw(PDI, GetAnimPreviewScene().GetPreviewMeshComponent());
	}
}

void FLKAnimVerletEditMode::DrawHUD(FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas)
{
	if (AnimNode != nullptr)
	{
		AnimNode->DrawCanvas(*Viewport, *const_cast<FSceneView*>(View), *Canvas, GetAnimPreviewScene().GetPreviewMeshComponent());
	}
}

IPersonaPreviewScene& FLKAnimVerletEditMode::GetAnimPreviewScene() const
{
	return *(IPersonaPreviewScene*)(((FAssetEditorModeManager*)Owner)->GetPreviewScene());
}

void FLKAnimVerletEditMode::GetOnScreenDebugInfo(TArray<FText>& OutDebugInfo) const
{
	if (AnimNode != nullptr)
	{
		AnimNode->GetOnScreenDebugInfo(OutDebugInfo, RuntimeAnimNode, GetAnimPreviewScene().GetPreviewMeshComponent());
	}
}