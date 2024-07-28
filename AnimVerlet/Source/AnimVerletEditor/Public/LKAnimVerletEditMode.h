#pragma once
#include <AnimNodeEditMode.h>
#include <CoreMinimal.h>
#include "LKAnimGraphNode_AnimVerlet.h"
#include "LKAnimNode_AnimVerlet.h"
#include "LKAnimVerletType.h"

class FLKAnimVerletEditMode : public IAnimNodeEditMode
{
public:
	/// FEdMode =====================================================================================================================================
	virtual void Render(const class FSceneView* View, class FViewport* Viewport, class FPrimitiveDrawInterface* PDI) override;
	virtual void DrawHUD(class FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas) override;
	///==============================================================================================================================================
	
	/// IAnimationEditContext =======================================================================================================================
	virtual bool GetCameraTarget(FSphere& OutTarget) const override { return false; }
	virtual class IPersonaPreviewScene& GetAnimPreviewScene() const override;
	virtual void GetOnScreenDebugInfo(TArray<FText>& OutDebugInfo) const override;
	///==============================================================================================================================================
	
	/// IAnimNodeEditMode ===========================================================================================================================
	virtual ECoordSystem GetWidgetCoordinateSystem() const override { return ECoordSystem::COORD_Local; }
	virtual LK_UEWIDGET::EWidgetMode GetWidgetMode() const override { return LK_UEWIDGET::WM_None; }
	virtual LK_UEWIDGET::EWidgetMode ChangeToNextWidgetMode(LK_UEWIDGET::EWidgetMode CurWidgetMode) override { return LK_UEWIDGET::WM_None; }
	virtual bool SetWidgetMode(LK_UEWIDGET::EWidgetMode InWidgetMode) override { return false; }
	virtual FName GetSelectedBone() const override { return NAME_None; }
	virtual void DoTranslation(FVector& InTranslation) override {}
	virtual void DoRotation(FRotator& InRotation) override {}
	virtual void DoScale(FVector& InScale) override {}
	virtual void EnterMode(class UAnimGraphNode_Base* InEditorNode, struct FAnimNode_Base* InRuntimeNode) override;
	virtual void ExitMode() override;
	virtual bool SupportsPoseWatch() override { return false; }
	virtual void RegisterPoseWatchedNode(UAnimGraphNode_Base* InEditorNode, FAnimNode_Base* InRuntimeNode) override {}
	///==============================================================================================================================================

private:
	class UAnimGraphNode_Base* AnimNode = nullptr;
	struct FAnimNode_Base* RuntimeAnimNode = nullptr;
};