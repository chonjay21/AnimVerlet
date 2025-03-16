#pragma once
#include <AnimNodeEditMode.h>
#include <CoreMinimal.h>
#include "LKAnimGraphNode_AnimVerlet.h"
#include "LKAnimNode_AnimVerlet.h"
#include "LKAnimVerletCollisionPhysicsAsset.h"
#include "LKAnimVerletType.h"

///=========================================================================================================================================
/// ELKAnimVerletColliderSource
///=========================================================================================================================================
UENUM()
enum class ELKAnimVerletColliderSource : uint8
{
	None,

	Node,
	DataAsset,
	PhysicsAsset,

	Count
};

///=========================================================================================================================================
/// FLKAnimVerletEditModeSelection
///=========================================================================================================================================
struct FLKAnimVerletEditModeSelection
{
public:
	ELKAnimVerletCollider ColliderType = ELKAnimVerletCollider::None;
	ELKAnimVerletColliderSource ColliderSourceType = ELKAnimVerletColliderSource::None;
	int32 ColliderListIndex = INDEX_NONE;
	int32 OptionalBodySetupIndex = INDEX_NONE;

public:
	void Select(ELKAnimVerletCollider InColliderType, ELKAnimVerletColliderSource InColliderSourceType, int32 InColliderListIndex, int32 InOptionalBodySetupIndex)
	{
		ColliderType = InColliderType;
		ColliderSourceType = InColliderSourceType;
		ColliderListIndex = InColliderListIndex;
		OptionalBodySetupIndex = InOptionalBodySetupIndex;
	}

	bool IsSelected() const { return ColliderType != ELKAnimVerletCollider::None && ColliderListIndex != INDEX_NONE; }
	void ResetSelection()
	{
		ColliderType = ELKAnimVerletCollider::None;
		ColliderSourceType = ELKAnimVerletColliderSource::None;
		ColliderListIndex = INDEX_NONE;
		OptionalBodySetupIndex = INDEX_NONE;
	}
};


///=========================================================================================================================================
/// FLKAnimVerletEditMode
///=========================================================================================================================================
class FLKAnimVerletEditMode : public FAnimNodeEditMode
{
public:
	/// FEdMode =====================================================================================================================================
	virtual void Render(const class FSceneView* View, class FViewport* Viewport, class FPrimitiveDrawInterface* PDI) override;
	virtual void DrawHUD(class FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas) override;
	virtual bool HandleClick(class FEditorViewportClient* InViewportClient, HHitProxy* HitProxy, const FViewportClick& Click) override;
	virtual bool GetCustomDrawingCoordinateSystem(FMatrix& InMatrix, void* InData) override;
	virtual bool ShouldDrawWidget() const override;
	///==============================================================================================================================================
	
	/// IAnimationEditContext =======================================================================================================================
	virtual bool GetCameraTarget(FSphere& OutTarget) const override { return false; }
	virtual class IPersonaPreviewScene& GetAnimPreviewScene() const override;
	virtual void GetOnScreenDebugInfo(TArray<FText>& OutDebugInfo) const override;
	///==============================================================================================================================================
	
	/// IAnimNodeEditMode ===========================================================================================================================
	/** Returns the coordinate system that should be used for this bone */
	virtual ECoordSystem GetWidgetCoordinateSystem() const override { return ECoordSystem::COORD_World; }
	/** @return current widget mode this anim graph node supports */
	virtual LK_UEWIDGET::EWidgetMode GetWidgetMode() const override;
	/** Called when the user changed widget mode by pressing "Space" key */
	virtual LK_UEWIDGET::EWidgetMode ChangeToNextWidgetMode(LK_UEWIDGET::EWidgetMode CurWidgetMode) override;
	/** Called when the user set widget mode directly, returns true if InWidgetMode is available */
	virtual bool SetWidgetMode(LK_UEWIDGET::EWidgetMode InWidgetMode) override;
	virtual FVector GetWidgetLocation() const override;
	virtual void DoTranslation(FVector& InTranslation) override;
	virtual void DoRotation(FRotator& InRotation) override;
	virtual void DoScale(FVector& InScale) override;
	virtual void EnterMode(class UAnimGraphNode_Base* InEditorNode, struct FAnimNode_Base* InRuntimeNode) override;
	virtual void ExitMode() override;
	virtual bool SupportsPoseWatch() override { return false; }
	virtual void RegisterPoseWatchedNode(UAnimGraphNode_Base* InEditorNode, FAnimNode_Base* InRuntimeNode) override {}
	///==============================================================================================================================================

private:
	void RenderSphereColliders(class FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent);
	void RenderCapsuleColliders(class FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent);
	void RenderBoxColliders(class FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent);
	void RenderPlaneColliders(class FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent);
	void RenderText(const class FViewport* Viewport, const class FSceneView* View, class FCanvas* Canvas, const FVector& Location, const FText& Text);

	bool ValidateSelection(bool bGraphNode) const;
	struct FLKAnimVerletColliderInterface* GetSelectedColliderFromGraphNode(bool bMarkDirty) const;				///Editor Graph setting
	struct FLKAnimVerletColliderInterface* GetSelectedColliderFromRuntime(bool bMarkDirty) const;				///Preview or real runtime setting

	bool ConvertBoneTToWorldT(OUT FTransform& OutWorldT, const USkeletalMeshComponent* SkeletalMeshComponent, const FName& BoneName) const;
	FTransform GetColliderWorldT(const struct FLKAnimVerletColliderInterface& InCollider, const USkeletalMeshComponent* SkeletalMeshComponent) const;
	FTransform GetColliderWorldT(const struct FLKAnimVerletColliderInterface& InCollider) const;

	FVector ConvertCSVectorToBoneSpaceNoScale(const USkeletalMeshComponent* SkelComp, FVector& InCSVector, FCSPose<FCompactHeapPose>& MeshBases, const FName& BoneName, const EBoneControlSpace Space);
	FQuat ConvertCSRotationToBoneSpaceNoScale(const USkeletalMeshComponent* SkelComp, FRotator& InCSRotator, FCSPose<FCompactHeapPose>& MeshBases, const FName& BoneName, const EBoneControlSpace Space);

private:
	class UAnimGraphNode_Base* AnimNode = nullptr;
	struct FAnimNode_Base* RuntimeAnimNode = nullptr;
	class ULKAnimGraphNode_AnimVerlet* CachedAnimVerletGraphNode = nullptr;
	struct FLKAnimNode_AnimVerlet* CachedAnimVerletNode = nullptr;

	LK_UEWIDGET::EWidgetMode CurrentWidgetMode = LK_UEWIDGET::EWidgetMode::WM_Translate;
	mutable FLKAnimVerletEditModeSelection SelectedElement;

	mutable FLKAnimVerletCollisionPASphere PASphereRuntimeSpace;
	mutable FLKAnimVerletCollisionPASphere PASphereGraphNodeSpace;
	mutable FLKAnimVerletCollisionPACapsule PACapsuleRuntimeSpace;
	mutable FLKAnimVerletCollisionPACapsule PACapsuleGraphNodeSpace;
	mutable FLKAnimVerletCollisionPABox PABoxRuntimeSpace;
	mutable FLKAnimVerletCollisionPABox PABoxGraphNodeSpace;
};