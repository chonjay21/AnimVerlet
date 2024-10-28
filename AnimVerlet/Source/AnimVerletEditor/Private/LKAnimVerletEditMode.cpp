#include "LKAnimVerletEditMode.h"

#include <AnimationRuntime.h>
#include <AssetEditorModeManager.h>
#include <IPersonaPreviewScene.h>
#include <Animation/DebugSkelMeshComponent.h>
#include "LKAnimVerletCollisionData.h"

struct FLKAnimVerletEditModeHitProxy : HHitProxy
{
	DECLARE_HIT_PROXY()

public:
	FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider InColliderType, bool bInColliderFromDA, int32 InColliderListIndex)
		: HHitProxy(HPP_Foreground)
	{
		Selection.Select(InColliderType, bInColliderFromDA, InColliderListIndex);
	}

	virtual EMouseCursor::Type GetMouseCursor() override { return EMouseCursor::Hand; }

public:
	FLKAnimVerletEditModeSelection Selection;
};
IMPLEMENT_HIT_PROXY(FLKAnimVerletEditModeHitProxy, HHitProxy);


void FLKAnimVerletEditMode::Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI)
{
	/*if (AnimNode != nullptr)
	{
		AnimNode->Draw(PDI, GetAnimPreviewScene().GetPreviewMeshComponent());
	}*/

	const USkeletalMeshComponent* PreviewMeshComponent = GetAnimPreviewScene().GetPreviewMeshComponent();
	if (PreviewMeshComponent != nullptr && PreviewMeshComponent->GetSkeletalMeshAsset() != nullptr)
	{
		RenderSphereColliders(PDI, PreviewMeshComponent);
		RenderCapsuleColliders(PDI, PreviewMeshComponent);
		RenderBoxColliders(PDI, PreviewMeshComponent);
		RenderPlaneColliders(PDI, PreviewMeshComponent);
	}
	FAnimNodeEditMode::Render(View, Viewport, PDI);
}

void FLKAnimVerletEditMode::RenderSphereColliders(FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent)
{
	if (CachedAnimVerletGraphNode->bShowAndModifySphereCollision == false)
	{
		if (SelectedElement.ColliderType == ELKAnimVerletCollider::Sphere)
			SelectedElement.ResetSelection();
		return;
	}

	for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.SphereCollisionShapes.Num(); i++)
	{
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Sphere, false, i));
		const FLKAnimVerletCollisionSphere& CurShape = CachedAnimVerletGraphNode->Node.SphereCollisionShapes[i];

		const FTransform CollisionWorldT = GetColliderWorldT(CurShape, PreviewMeshComponent);
		const FVector ShapeLocation = CollisionWorldT.GetLocation();
		DrawSphere(PDI, ShapeLocation, FRotator::ZeroRotator, FVector(CurShape.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
		DrawWireSphere(PDI, ShapeLocation, FLinearColor::Red, CurShape.Radius, 16, SDPG_World);
		DrawCoordinateSystem(PDI, ShapeLocation, CollisionWorldT.Rotator(), CurShape.Radius, SDPG_Foreground);

		PDI->SetHitProxy(nullptr);
	}

	if (CachedAnimVerletGraphNode->Node.CollisionDataAsset != nullptr)
	{
		for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData.Num(); i++)
		{
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Sphere, true, i));
			const FLKAnimVerletCollisionDataSphere& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData[i];

			const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
			const FVector ShapeLocation = CollisionWorldT.GetLocation();
			DrawSphere(PDI, ShapeLocation, FRotator::ZeroRotator, FVector(CurShapeData.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawWireSphere(PDI, ShapeLocation, FLinearColor::Red, CurShapeData.Radius, 16, SDPG_World);
			DrawCoordinateSystem(PDI, ShapeLocation, CollisionWorldT.Rotator(), CurShapeData.Radius, SDPG_Foreground);

			PDI->SetHitProxy(nullptr);
		}
	}
}

void FLKAnimVerletEditMode::RenderCapsuleColliders(FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent)
{
	if (CachedAnimVerletGraphNode->bShowAndModifyCapsuleCollision == false)
	{
		if (SelectedElement.ColliderType == ELKAnimVerletCollider::Capsule)
			SelectedElement.ResetSelection();
		return;
	}

	for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CapsuleCollisionShapes.Num(); i++)
	{
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Capsule, false, i));
		const FLKAnimVerletCollisionCapsule& CurShape = CachedAnimVerletGraphNode->Node.CapsuleCollisionShapes[i];

		const FTransform CollisionWorldT = GetColliderWorldT(CurShape, PreviewMeshComponent);
		const FVector ShapeLocation = CollisionWorldT.GetLocation();
		const FQuat ShapeQuat = CollisionWorldT.GetRotation();
		const FRotator ShapeRotator = ShapeQuat.Rotator();

		const FVector CapsuleXAxis = ShapeQuat.GetAxisX();
		const FVector CapsuleYAxis = ShapeQuat.GetAxisY();
		const FVector CapsuleZAxis = ShapeQuat.GetAxisZ();
		DrawCylinder(PDI, ShapeLocation, CapsuleXAxis, CapsuleYAxis, CapsuleZAxis, CurShape.Radius, CurShape.HalfHeight, 16, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
		DrawSphere(PDI, ShapeLocation - CapsuleZAxis * CurShape.HalfHeight, ShapeRotator, FVector(CurShape.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
		DrawSphere(PDI, ShapeLocation + CapsuleZAxis * CurShape.HalfHeight, ShapeRotator, FVector(CurShape.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
		DrawWireCapsule(PDI, ShapeLocation, CapsuleXAxis, CapsuleYAxis, CapsuleZAxis, FLinearColor::Red, CurShape.Radius, CurShape.HalfHeight + CurShape.Radius, 16, SDPG_World);
		DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, CurShape.Radius, SDPG_Foreground);

		PDI->SetHitProxy(nullptr);
	}

	if (CachedAnimVerletGraphNode->Node.CollisionDataAsset != nullptr)
	{
		for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.CapsuleCollisionData.Num(); i++)
		{
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Capsule, true, i));
			const FLKAnimVerletCollisionDataCapsule& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.CapsuleCollisionData[i];

			const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
			const FVector ShapeLocation = CollisionWorldT.GetLocation();
			const FQuat ShapeQuat = CollisionWorldT.GetRotation();
			const FRotator ShapeRotator = ShapeQuat.Rotator();

			const FVector CapsuleXAxis = ShapeQuat.GetAxisX();
			const FVector CapsuleYAxis = ShapeQuat.GetAxisY();
			const FVector CapsuleZAxis = ShapeQuat.GetAxisZ();
			DrawCylinder(PDI, ShapeLocation, CapsuleXAxis, CapsuleYAxis, CapsuleZAxis, CurShapeData.Radius, CurShapeData.HalfHeight, 16, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawSphere(PDI, ShapeLocation - CapsuleZAxis * CurShapeData.HalfHeight, ShapeRotator, FVector(CurShapeData.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawSphere(PDI, ShapeLocation + CapsuleZAxis * CurShapeData.HalfHeight, ShapeRotator, FVector(CurShapeData.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawWireCapsule(PDI, ShapeLocation, CapsuleXAxis, CapsuleYAxis, CapsuleZAxis, FLinearColor::Red, CurShapeData.Radius, CurShapeData.HalfHeight + CurShapeData.Radius, 16, SDPG_World);
			DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, CurShapeData.Radius, SDPG_Foreground);

			PDI->SetHitProxy(nullptr);
		}
	}
}

void FLKAnimVerletEditMode::RenderBoxColliders(FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent)
{
	if (CachedAnimVerletGraphNode->bShowAndModifyBoxCollision == false)
	{
		if (SelectedElement.ColliderType == ELKAnimVerletCollider::Box)
			SelectedElement.ResetSelection();
		return;
	}

	for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.BoxCollisionShapes.Num(); i++)
	{
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Box, false, i));
		const FLKAnimVerletCollisionBox& CurShape = CachedAnimVerletGraphNode->Node.BoxCollisionShapes[i];

		const FTransform CollisionWorldT = GetColliderWorldT(CurShape, PreviewMeshComponent);
		const FVector ShapeLocation = CollisionWorldT.GetLocation();
		const FQuat ShapeQuat = CollisionWorldT.GetRotation();
		const FRotator ShapeRotator = ShapeQuat.Rotator();

		const FMatrix BoxMat = CollisionWorldT.ToMatrixNoScale();
		const FBox Box(-CurShape.HalfExtents, CurShape.HalfExtents);
		DrawBox(PDI, CollisionWorldT.ToMatrixNoScale(), CurShape.HalfExtents, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
		DrawWireBox(PDI, BoxMat, Box, FColor::Red, SDPG_World);
		DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, FMath::Max3(CurShape.HalfExtents.X, CurShape.HalfExtents.Y, CurShape.HalfExtents.Z), SDPG_Foreground);

		PDI->SetHitProxy(nullptr);
	}

	if (CachedAnimVerletGraphNode->Node.CollisionDataAsset != nullptr)
	{
		for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.BoxCollisionData.Num(); i++)
		{
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Box, true, i));
			const FLKAnimVerletCollisionDataBox& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.BoxCollisionData[i];

			const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
			const FVector ShapeLocation = CollisionWorldT.GetLocation();
			const FQuat ShapeQuat = CollisionWorldT.GetRotation();
			const FRotator ShapeRotator = ShapeQuat.Rotator();

			const FMatrix BoxMat = CollisionWorldT.ToMatrixNoScale();
			const FBox Box(-CurShapeData.HalfExtents, CurShapeData.HalfExtents);
			DrawBox(PDI, CollisionWorldT.ToMatrixNoScale(), CurShapeData.HalfExtents, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawWireBox(PDI, BoxMat, Box, FColor::Red, SDPG_World);
			DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, FMath::Max3(CurShapeData.HalfExtents.X, CurShapeData.HalfExtents.Y, CurShapeData.HalfExtents.Z), SDPG_Foreground);

			PDI->SetHitProxy(nullptr);
		}
	}
}

void FLKAnimVerletEditMode::RenderPlaneColliders(FPrimitiveDrawInterface* PDI, const USkeletalMeshComponent* PreviewMeshComponent)
{
	if (CachedAnimVerletGraphNode->bShowAndModifyPlaneCollision == false)
	{
		if (SelectedElement.ColliderType == ELKAnimVerletCollider::Plane)
			SelectedElement.ResetSelection();
		return;
	}

	for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.PlaneCollisionShapes.Num(); i++)
	{
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Plane, false, i));
		const FLKAnimVerletCollisionPlane& CurShape = CachedAnimVerletGraphNode->Node.PlaneCollisionShapes[i];

		const FTransform CollisionWorldT = GetColliderWorldT(CurShape, PreviewMeshComponent);
		const FVector ShapeLocation = CollisionWorldT.GetLocation();
		const FQuat ShapeQuat = CollisionWorldT.GetRotation();
		const FRotator ShapeRotator = ShapeQuat.Rotator();
		const FMatrix PlaneMat = CollisionWorldT.ToMatrixNoScale();

		const FVector2D PlaneHalfExtents2D = CurShape.bFinitePlane ? CurShape.FinitePlaneHalfExtents : FVector2D(100.0f, 100.0f);
		const FVector PlaneHalfExtents = FVector(PlaneHalfExtents2D, 1.0f);
		const FBox Box(-PlaneHalfExtents, PlaneHalfExtents);
		DrawBox(PDI, PlaneMat, PlaneHalfExtents, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
		DrawWireBox(PDI, PlaneMat, Box, FColor::Red, SDPG_World);
		DrawDirectionalArrow(PDI, FRotationMatrix(FRotator(90.0f, 0.0f, 0.0f)) * PlaneMat, FLinearColor::Gray, 50.0f, 20.0f, SDPG_World, 0.5f);
		DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, FMath::Max3(PlaneHalfExtents.X, PlaneHalfExtents.Y, PlaneHalfExtents.Z), SDPG_Foreground);

		PDI->SetHitProxy(nullptr);
	}

	if (CachedAnimVerletGraphNode->Node.CollisionDataAsset != nullptr)
	{
		for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.PlaneCollisionData.Num(); i++)
		{
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Plane, true, i));
			const FLKAnimVerletCollisionDataPlane& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.PlaneCollisionData[i];

			const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
			const FVector ShapeLocation = CollisionWorldT.GetLocation();
			const FQuat ShapeQuat = CollisionWorldT.GetRotation();
			const FRotator ShapeRotator = ShapeQuat.Rotator();
			const FMatrix PlaneMat = CollisionWorldT.ToMatrixNoScale();

			const FVector2D PlaneHalfExtents2D = CurShapeData.bFinitePlane ? CurShapeData.FinitePlaneHalfExtents : FVector2D(100.0f, 100.0f);
			const FVector PlaneHalfExtents = FVector(PlaneHalfExtents2D, 1.0f);
			const FBox Box(-PlaneHalfExtents, PlaneHalfExtents);
			DrawBox(PDI, PlaneMat, PlaneHalfExtents, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawWireBox(PDI, PlaneMat, Box, FColor::Red, SDPG_World);
			DrawDirectionalArrow(PDI, FRotationMatrix(FRotator(90.0f, 0.0f, 0.0f)) * PlaneMat, FLinearColor::Gray, 50.0f, 20.0f, SDPG_World, 0.5f);
			DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, FMath::Max3(PlaneHalfExtents.X, PlaneHalfExtents.Y, PlaneHalfExtents.Z), SDPG_Foreground);

			PDI->SetHitProxy(nullptr);
		}
	}
}

void FLKAnimVerletEditMode::DrawHUD(FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas)
{
	if (AnimNode != nullptr)
	{
		AnimNode->DrawCanvas(*Viewport, *const_cast<FSceneView*>(View), *Canvas, GetAnimPreviewScene().GetPreviewMeshComponent());
	}
}

bool FLKAnimVerletEditMode::HandleClick(FEditorViewportClient* InViewportClient, HHitProxy* HitProxy, const FViewportClick& Click)
{
	bool bResult = FAnimNodeEditMode::HandleClick(InViewportClient, HitProxy, Click);

	if (HitProxy != nullptr && HitProxy->IsA(FLKAnimVerletEditModeHitProxy::StaticGetType()))
	{
		FLKAnimVerletEditModeHitProxy* AnimVerletEditModeHitProxy = static_cast<FLKAnimVerletEditModeHitProxy*>(HitProxy);
		SelectedElement = AnimVerletEditModeHitProxy->Selection;
		bResult = true;
	}
	else
	{
		SelectedElement.ResetSelection();
	}

	return bResult;
}

bool FLKAnimVerletEditMode::GetCustomDrawingCoordinateSystem(FMatrix& InMatrix, void* InData)
{
	if (ValidateSelection(false) == false)
		///return FAnimNodeEditMode::GetCustomDrawingCoordinateSystem(InMatrix, InData);
		return false;

	const FLKAnimVerletColliderInterface* CurCollider = GetSelectedColliderFromRuntime();
	if (CurCollider == nullptr)
		return false;

	///const FTransform CurColliderT = GetColliderWorldT(*CurCollider);
	const FTransform CurColliderT = CurCollider->GetTransform();
	InMatrix = CurColliderT.ToMatrixNoScale().RemoveTranslation();
	return true;
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

void FLKAnimVerletEditMode::EnterMode(UAnimGraphNode_Base* InEditorNode, FAnimNode_Base* InRuntimeNode)
{
	FAnimNodeEditMode::EnterMode(InEditorNode, InRuntimeNode);

	AnimNode = InEditorNode;
	RuntimeAnimNode = InRuntimeNode;
	CachedAnimVerletGraphNode = Cast<ULKAnimGraphNode_AnimVerlet>(AnimNode);
	if (CachedAnimVerletGraphNode == nullptr)
		return;

	CachedAnimVerletNode = static_cast<FLKAnimNode_AnimVerlet*>(RuntimeAnimNode);

	CachedAnimVerletNode->SphereCollisionShapes = CachedAnimVerletGraphNode->Node.SphereCollisionShapes;
	CachedAnimVerletNode->CapsuleCollisionShapes = CachedAnimVerletGraphNode->Node.CapsuleCollisionShapes;
	CachedAnimVerletNode->BoxCollisionShapes = CachedAnimVerletGraphNode->Node.BoxCollisionShapes;
	CachedAnimVerletNode->PlaneCollisionShapes = CachedAnimVerletGraphNode->Node.PlaneCollisionShapes;
}

void FLKAnimVerletEditMode::ExitMode()
{
	AnimNode = nullptr;
	RuntimeAnimNode = nullptr;
	CachedAnimVerletGraphNode = nullptr;
	CachedAnimVerletNode = nullptr;

	FAnimNodeEditMode::ExitMode();
}

LK_UEWIDGET::EWidgetMode FLKAnimVerletEditMode::GetWidgetMode() const 
{
	return CurrentWidgetMode;
}

LK_UEWIDGET::EWidgetMode FLKAnimVerletEditMode::ChangeToNextWidgetMode(LK_UEWIDGET::EWidgetMode CurWidgetMode) 
{
	switch (CurWidgetMode)
	{
		case UE::Widget::WM_Translate:		return LK_UEWIDGET::EWidgetMode::WM_Rotate;
		case UE::Widget::WM_Rotate:			return LK_UEWIDGET::EWidgetMode::WM_Scale;
		case UE::Widget::WM_Scale:			return LK_UEWIDGET::EWidgetMode::WM_Translate;		
		default:							break;
	}
	return LK_UEWIDGET::EWidgetMode::WM_Translate;
}

bool FLKAnimVerletEditMode::SetWidgetMode(LK_UEWIDGET::EWidgetMode InWidgetMode) 
{ 
	switch (InWidgetMode)
	{
		case LK_UEWIDGET::EWidgetMode::WM_Translate:
		case LK_UEWIDGET::EWidgetMode::WM_Rotate:
		case LK_UEWIDGET::EWidgetMode::WM_Scale:
			CurrentWidgetMode = InWidgetMode;
			return true;

		default:
			break;
	}

	CurrentWidgetMode = LK_UEWIDGET::EWidgetMode::WM_Translate;
	return false; 
}

FVector FLKAnimVerletEditMode::GetWidgetLocation() const
{
	if (ValidateSelection(false) == false)
		return FVector::ZeroVector;

	const FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime();
	if (ColliderFromRuntime == nullptr)
		return FVector::ZeroVector;

	return GetColliderWorldT(*ColliderFromRuntime).GetLocation();
}

void FLKAnimVerletEditMode::DoTranslation(FVector& InTranslation)
{
	if (CachedAnimVerletNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromGraphNode = GetSelectedColliderFromGraphNode();
	if (ColliderFromGraphNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime();
	if (ColliderFromRuntime == nullptr)
		return;

	FVector NewDelta = FVector::ZeroVector;
	if (ColliderFromRuntime->IsUseAbsoluteWorldTransform() == false && ColliderFromRuntime->GetAttachBoneName() != NAME_None)
	{
		const USkeletalMeshComponent* PreviewMeshComponent = GetAnimPreviewScene().GetPreviewMeshComponent();
		if (PreviewMeshComponent != nullptr)
		{
			NewDelta = ConvertCSVectorToBoneSpace(PreviewMeshComponent, InTranslation, CachedAnimVerletNode->ForwardedPose, ColliderFromRuntime->GetAttachBoneName(), BCS_BoneSpace);
		}
	}
	else
	{
		NewDelta = InTranslation;
	}
	ColliderFromRuntime->SetLocation(ColliderFromRuntime->GetLocation() + NewDelta);
	ColliderFromGraphNode->SetLocation(ColliderFromRuntime->GetLocation());

	CachedAnimVerletNode->MarkLocalColliderDirty();
}

void FLKAnimVerletEditMode::DoRotation(FRotator& InRotation)
{
	if (CachedAnimVerletNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromGraphNode = GetSelectedColliderFromGraphNode();
	if (ColliderFromGraphNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime();
	if (ColliderFromRuntime == nullptr)
		return;

	FQuat NewDelta = FQuat::Identity;
	if (ColliderFromRuntime->IsUseAbsoluteWorldTransform() == false && ColliderFromRuntime->GetAttachBoneName() != NAME_None)
	{
		const USkeletalMeshComponent* PreviewMeshComponent = GetAnimPreviewScene().GetPreviewMeshComponent();
		if (PreviewMeshComponent != nullptr)
		{
			NewDelta = ConvertCSRotationToBoneSpaceNoScale(PreviewMeshComponent, InRotation, CachedAnimVerletNode->ForwardedPose, ColliderFromRuntime->GetAttachBoneName(), BCS_BoneSpace);
		}
	}
	else
	{
		NewDelta = InRotation.Quaternion();
	}
	const FQuat NewRotationOffset = NewDelta * ColliderFromRuntime->GetRotation().Quaternion();
	ColliderFromRuntime->SetRotation(NewRotationOffset.Rotator());
	ColliderFromGraphNode->SetRotation(ColliderFromRuntime->GetRotation());

	CachedAnimVerletNode->MarkLocalColliderDirty();
}

void FLKAnimVerletEditMode::DoScale(FVector& InScale)
{
	if (CachedAnimVerletNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromGraphNode = GetSelectedColliderFromGraphNode();
	if (ColliderFromGraphNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime();
	if (ColliderFromRuntime == nullptr)
		return;

	FVector NewDelta = FVector::ZeroVector;
	if (ColliderFromRuntime->IsUseAbsoluteWorldTransform() == false && ColliderFromRuntime->GetAttachBoneName() != NAME_None)
	{
		const USkeletalMeshComponent* PreviewMeshComponent = GetAnimPreviewScene().GetPreviewMeshComponent();
		if (PreviewMeshComponent != nullptr)
		{
			NewDelta = ConvertCSVectorToBoneSpaceNoScale(PreviewMeshComponent, InScale, CachedAnimVerletNode->ForwardedPose, ColliderFromRuntime->GetAttachBoneName(), BCS_BoneSpace);
		}
	}
	else
	{
		NewDelta = InScale;
	}

	const FVector OriginalHalfExtents = ColliderFromRuntime->GetHalfExtents();
	FVector NewHalfExtents = OriginalHalfExtents + NewDelta;
	NewHalfExtents.X = FMath::Max(NewHalfExtents.X, 0.0f);
	NewHalfExtents.Y = FMath::Max(NewHalfExtents.Y, 0.0f);
	NewHalfExtents.Z = FMath::Max(NewHalfExtents.Z, 0.0f);
	ColliderFromRuntime->SetHalfExtents(NewHalfExtents);
	ColliderFromGraphNode->SetHalfExtents(NewHalfExtents);

	CachedAnimVerletNode->MarkLocalColliderDirty();
}

bool FLKAnimVerletEditMode::ValidateSelection(bool bGraphNode) const
{
	if (SelectedElement.IsSelected() == false)
		return false;
	else if (bGraphNode)
	{
		if (CachedAnimVerletGraphNode == nullptr)
		{
			SelectedElement.ResetSelection();
			return false;
		}
		else if (SelectedElement.bColliderFromDA && CachedAnimVerletGraphNode->Node.CollisionDataAsset == nullptr)
		{
			SelectedElement.ResetSelection();
			return false;
		}
	}
	else
	{
		if (CachedAnimVerletNode == nullptr)
		{
			SelectedElement.ResetSelection();
			return false;
		}
		else if (SelectedElement.bColliderFromDA && CachedAnimVerletNode->CollisionDataAsset == nullptr)
		{
			SelectedElement.ResetSelection();
			return false;
		}
	}

	FLKAnimNode_AnimVerlet* CurAnimVerletNode = bGraphNode ? &CachedAnimVerletGraphNode->Node : CachedAnimVerletNode;
	switch (SelectedElement.ColliderType)
	{
		case ELKAnimVerletCollider::Sphere:
		{
			if (SelectedElement.bColliderFromDA)
			{
				if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.SphereCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else
			{
				if (CurAnimVerletNode->SphereCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			break;
		}

		case ELKAnimVerletCollider::Capsule:
		{
			if (SelectedElement.bColliderFromDA)
			{
				if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.CapsuleCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else
			{
				if (CurAnimVerletNode->CapsuleCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			break;
		}

		case ELKAnimVerletCollider::Box:
		{
			if (SelectedElement.bColliderFromDA)
			{
				if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.BoxCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else
			{
				if (CurAnimVerletNode->BoxCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			break;
		}

		case ELKAnimVerletCollider::Plane:
		{
			if (SelectedElement.bColliderFromDA)
			{
				if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.PlaneCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else
			{
				if (CurAnimVerletNode->PlaneCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			break;
		}

		default:
			verify(false);
			break;
	}

	return SelectedElement.IsSelected();
}

FLKAnimVerletColliderInterface* FLKAnimVerletEditMode::GetSelectedColliderFromGraphNode() const
{
	if (ValidateSelection(true) == false)
		return nullptr;

	if (SelectedElement.bColliderFromDA)
	{
		if (CachedAnimVerletGraphNode->Node.CollisionDataAsset == nullptr)
			return nullptr;

		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:			return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Capsule:		return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.CapsuleCollisionData[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Box:			return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.BoxCollisionData[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Plane:			return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.PlaneCollisionData[SelectedElement.ColliderListIndex];
			default:									verify(false); break;
		}
	}
	else
	{
		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:			return &CachedAnimVerletGraphNode->Node.SphereCollisionShapes[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Capsule:		return &CachedAnimVerletGraphNode->Node.CapsuleCollisionShapes[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Box:			return &CachedAnimVerletGraphNode->Node.BoxCollisionShapes[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Plane:			return &CachedAnimVerletGraphNode->Node.PlaneCollisionShapes[SelectedElement.ColliderListIndex];
			default:									verify(false); break;
		}
	}
	return nullptr;
}

FLKAnimVerletColliderInterface* FLKAnimVerletEditMode::GetSelectedColliderFromRuntime() const
{
	if (ValidateSelection(false) == false)
		return nullptr;

	if (SelectedElement.bColliderFromDA)
	{
		if (CachedAnimVerletNode->CollisionDataAsset == nullptr)
			return nullptr;

		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:			return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.SphereCollisionData[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Capsule:		return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.CapsuleCollisionData[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Box:			return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.BoxCollisionData[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Plane:			return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.PlaneCollisionData[SelectedElement.ColliderListIndex];
			default:									verify(false); break;
		}
	}
	else
	{
		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:			return &CachedAnimVerletNode->SphereCollisionShapes[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Capsule:		return &CachedAnimVerletNode->CapsuleCollisionShapes[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Box:			return &CachedAnimVerletNode->BoxCollisionShapes[SelectedElement.ColliderListIndex];
			case ELKAnimVerletCollider::Plane:			return &CachedAnimVerletNode->PlaneCollisionShapes[SelectedElement.ColliderListIndex];
			default:									verify(false); break;
		}
	}
	return nullptr;
}

bool FLKAnimVerletEditMode::ConvertBoneTToWorldT(OUT FTransform& OutWorldT, const USkeletalMeshComponent* SkeletalMeshComponent, const FName& BoneName) const
{
	verify(SkeletalMeshComponent != nullptr);
	if (BoneName == NAME_None)
		return false;

	const USkeletalMesh* SkelMesh = SkeletalMeshComponent->GetSkeletalMeshAsset();
	if (SkelMesh == nullptr)
		return false;

	const USkeleton* Skeleton = SkelMesh->GetSkeleton();
	if (Skeleton == nullptr)
		return false;

	OutWorldT = SkeletalMeshComponent->GetBoneTransform(BoneName);
	return true;
}

FTransform FLKAnimVerletEditMode::GetColliderWorldT(const FLKAnimVerletColliderInterface& InCollider, const USkeletalMeshComponent* SkeletalMeshComponent) const
{
	if (InCollider.IsUseAbsoluteWorldTransform())
		return InCollider.GetTransform();

	if (SkeletalMeshComponent == nullptr)
		return FTransform::Identity;

	FTransform BoneWorldT;
	ConvertBoneTToWorldT(OUT BoneWorldT, SkeletalMeshComponent, InCollider.GetAttachBoneName());

	const FTransform LocalOffsetT = InCollider.GetTransform();
	BoneWorldT = LocalOffsetT * BoneWorldT;
	return BoneWorldT;
}

FTransform FLKAnimVerletEditMode::GetColliderWorldT(const FLKAnimVerletColliderInterface& InCollider) const
{
	return GetColliderWorldT(InCollider, GetAnimPreviewScene().GetPreviewMeshComponent());
}

FVector FLKAnimVerletEditMode::ConvertCSVectorToBoneSpaceNoScale(const USkeletalMeshComponent* SkelComp, FVector& InCSVector, FCSPose<FCompactHeapPose>& MeshBases, const FName& BoneName, const EBoneControlSpace Space)
{
	/// From FAnimNodeEditMode::ConvertCSVectorToBoneSpace()

	FVector OutVector = FVector::ZeroVector;

	if (MeshBases.GetPose().IsValid())
	{
		const FMeshPoseBoneIndex MeshBoneIndex(SkelComp->GetBoneIndex(BoneName));
		const FCompactPoseBoneIndex BoneIndex = MeshBases.GetPose().GetBoneContainer().MakeCompactPoseIndex(MeshBoneIndex);

		switch (Space)
		{
			// World Space, no change in preview window
			case BCS_WorldSpace:
			case BCS_ComponentSpace:
				// Component Space, no change.
				OutVector = InCSVector;
				break;

			case BCS_ParentBoneSpace:
			{
				const FCompactPoseBoneIndex ParentIndex = MeshBases.GetPose().GetParentBoneIndex(BoneIndex);
				if (ParentIndex != INDEX_NONE)
				{
					FTransform ParentTM = MeshBases.GetComponentSpaceTransform(ParentIndex);
					ParentTM.SetScale3D(FVector::OneVector);
					OutVector = ParentTM.InverseTransformVector(InCSVector);
				}
			}
			break;

			case BCS_BoneSpace:
			{
				if (BoneIndex != INDEX_NONE)
				{
					FTransform BoneTM = MeshBases.GetComponentSpaceTransform(BoneIndex);
					BoneTM.SetScale3D(FVector::OneVector);
					OutVector = BoneTM.InverseTransformVector(InCSVector);
				}
			}
			break;
		}
	}

	return OutVector;
}

FQuat FLKAnimVerletEditMode::ConvertCSRotationToBoneSpaceNoScale(const USkeletalMeshComponent* SkelComp, FRotator& InCSRotator, FCSPose<FCompactHeapPose>& MeshBases, const FName& BoneName, const EBoneControlSpace Space)
{
	/// From FAnimNodeEditMode::ConvertCSRotationToBoneSpace()

	FQuat OutQuat = FQuat::Identity;

	if (MeshBases.GetPose().IsValid())
	{
		const FMeshPoseBoneIndex MeshBoneIndex(SkelComp->GetBoneIndex(BoneName));
		const FCompactPoseBoneIndex BoneIndex = MeshBases.GetPose().GetBoneContainer().MakeCompactPoseIndex(MeshBoneIndex);

		FVector RotAxis;
		float RotAngle;
		InCSRotator.Quaternion().ToAxisAndAngle(RotAxis, RotAngle);

		switch (Space)
		{
			// World Space, no change in preview window
			case BCS_WorldSpace:
			case BCS_ComponentSpace:
				// Component Space, no change.
				OutQuat = InCSRotator.Quaternion();
				break;

			case BCS_ParentBoneSpace:
			{
				const FCompactPoseBoneIndex ParentIndex = MeshBases.GetPose().GetParentBoneIndex(BoneIndex);
				if (ParentIndex != INDEX_NONE)
				{
					FTransform ParentTM = MeshBases.GetComponentSpaceTransform(ParentIndex);
					ParentTM.SetScale3D(FVector::OneVector);

					FTransform InverseParentTM = ParentTM.Inverse();
					//Calculate the new delta rotation
					FVector4 BoneSpaceAxis = InverseParentTM.TransformVector(RotAxis);
					FQuat DeltaQuat(BoneSpaceAxis, RotAngle);
					DeltaQuat.Normalize();
					OutQuat = DeltaQuat;
				}
			}
			break;

			case BCS_BoneSpace:
			{
				FTransform BoneTM = MeshBases.GetComponentSpaceTransform(BoneIndex);
				BoneTM.SetScale3D(FVector::OneVector);


				FTransform InverseBoneTM = BoneTM.Inverse();
				FVector4 BoneSpaceAxis = InverseBoneTM.TransformVector(RotAxis);
				//Calculate the new delta rotation
				FQuat DeltaQuat(BoneSpaceAxis, RotAngle);
				DeltaQuat.Normalize();
				OutQuat = DeltaQuat;
			}
			break;
		}
	}

	return OutQuat;
}