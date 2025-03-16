#include "LKAnimVerletEditMode.h"

#include <AnimationRuntime.h>
#include <AssetEditorModeManager.h>
#include <CanvasItem.h>
#include <CanvasTypes.h>
#include <IPersonaPreviewScene.h>
#include <Animation/DebugSkelMeshComponent.h>
#include <PhysicsEngine/PhysicsAsset.h>
#if (ENGINE_MINOR_VERSION >= 5)
#include <PhysicsEngine/SkeletalBodySetup.h>
#endif
#include "LKAnimVerletCollisionData.h"

struct FLKAnimVerletEditModeHitProxy : HHitProxy
{
	DECLARE_HIT_PROXY()

public:
	FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider InColliderType, ELKAnimVerletColliderSource InColliderSourceType, int32 InColliderListIndex, int32 InOptionalBodySetupIndex)
		: HHitProxy(HPP_Foreground)
	{
		Selection.Select(InColliderType, InColliderSourceType, InColliderListIndex, InOptionalBodySetupIndex);
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
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Sphere, ELKAnimVerletColliderSource::Node, i, INDEX_NONE));
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
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Sphere, ELKAnimVerletColliderSource::DataAsset, i, INDEX_NONE));
			const FLKAnimVerletCollisionDataSphere& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData[i];

			const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
			const FVector ShapeLocation = CollisionWorldT.GetLocation();
			DrawSphere(PDI, ShapeLocation, FRotator::ZeroRotator, FVector(CurShapeData.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
			DrawWireSphere(PDI, ShapeLocation, FLinearColor::Red, CurShapeData.Radius, 16, SDPG_World);
			DrawCoordinateSystem(PDI, ShapeLocation, CollisionWorldT.Rotator(), CurShapeData.Radius, SDPG_Foreground);

			PDI->SetHitProxy(nullptr);
		}
	}

	if (CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset != nullptr)
	{
		for (int32 BodySetupI = 0; BodySetupI < CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups.Num(); ++BodySetupI)
		{
			const TObjectPtr<USkeletalBodySetup>& BodySetup = CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups[BodySetupI];
			const FKAggregateGeom& CurGeometry = BodySetup->AggGeom;
			for (int32 GeomI = 0; GeomI < CurGeometry.SphereElems.Num(); ++GeomI)
			{
				PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Sphere, ELKAnimVerletColliderSource::PhysicsAsset, GeomI, BodySetupI));
				const FKSphereElem& SphereElem = CurGeometry.SphereElems[GeomI];		
				FLKAnimVerletCollisionSphere SphereShape;
				{
					SphereShape.AttachedBone = BodySetup->BoneName;
					SphereShape.LocationOffset = SphereElem.Center;
					SphereShape.Radius = SphereElem.Radius;
				}

				const FTransform CollisionWorldT = GetColliderWorldT(SphereShape, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				DrawSphere(PDI, ShapeLocation, FRotator::ZeroRotator, FVector(SphereShape.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
				DrawWireSphere(PDI, ShapeLocation, FLinearColor::Red, SphereShape.Radius, 16, SDPG_World);
				DrawCoordinateSystem(PDI, ShapeLocation, CollisionWorldT.Rotator(), SphereShape.Radius, SDPG_Foreground);

				PDI->SetHitProxy(nullptr);
			}
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
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Capsule, ELKAnimVerletColliderSource::Node, i, INDEX_NONE));
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
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Capsule, ELKAnimVerletColliderSource::DataAsset, i, INDEX_NONE));
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

	if (CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset != nullptr)
	{
		for (int32 BodySetupI = 0; BodySetupI < CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups.Num(); ++BodySetupI)
		{
			const TObjectPtr<USkeletalBodySetup>& BodySetup = CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups[BodySetupI];
			const FKAggregateGeom& CurGeometry = BodySetup->AggGeom;
			for (int32 GeomI = 0; GeomI < CurGeometry.SphylElems.Num(); ++GeomI)
			{
				PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Capsule, ELKAnimVerletColliderSource::PhysicsAsset, GeomI, BodySetupI));
				const FKSphylElem& CapsuleElem = CurGeometry.SphylElems[GeomI];
				FLKAnimVerletCollisionCapsule CapsuleShape;
				{
					CapsuleShape.AttachedBone = BodySetup->BoneName;
					CapsuleShape.LocationOffset = CapsuleElem.Center;
					CapsuleShape.RotationOffset = CapsuleElem.Rotation;
					CapsuleShape.Radius = CapsuleElem.Radius;
					CapsuleShape.HalfHeight = CapsuleElem.Length * 0.5f;
				}

				const FTransform CollisionWorldT = GetColliderWorldT(CapsuleShape, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				const FQuat ShapeQuat = CollisionWorldT.GetRotation();
				const FRotator ShapeRotator = ShapeQuat.Rotator();

				const FVector CapsuleXAxis = ShapeQuat.GetAxisX();
				const FVector CapsuleYAxis = ShapeQuat.GetAxisY();
				const FVector CapsuleZAxis = ShapeQuat.GetAxisZ();
				DrawCylinder(PDI, ShapeLocation, CapsuleXAxis, CapsuleYAxis, CapsuleZAxis, CapsuleShape.Radius, CapsuleShape.HalfHeight, 16, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
				DrawSphere(PDI, ShapeLocation - CapsuleZAxis * CapsuleShape.HalfHeight, ShapeRotator, FVector(CapsuleShape.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
				DrawSphere(PDI, ShapeLocation + CapsuleZAxis * CapsuleShape.HalfHeight, ShapeRotator, FVector(CapsuleShape.Radius), 16, 6, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
				DrawWireCapsule(PDI, ShapeLocation, CapsuleXAxis, CapsuleYAxis, CapsuleZAxis, FLinearColor::Red, CapsuleShape.Radius, CapsuleShape.HalfHeight + CapsuleShape.Radius, 16, SDPG_World);
				DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, CapsuleShape.Radius, SDPG_Foreground);

				PDI->SetHitProxy(nullptr);
			}
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
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Box, ELKAnimVerletColliderSource::Node, i, INDEX_NONE));
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
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Box, ELKAnimVerletColliderSource::DataAsset, i, INDEX_NONE));
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

	if (CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset != nullptr)
	{
		for (int32 BodySetupI = 0; BodySetupI < CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups.Num(); ++BodySetupI)
		{
			const TObjectPtr<USkeletalBodySetup>& BodySetup = CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups[BodySetupI];
			const FKAggregateGeom& CurGeometry = BodySetup->AggGeom;
			for (int32 GeomI = 0; GeomI < CurGeometry.BoxElems.Num(); ++GeomI)
			{
				PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Capsule, ELKAnimVerletColliderSource::PhysicsAsset, GeomI, BodySetupI));
				const FKBoxElem& BoxElem = CurGeometry.BoxElems[GeomI];
				FLKAnimVerletCollisionBox BoxShape;
				{
					BoxShape.AttachedBone = BodySetup->BoneName;
					BoxShape.LocationOffset = BoxElem.Center;
					BoxShape.RotationOffset = BoxElem.Rotation;
					BoxShape.HalfExtents = FVector(BoxElem.X, BoxElem.Y, BoxElem.Z) * 0.5f;
				}

				const FTransform CollisionWorldT = GetColliderWorldT(BoxShape, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				const FQuat ShapeQuat = CollisionWorldT.GetRotation();
				const FRotator ShapeRotator = ShapeQuat.Rotator();

				const FMatrix BoxMat = CollisionWorldT.ToMatrixNoScale();
				const FBox Box(-BoxShape.HalfExtents, BoxShape.HalfExtents);
				DrawBox(PDI, CollisionWorldT.ToMatrixNoScale(), BoxShape.HalfExtents, GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
				DrawWireBox(PDI, BoxMat, Box, FColor::Red, SDPG_World);
				DrawCoordinateSystem(PDI, ShapeLocation, ShapeRotator, FMath::Max3(BoxShape.HalfExtents.X, BoxShape.HalfExtents.Y, BoxShape.HalfExtents.Z), SDPG_Foreground);

				PDI->SetHitProxy(nullptr);
			}
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
		PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Plane, ELKAnimVerletColliderSource::Node, i, INDEX_NONE));
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
			PDI->SetHitProxy(new FLKAnimVerletEditModeHitProxy(ELKAnimVerletCollider::Plane, ELKAnimVerletColliderSource::DataAsset, i, INDEX_NONE));
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

void FLKAnimVerletEditMode::RenderText(const FViewport* Viewport, const FSceneView* View, FCanvas* Canvas, const FVector& Location, const FText& Text)
{
	if (Canvas == nullptr || View == nullptr || Viewport == nullptr)
		return;

	const int32 HalfX = (Viewport->GetSizeXY().X * 0.5f) / Canvas->GetDPIScale();
	const int32 HalfY = (Viewport->GetSizeXY().Y * 0.5f) / Canvas->GetDPIScale();
	const FPlane Projected = View->Project(Location);
	if (Projected.W > 0.f)
	{
		const int32 XPos = HalfX + (HalfX * Projected.X);
		const int32 YPos = HalfY + (HalfY * -Projected.Y);

		FCanvasTextItem TextItem(FVector2D(XPos, YPos), Text, GEngine->GetSmallFont(), FLinearColor::White);
		TextItem.EnableShadow(FLinearColor::Black);
		Canvas->DrawItem(TextItem);
	}
}

void FLKAnimVerletEditMode::DrawHUD(FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas)
{
	FAnimNodeEditMode::DrawHUD(ViewportClient, Viewport, View, Canvas);
	///if (AnimNode != nullptr)
	///{
	///	AnimNode->DrawCanvas(*Viewport, *const_cast<FSceneView*>(View), *Canvas, GetAnimPreviewScene().GetPreviewMeshComponent());
	///}
	
	const USkeletalMeshComponent* PreviewMeshComponent = GetAnimPreviewScene().GetPreviewMeshComponent();
	if (PreviewMeshComponent != nullptr && PreviewMeshComponent->GetSkeletalMeshAsset() != nullptr)
	{
		if (CachedAnimVerletGraphNode->Node.CollisionDataAsset != nullptr)
		{
			for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData.Num(); i++)
			{
				const FLKAnimVerletCollisionDataSphere& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData[i];
				const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("DataAsset")));
			}
			for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.CapsuleCollisionData.Num(); i++)
			{
				const FLKAnimVerletCollisionDataCapsule& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.CapsuleCollisionData[i];
				const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("DataAsset")));
			}
			for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.BoxCollisionData.Num(); i++)
			{
				const FLKAnimVerletCollisionDataBox& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.BoxCollisionData[i];
				const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("DataAsset")));
			}
			for (int32 i = 0; i < CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.PlaneCollisionData.Num(); i++)
			{
				const FLKAnimVerletCollisionDataPlane& CurShapeData = CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.PlaneCollisionData[i];
				const FTransform CollisionWorldT = GetColliderWorldT(CurShapeData, PreviewMeshComponent);
				const FVector ShapeLocation = CollisionWorldT.GetLocation();
				RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("DataAsset")));
			}
		}

		if (CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset != nullptr)
		{
			for (int32 BodySetupI = 0; BodySetupI < CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups.Num(); ++BodySetupI)
			{
				const TObjectPtr<USkeletalBodySetup>& BodySetup = CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->SkeletalBodySetups[BodySetupI];
				const FKAggregateGeom& CurGeometry = BodySetup->AggGeom;
				for (int32 GeomI = 0; GeomI < CurGeometry.SphereElems.Num(); ++GeomI)
				{
					const FKSphereElem& SphereElem = CurGeometry.SphereElems[GeomI];
					FLKAnimVerletCollisionSphere SphereShape;
					{
						SphereShape.AttachedBone = BodySetup->BoneName;
						SphereShape.LocationOffset = SphereElem.Center;
						SphereShape.Radius = SphereElem.Radius;
					}
					const FTransform CollisionWorldT = GetColliderWorldT(SphereShape, PreviewMeshComponent);
					const FVector ShapeLocation = CollisionWorldT.GetLocation();
					RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("PhysicsAsset")));
				}
				for (int32 GeomI = 0; GeomI < CurGeometry.SphylElems.Num(); ++GeomI)
				{
					const FKSphylElem& CapsuleElem = CurGeometry.SphylElems[GeomI];
					FLKAnimVerletCollisionCapsule CapsuleShape;
					{
						CapsuleShape.AttachedBone = BodySetup->BoneName;
						CapsuleShape.LocationOffset = CapsuleElem.Center;
						CapsuleShape.RotationOffset = CapsuleElem.Rotation;
						CapsuleShape.Radius = CapsuleElem.Radius;
						CapsuleShape.HalfHeight = CapsuleElem.Length * 0.5f;
					}
					const FTransform CollisionWorldT = GetColliderWorldT(CapsuleShape, PreviewMeshComponent);
					const FVector ShapeLocation = CollisionWorldT.GetLocation();
					RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("PhysicsAsset")));
				}
				for (int32 GeomI = 0; GeomI < CurGeometry.BoxElems.Num(); ++GeomI)
				{
					const FKBoxElem& BoxElem = CurGeometry.BoxElems[GeomI];
					FLKAnimVerletCollisionBox BoxShape;
					{
						BoxShape.AttachedBone = BodySetup->BoneName;
						BoxShape.LocationOffset = BoxElem.Center;
						BoxShape.RotationOffset = BoxElem.Rotation;
						BoxShape.HalfExtents = FVector(BoxElem.X, BoxElem.Y, BoxElem.Z) * 0.5f;
					}
					const FTransform CollisionWorldT = GetColliderWorldT(BoxShape, PreviewMeshComponent);
					const FVector ShapeLocation = CollisionWorldT.GetLocation();
					RenderText(Viewport, View, Canvas, ShapeLocation, FText::FromString(TEXT("PhysicsAsset")));
				}
			}
		}
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

	const FLKAnimVerletColliderInterface* CurCollider = GetSelectedColliderFromRuntime(false);
	if (CurCollider == nullptr)
		return false;

	const FTransform CurColliderT = GetColliderWorldT(*CurCollider);
	///const FTransform CurColliderT = CurCollider->GetTransform();
	InMatrix = CurColliderT.ToMatrixNoScale().RemoveTranslation();
	return true;
}

bool FLKAnimVerletEditMode::ShouldDrawWidget() const
{
	return ValidateSelection(false);
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

	CachedAnimVerletNode->CollisionDataAsset = CachedAnimVerletGraphNode->Node.CollisionDataAsset;
	CachedAnimVerletNode->CollisionPhysicsAsset = CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset;
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

	const FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime(false);
	if (ColliderFromRuntime == nullptr)
		return FVector::ZeroVector;

	return GetColliderWorldT(*ColliderFromRuntime).GetLocation();
}

void FLKAnimVerletEditMode::DoTranslation(FVector& InTranslation)
{
	if (CachedAnimVerletNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromGraphNode = GetSelectedColliderFromGraphNode(true);
	if (ColliderFromGraphNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime(true);
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

	FLKAnimVerletColliderInterface* ColliderFromGraphNode = GetSelectedColliderFromGraphNode(true);
	if (ColliderFromGraphNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime(true);
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

	FLKAnimVerletColliderInterface* ColliderFromGraphNode = GetSelectedColliderFromGraphNode(true);
	if (ColliderFromGraphNode == nullptr)
		return;

	FLKAnimVerletColliderInterface* ColliderFromRuntime = GetSelectedColliderFromRuntime(true);
	if (ColliderFromRuntime == nullptr)
		return;

	/*FVector NewDelta = FVector::ZeroVector;
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
	}*/

	const FVector NewDelta = InScale;
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
		else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset && CachedAnimVerletGraphNode->Node.CollisionDataAsset == nullptr)
		{
			SelectedElement.ResetSelection();
			return false;
		}
		else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset && CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset == nullptr)
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
		else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset && CachedAnimVerletNode->CollisionDataAsset == nullptr)
		{
			SelectedElement.ResetSelection();
			return false;
		}
		else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset && CachedAnimVerletNode->CollisionPhysicsAsset == nullptr)
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
			if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::Node)
			{
				if (CurAnimVerletNode->SphereCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset)
			{
				if (CurAnimVerletNode->CollisionDataAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.SphereCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset)
			{
				if (CurAnimVerletNode->CollisionPhysicsAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionPhysicsAsset->SkeletalBodySetups.IsValidIndex(SelectedElement.OptionalBodySetupIndex) == false)
					SelectedElement.ResetSelection();
				else
				{
					const TObjectPtr<USkeletalBodySetup>& CurBodySetup = CurAnimVerletNode->CollisionPhysicsAsset->SkeletalBodySetups[SelectedElement.OptionalBodySetupIndex];
					if (CurBodySetup == nullptr)
						SelectedElement.ResetSelection();
					else if (CurBodySetup->AggGeom.SphereElems.IsValidIndex(SelectedElement.ColliderListIndex) == false)
						SelectedElement.ResetSelection();
				}
			}
			break;
		}

		case ELKAnimVerletCollider::Capsule:
		{
			if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::Node)
			{
				if (CurAnimVerletNode->CapsuleCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset)
			{
				if (CurAnimVerletNode->CollisionDataAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.CapsuleCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset)
			{
				if (CurAnimVerletNode->CollisionPhysicsAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionPhysicsAsset->SkeletalBodySetups.IsValidIndex(SelectedElement.OptionalBodySetupIndex) == false)
					SelectedElement.ResetSelection();
				else
				{
					const TObjectPtr<USkeletalBodySetup>& CurBodySetup = CurAnimVerletNode->CollisionPhysicsAsset->SkeletalBodySetups[SelectedElement.OptionalBodySetupIndex];
					if (CurBodySetup == nullptr)
						SelectedElement.ResetSelection();
					else if (CurBodySetup->AggGeom.SphylElems.IsValidIndex(SelectedElement.ColliderListIndex) == false)
						SelectedElement.ResetSelection();
				}
			}
			break;
		}

		case ELKAnimVerletCollider::Box:
		{
			if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::Node)
			{
				if (CurAnimVerletNode->BoxCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset)
			{
				if (CurAnimVerletNode->CollisionDataAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.BoxCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset)
			{
				if (CurAnimVerletNode->CollisionPhysicsAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionPhysicsAsset->SkeletalBodySetups.IsValidIndex(SelectedElement.OptionalBodySetupIndex) == false)
					SelectedElement.ResetSelection();
				else
				{
					const TObjectPtr<USkeletalBodySetup>& CurBodySetup = CurAnimVerletNode->CollisionPhysicsAsset->SkeletalBodySetups[SelectedElement.OptionalBodySetupIndex];
					if (CurBodySetup == nullptr)
						SelectedElement.ResetSelection();
					else if (CurBodySetup->AggGeom.BoxElems.IsValidIndex(SelectedElement.ColliderListIndex) == false)
						SelectedElement.ResetSelection();
				}
			}
			break;
		}

		case ELKAnimVerletCollider::Plane:
		{
			if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::Node)
			{
				if (CurAnimVerletNode->PlaneCollisionShapes.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset)
			{
				if (CurAnimVerletNode->CollisionDataAsset == nullptr)
					SelectedElement.ResetSelection();
				else if (CurAnimVerletNode->CollisionDataAsset->CollisionDataList.PlaneCollisionData.IsValidIndex(SelectedElement.ColliderListIndex) == false)
					SelectedElement.ResetSelection();
			}
			else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset)
			{
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

FLKAnimVerletColliderInterface* FLKAnimVerletEditMode::GetSelectedColliderFromGraphNode(bool bMarkDirty) const
{
	if (ValidateSelection(true) == false)
		return nullptr;

	if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::Node)
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
	else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset)
	{
		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.SphereCollisionData[SelectedElement.ColliderListIndex];

			case ELKAnimVerletCollider::Capsule:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.CapsuleCollisionData[SelectedElement.ColliderListIndex];

			case ELKAnimVerletCollider::Box:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.BoxCollisionData[SelectedElement.ColliderListIndex];

			case ELKAnimVerletCollider::Plane:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletGraphNode->Node.CollisionDataAsset->CollisionDataList.PlaneCollisionData[SelectedElement.ColliderListIndex];

			default:									
				verify(false); 
				break;
		}
	}
	else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset)
	{
		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->MarkPackageDirty();
				PASphereGraphNodeSpace = FLKAnimVerletCollisionPASphere(CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset, SelectedElement.OptionalBodySetupIndex, SelectedElement.ColliderListIndex);
				return &PASphereGraphNodeSpace;

			case ELKAnimVerletCollider::Capsule:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->MarkPackageDirty();
				PACapsuleGraphNodeSpace = FLKAnimVerletCollisionPACapsule(CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset, SelectedElement.OptionalBodySetupIndex, SelectedElement.ColliderListIndex);
				return &PACapsuleGraphNodeSpace;

			case ELKAnimVerletCollider::Box:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->MarkPackageDirty();
				PABoxGraphNodeSpace = FLKAnimVerletCollisionPABox(CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset, SelectedElement.OptionalBodySetupIndex, SelectedElement.ColliderListIndex);
				return &PABoxGraphNodeSpace;

			case ELKAnimVerletCollider::Plane:
			default:									
				verify(false); 
				break;
		}
	}
	return nullptr;
}

FLKAnimVerletColliderInterface* FLKAnimVerletEditMode::GetSelectedColliderFromRuntime(bool bMarkDirty) const
{
	if (ValidateSelection(false) == false)
		return nullptr;

	if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::Node)
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
	else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::DataAsset)
	{
		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:
				if (bMarkDirty)
					CachedAnimVerletNode->CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.SphereCollisionData[SelectedElement.ColliderListIndex];

			case ELKAnimVerletCollider::Capsule:
				if (bMarkDirty)
					CachedAnimVerletNode->CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.CapsuleCollisionData[SelectedElement.ColliderListIndex];

			case ELKAnimVerletCollider::Box:
				if (bMarkDirty)
					CachedAnimVerletNode->CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.BoxCollisionData[SelectedElement.ColliderListIndex];

			case ELKAnimVerletCollider::Plane:
				if (bMarkDirty)
					CachedAnimVerletNode->CollisionDataAsset->MarkPackageDirty();
				return &CachedAnimVerletNode->CollisionDataAsset->CollisionDataList.PlaneCollisionData[SelectedElement.ColliderListIndex];

			default:									
				verify(false); 
				break;
		}
	}
	else if (SelectedElement.ColliderSourceType == ELKAnimVerletColliderSource::PhysicsAsset)
	{
		switch (SelectedElement.ColliderType)
		{
			case ELKAnimVerletCollider::Sphere:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->MarkPackageDirty();
				PASphereRuntimeSpace = FLKAnimVerletCollisionPASphere(CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset, SelectedElement.OptionalBodySetupIndex, SelectedElement.ColliderListIndex);
				return &PASphereRuntimeSpace;

			case ELKAnimVerletCollider::Capsule:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->MarkPackageDirty();
				PACapsuleRuntimeSpace = FLKAnimVerletCollisionPACapsule(CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset, SelectedElement.OptionalBodySetupIndex, SelectedElement.ColliderListIndex);
				return &PACapsuleRuntimeSpace;

			case ELKAnimVerletCollider::Box:
				if (bMarkDirty)
					CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset->MarkPackageDirty();
				PABoxRuntimeSpace = FLKAnimVerletCollisionPABox(CachedAnimVerletGraphNode->Node.CollisionPhysicsAsset, SelectedElement.OptionalBodySetupIndex, SelectedElement.ColliderListIndex);
				return &PABoxRuntimeSpace;

			case ELKAnimVerletCollider::Plane:
			default:									
				verify(false); 
				break;
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