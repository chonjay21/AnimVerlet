#include "LKAnimGraphNode_AnimVerlet.h"

#include <AnimNodeEditModes.h>
#include <DetailLayoutBuilder.h>
#include <DetailWidgetRow.h>
#include <DetailCategoryBuilder.h>
#include <EngineGlobals.h>
#include <PropertyHandle.h>
#include <Animation/AnimInstance.h>
#include <Framework/Notifications/NotificationManager.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Widgets/Input/SButton.h>
#include <Widgets/Notifications/SNotificationList.h>

#define LOCTEXT_NAMESPACE "AnimVerlet"
ULKAnimGraphNode_AnimVerlet::ULKAnimGraphNode_AnimVerlet(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText ULKAnimGraphNode_AnimVerlet::GetControllerDescription() const
{
	return LOCTEXT("AnimVerlet", "AnimVerlet");
}

void ULKAnimGraphNode_AnimVerlet::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	/// Apply PresetType
	const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ULKAnimGraphNode_AnimVerlet, PresetType))
	{
		ApplyAnimVerletPresetType(Node);
	}
	else if (Node.IsPresetTypeRelatedProperty(PropertyName))
	{
		PresetType = ELKAnimVerletPreset::Custom;
	}

	FLKAnimNode_AnimVerlet* PreviewNode = GetPreviewAnimVerletNode();
	if (PreviewNode != nullptr)
	{
		/// data sync from GraphNode to Preview(Runtime) node(for realtime sync without blueprint compile)
		PreviewNode->SyncFromOtherAnimVerletNode(Node);
		PreviewNode->ForceClearSimulateBones();
		PreviewNode->MarkLocalColliderDirty();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
	ReconstructNode();
}

///void ULKAnimGraphNode_AnimVerlet::PostEditChangeChainProperty(FPropertyChangedChainEvent& PropertyChangedEvent)
///{
///
///	Super::PostEditChangeChainProperty(PropertyChangedEvent);
///}

FText ULKAnimGraphNode_AnimVerlet::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	if ((TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle))
	{
		return GetControllerDescription();
	}
	// @TODO: the bone can be altered in the property editor, so we have to 
	//        choose to mark this dirty when that happens for this to properly work
	else //if (!CachedNodeTitles.IsTitleCached(TitleType, this))
	{
		FFormatNamedArguments Args;
		Args.Add(TEXT("ControllerDescription"), GetControllerDescription());
		Args.Add(TEXT("RootBoneName"), FText::FromName(Node.VerletBones.Num() > 0 ? Node.VerletBones[0].RootBone.BoneName : TEXT("None")));

		// FText::Format() is slow, so we cache this to save on performance
		if (TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle)
		{
			CachedNodeTitles.SetCachedTitle(TitleType, FText::Format(LOCTEXT("AnimGraphNode_LKAnimVerlet_ListTitle", "{ControllerDescription} - Root: {RootBoneName}"), Args), this);
		}
		else
		{
			CachedNodeTitles.SetCachedTitle(TitleType, FText::Format(LOCTEXT("AnimGraphNode_LKAnimVerlet_Title", "{ControllerDescription}\nRoot: {RootBoneName} "), Args), this);
		}
	}
	return CachedNodeTitles[TitleType];
}

FEditorModeID ULKAnimGraphNode_AnimVerlet::GetEditorMode() const
{
	return "AnimGraph.SkeletalControl.AnimVerlet";
}

void ULKAnimGraphNode_AnimVerlet::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	Super::CustomizeDetails(DetailBuilder);

	IDetailCategoryBuilder& PreviewCategory = DetailBuilder.EditCategory(TEXT("Preview"));
	FDetailWidgetRow& ResetWidgetRow = PreviewCategory.AddCustomRow(LOCTEXT("ResetButtonRow", "Reset"));
	ResetWidgetRow
		[
			SNew(SButton)
			.Text(LOCTEXT("ResetButtonText", "Reset Simulation"))
		.ToolTipText(LOCTEXT("ResetButtonToolTip", "Resets the simulation for this node"))
		.OnClicked(FOnClicked::CreateStatic(&ULKAnimGraphNode_AnimVerlet::ResetSimulationButtonClicked, &DetailBuilder))
		];

	IDetailCategoryBuilder& AnimVerletToolCategory = DetailBuilder.EditCategory(TEXT("AnimVerlet Tool"));
	FDetailWidgetRow& ConvertToDaWidgetRow = AnimVerletToolCategory.AddCustomRow(LOCTEXT("ConvertToDataAssetRow", "ConvertCollisionToDataAsset"));
	ConvertToDaWidgetRow
		[
			SNew(SButton)
				.Text(LOCTEXT("ConvertToDaButtonText", "Convert Collision To DataAsset"))
				.ToolTipText(LOCTEXT("ConvertToDaButtonToolTip", "Convert collision shape data list to CollisionDataAsset(Need to save the DataAsset manually after convert)"))
				.OnClicked(FOnClicked::CreateStatic(&ULKAnimGraphNode_AnimVerlet::ConvertToDaButtonClicked, &DetailBuilder))
		];

	FDetailWidgetRow& ConvertFromDaWidgetRow = AnimVerletToolCategory.AddCustomRow(LOCTEXT("ConvertFromDataAssetRow", "ConvertCollisionFromDataAsset"));
	ConvertFromDaWidgetRow
		[
			SNew(SButton)
				.Text(LOCTEXT("ConvertFromDaButtonText", "Convert Collision From DataAsset"))
				.ToolTipText(LOCTEXT("ConvertFromDaButtonToolTip", "Convert CollisionDataAsset to collision shape data list(Need to save the AnimBlueprint manually after convert)"))
				.OnClicked(FOnClicked::CreateStatic(&ULKAnimGraphNode_AnimVerlet::ConvertFromDaButtonClicked, &DetailBuilder))
		];

	FDetailWidgetRow& ConvertFromPaToDaWidgetRow = AnimVerletToolCategory.AddCustomRow(LOCTEXT("ConvertFromPhysicsAssetToDataAssetRow", "ConvertCollisionFromPhysicsAssetToDataAsset"));
	ConvertFromPaToDaWidgetRow
		[
			SNew(SButton)
				.Text(LOCTEXT("ConvertFromPaToDaButtonText", "Convert Collision From PhysicsAsset to DataAsset"))
				.ToolTipText(LOCTEXT("ConvertFromPaToDaButtonToolTip", "Convert CollisionPhysicsAsset to CollisionDataAsset(Need to save the DataAsset manually after convert)"))
				.OnClicked(FOnClicked::CreateStatic(&ULKAnimGraphNode_AnimVerlet::ConvertFromPaToDaButtonClicked, &DetailBuilder))
		];

	FDetailWidgetRow& ConvertFromPaWidgetRow = AnimVerletToolCategory.AddCustomRow(LOCTEXT("ConvertFromPhysicsAssetRow", "ConvertCollisionFromPhysicsAsset"));
	ConvertFromPaWidgetRow
		[
			SNew(SButton)
				.Text(LOCTEXT("ConvertFromPaButtonText", "Convert Collision From PhysicsAsset"))
				.ToolTipText(LOCTEXT("ConvertFromPaButtonToolTip", "Convert CollisionPhysicsAsset to collision shape data list(Need to save the AnimBlueprint manually after convert)"))
				.OnClicked(FOnClicked::CreateStatic(&ULKAnimGraphNode_AnimVerlet::ConvertFromPaButtonClicked, &DetailBuilder))
		];
}

void ULKAnimGraphNode_AnimVerlet::Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* PreviewSkelMeshComp) const
{
	if (LastPreviewComponent != PreviewSkelMeshComp)
		LastPreviewComponent = PreviewSkelMeshComp;

	if (PreviewSkelMeshComp == nullptr)
		return;

	FLKAnimNode_AnimVerlet* AnimVerletNode = GetActiveInstanceNode<FLKAnimNode_AnimVerlet>(PreviewSkelMeshComp->GetAnimInstance());
	if (AnimVerletNode == nullptr)
		return;

	if (bShowBones)
	{
		const TArray<FLKAnimVerletBone>& AnimVerletBones = AnimVerletNode->GetSimulateBones();
		for (const FLKAnimVerletBone& CurBone : AnimVerletBones)
		{
			const bool bSleep = (bShowSleep && CurBone.IsSleep());
			DrawWireSphere(PDI, CurBone.Location, bSleep ? FColor::Turquoise : (CurBone.bFakeBone ? FColor::Black : FColor::Yellow), CurBone.Thickness * BoneThicknessRenderScale, 16, SDPG_Foreground);
		}

		if (bShowCapsuleBoneChainConstraints && AnimVerletNode->bUseCapsuleCollisionForChain)
		{
			const TArray<FLKAnimVerletBoneIndicatorPair>& AnimVerletBoneIndicatorPairList = AnimVerletNode->GetSimulateBonePairIndicators();
			for (const FLKAnimVerletBoneIndicatorPair& CurPair : AnimVerletBoneIndicatorPairList)
			{
				if (CurPair.BoneB.IsValidBoneIndicator() == false || AnimVerletBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) == false)
					continue;

				const FLKAnimVerletBone& CurVerletBone = AnimVerletBones[CurPair.BoneB.AnimVerletBoneIndex];
				if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
					continue;

				const FLKAnimVerletBone& ParentVerletBone = AnimVerletBones[CurPair.BoneA.AnimVerletBoneIndex];
				const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(ParentVerletBone.Location - CurVerletBone.Location).ToQuat();
				DrawWireCapsule(PDI, (CurVerletBone.Location + ParentVerletBone.Location) * 0.5f, CapsuleRotation.GetAxisX(), CapsuleRotation.GetAxisY(), CapsuleRotation.GetAxisZ(), FColor::Emerald, CurVerletBone.Thickness * BoneThicknessRenderScale, ((CurVerletBone.Location - ParentVerletBone.Location).Size() * 0.5f) + CurVerletBone.Thickness * BoneThicknessRenderScale, 16, SDPG_Foreground);
			}
		}
	}

	if (bShowBoneBounds)
	{
		const TArray<FLKAnimVerletBone>& AnimVerletBones = AnimVerletNode->GetSimulateBones();
		if (AnimVerletNode->bUseCapsuleCollisionForChain == false)
		{
			for (const FLKAnimVerletBone& CurBone : AnimVerletBones)
			{
				const FLKAnimVerletBound CurBound = CurBone.MakeBound();
				const FTransform BoxT(FQuat::Identity, CurBound.GetCenter());
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurBound.GetHalfExtents(), CurBound.GetHalfExtents());
				DrawWireBox(PDI, BoxMat, Box, FColor::Green, SDPG_Foreground);
			}
		}
		else
		{
			const TArray<FLKAnimVerletBoneIndicatorPair>& AnimVerletBoneIndicatorPairList = AnimVerletNode->GetSimulateBonePairIndicators();
			for (const FLKAnimVerletBoneIndicatorPair& CurPair : AnimVerletBoneIndicatorPairList)
			{
				if (CurPair.BoneB.IsValidBoneIndicator() == false || AnimVerletBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) == false)
					continue;

				const FLKAnimVerletBone& CurVerletBone = AnimVerletBones[CurPair.BoneB.AnimVerletBoneIndex];
				if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
				{
					const FLKAnimVerletBound CurBound = CurVerletBone.MakeBound();
					const FTransform BoxT(FQuat::Identity, CurBound.GetCenter());
					const FMatrix BoxMat = BoxT.ToMatrixNoScale();
					const FBox Box(-CurBound.GetHalfExtents(), CurBound.GetHalfExtents());
					DrawWireBox(PDI, BoxMat, Box, FColor::Green, SDPG_Foreground);

					continue;
				}

				const FLKAnimVerletBone& ParentVerletBone = AnimVerletBones[CurPair.BoneA.AnimVerletBoneIndex];
				
				const FLKAnimVerletBound ParentBound = FLKAnimVerletBone::MakeBound(ParentVerletBone.Location, CurVerletBone.Thickness);
				FLKAnimVerletBound CurBound = CurVerletBone.MakeBound();
				CurBound.Expand(ParentBound);

				const FTransform BoxT(FQuat::Identity, CurBound.GetCenter());
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurBound.GetHalfExtents(), CurBound.GetHalfExtents());
				DrawWireBox(PDI, BoxMat, Box, FColor::Green, SDPG_Foreground);
			}
		}
	}

	if (bShowConstraints)
	{
		const TArray<FLKAnimVerletConstraint_Distance>& DistanceConstraints = AnimVerletNode->GetDistanceConstraints();
		for (const FLKAnimVerletConstraint_Distance& CurConstraint : DistanceConstraints)
			PDI->DrawLine(CurConstraint.BoneA->Location, CurConstraint.BoneB->Location, FColor::White, SDPG_Foreground);

		if (bShowFixedPoints)
		{
			const TArray<FLKAnimVerletConstraint_Pin>& PinConstraints = AnimVerletNode->GetPinConstraints();
			for (const FLKAnimVerletConstraint_Pin& PinConstraint : PinConstraints)
				DrawWireSphere(PDI, PinConstraint.Bone->Location, FColor::Red, PinConstraint.Bone->Thickness * 2, 16, SDPG_Foreground);
		}

		if (bShowSimulatingBallSocketConstraints)
		{
			const TArray<FLKAnimVerletConstraint_BallSocket>& BallSocketConstraints = AnimVerletNode->GetBallSocketConstraints();
			for (const FLKAnimVerletConstraint_BallSocket& CurConstraint : BallSocketConstraints)
			{
				FVector ConstraintVector = FVector::ZeroVector;
				if (CurConstraint.GrandParentBoneNullable != nullptr && CurConstraint.ParentBoneNullable != nullptr)
				{
					const FVector GrandParentToParent = (CurConstraint.ParentBoneNullable->Location - CurConstraint.GrandParentBoneNullable->Location);
					ConstraintVector = GrandParentToParent;
				}
				else
				{
					const FVector PoseAToPoseB = (CurConstraint.BoneB->PoseLocation - CurConstraint.BoneA->PoseLocation);
					ConstraintVector = PoseAToPoseB;
				}

				const FTransform ParentBoneTransform = FTransform(FQuat::FindBetween(FVector::ForwardVector, ConstraintVector), CurConstraint.BoneA->Location);
				DrawWireCone(PDI, VertexSpaceCache, ParentBoneTransform, ConstraintVector.Size(), CurConstraint.AngleDegrees, 16, FColor::Magenta, SDPG_Foreground);
				VertexSpaceCache.Reset();
			}
			VertexSpaceCache.Reset();
		}

		/// Show simulating local collision constraints
		if (bShowSimulatingSphereCollisionConstraints)
		{
			const TArray<FLKAnimVerletConstraint_Sphere>& SphereCollisionConstraints = AnimVerletNode->GetSphereCollisionConstraints();
			for (const FLKAnimVerletConstraint_Sphere& CurConstraint : SphereCollisionConstraints)
				DrawWireSphere(PDI, CurConstraint.Location, FColor::Blue, CurConstraint.Radius, 16, SDPG_Foreground);
		}
		if (bShowSimulatingSphereCollisionConstraintsBounds)
		{
			const TArray<FLKAnimVerletConstraint_Sphere>& SphereCollisionConstraints = AnimVerletNode->GetSphereCollisionConstraints();
			for (const FLKAnimVerletConstraint_Sphere& CurConstraint : SphereCollisionConstraints)
			{
				const FLKAnimVerletBound CurBound = CurConstraint.MakeBound();
				const FTransform BoxT(FQuat::Identity, CurBound.GetCenter());
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurBound.GetHalfExtents(), CurBound.GetHalfExtents());
				DrawWireBox(PDI, BoxMat, Box, FColor::Green, SDPG_Foreground);
			}
		}

		if (bShowSimulatingCapsuleCollisionConstraints)
		{
			const TArray<FLKAnimVerletConstraint_Capsule>& CapsuleCollisionConstraints = AnimVerletNode->GetCapsuleCollisionConstraints();
			for (const FLKAnimVerletConstraint_Capsule& CurConstraint : CapsuleCollisionConstraints)
				DrawWireCapsule(PDI, CurConstraint.Location, CurConstraint.Rotation.GetAxisX(), CurConstraint.Rotation.GetAxisY(), CurConstraint.Rotation.GetAxisZ(), FColor::Blue, CurConstraint.Radius, CurConstraint.HalfHeight + CurConstraint.Radius, 16, SDPG_Foreground);
		}
		if (bShowSimulatingCapsuleCollisionConstraintsBounds)
		{
			const TArray<FLKAnimVerletConstraint_Capsule>& CapsuleCollisionConstraints = AnimVerletNode->GetCapsuleCollisionConstraints();
			for (const FLKAnimVerletConstraint_Capsule& CurConstraint : CapsuleCollisionConstraints)
			{
				const FLKAnimVerletBound CurBound = CurConstraint.MakeBound();
				const FTransform BoxT(FQuat::Identity, CurBound.GetCenter());
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurBound.GetHalfExtents(), CurBound.GetHalfExtents());
				DrawWireBox(PDI, BoxMat, Box, FColor::Green, SDPG_Foreground);
			}
		}

		if (bShowSimulatingBoxCollisionConstraints)
		{
			const TArray<FLKAnimVerletConstraint_Box>& BoxCollisionConstraints = AnimVerletNode->GetBoxCollisionConstraints();
			for (const FLKAnimVerletConstraint_Box& CurConstraint : BoxCollisionConstraints)
			{
				const FTransform BoxT(CurConstraint.Rotation, CurConstraint.Location);
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurConstraint.HalfExtents, CurConstraint.HalfExtents);
				DrawWireBox(PDI, BoxMat, Box, FColor::Blue, SDPG_Foreground);
			}
		}
		if (bShowSimulatingBoxCollisionConstraintsBounds)
		{
			const TArray<FLKAnimVerletConstraint_Box>& BoxCollisionConstraints = AnimVerletNode->GetBoxCollisionConstraints();
			for (const FLKAnimVerletConstraint_Box& CurConstraint : BoxCollisionConstraints)
			{
				const FLKAnimVerletBound CurBound = CurConstraint.MakeBound();
				const FTransform BoxT(FQuat::Identity, CurBound.GetCenter());
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurBound.GetHalfExtents(), CurBound.GetHalfExtents());
				DrawWireBox(PDI, BoxMat, Box, FColor::Green, SDPG_Foreground);
			}
		}

		if (bShowSimulatingPlaneCollisionConstraints)
		{
			const TArray<FLKAnimVerletConstraint_Plane>& PlaneCollisionConstraints = AnimVerletNode->GetPlaneCollisionConstraints();
			for (const FLKAnimVerletConstraint_Plane& CurConstraint : PlaneCollisionConstraints)
			{
				const FTransform PlaneT(CurConstraint.Rotation, CurConstraint.PlaneBase);
				const FMatrix PlaneMat = PlaneT.ToMatrixNoScale();
				if (CurConstraint.PlaneHalfExtents.IsNearlyZero() == false)
				{
					const FVector PlaneBoxExtents(CurConstraint.PlaneHalfExtents.X, CurConstraint.PlaneHalfExtents.Y, 1.0f);
					const FBox Box(-PlaneBoxExtents, PlaneBoxExtents);
					DrawWireBox(PDI, PlaneMat, Box, FColor::Orange, SDPG_Foreground);
					DrawDirectionalArrow(PDI, FRotationMatrix(FRotator(90.0f, 0.0f, 0.0f)) * PlaneMat, FLinearColor::Gray, 50.0f, 20.0f, SDPG_Foreground, 0.5f);
				}
				else
				{
					///DrawPlane10x10(PDI, PlaneMat, 200.0f, FVector2D(0.0f, 0.0f), FVector2D(1.0f, 1.0f), GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy(), SDPG_World);
					const FVector PlaneBoxExtents(100.0f, 100.0f, 1.0f);
					const FBox Box(-PlaneBoxExtents, PlaneBoxExtents);
					DrawWireBox(PDI, PlaneMat, Box, FColor::Orange, SDPG_Foreground);
					DrawDirectionalArrow(PDI, FRotationMatrix(FRotator(90.0f, 0.0f, 0.0f)) * PlaneMat, FLinearColor::Gray, 50.0f, 20.0f, SDPG_Foreground, 0.5f);
				}
			}
		}
	}

	if (bShowIsometricBendingConstraints)
	{
		const TArray<FLKAnimVerletConstraint_IsometricBending>& IsometricBendingConstraints = AnimVerletNode->GetIsometricBendingConstraints();
		for (const FLKAnimVerletConstraint_IsometricBending& CurConstraint : IsometricBendingConstraints)
		{
			PDI->DrawLine(CurConstraint.BoneA->Location, CurConstraint.BoneB->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneB->Location, CurConstraint.BoneC->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneC->Location, CurConstraint.BoneA->Location, FColor::White, SDPG_Foreground);

			PDI->DrawLine(CurConstraint.BoneD->Location, CurConstraint.BoneB->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneB->Location, CurConstraint.BoneC->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneC->Location, CurConstraint.BoneD->Location, FColor::White, SDPG_Foreground);
		}
	}

	if (bShowFlatBendingConstraints)
	{
		const TArray<FLKAnimVerletConstraint_FlatBending>& FlatBendingConstraints = AnimVerletNode->GetFlatBendingConstraints();
		for (const FLKAnimVerletConstraint_FlatBending& CurConstraint : FlatBendingConstraints)
		{
			PDI->DrawLine(CurConstraint.BoneA->Location, CurConstraint.BoneB->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneB->Location, CurConstraint.BoneC->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneC->Location, CurConstraint.BoneA->Location, FColor::White, SDPG_Foreground);

			PDI->DrawLine(CurConstraint.BoneD->Location, CurConstraint.BoneB->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneB->Location, CurConstraint.BoneC->Location, FColor::White, SDPG_Foreground);
			PDI->DrawLine(CurConstraint.BoneC->Location, CurConstraint.BoneD->Location, FColor::White, SDPG_Foreground);
		}
	}
}

void ULKAnimGraphNode_AnimVerlet::ShowNotification(const FText& InText, bool bSuccess)
{
	FNotificationInfo NotificationInfo(InText);
	NotificationInfo.ExpireDuration = 5.0f;

	TSharedPtr<SNotificationItem> NotificationItem = FSlateNotificationManager::Get().AddNotification(NotificationInfo);
	NotificationItem->SetCompletionState(bSuccess ? SNotificationItem::CS_Success : SNotificationItem::CS_Fail);
}

void ULKAnimGraphNode_AnimVerlet::ResetSimulation()
{
	FLKAnimNode_AnimVerlet* PreviewNode = GetPreviewAnimVerletNode();
	if (PreviewNode != nullptr)
	{
		PreviewNode->ResetDynamics(ETeleportType::ResetPhysics);
	}
}
FReply ULKAnimGraphNode_AnimVerlet::ResetSimulationButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder)
{
	const TArray<TWeakObjectPtr<UObject>>& SelectedObjectsList = DetailLayoutBuilder->GetSelectedObjects();
	for (TWeakObjectPtr<UObject> Object : SelectedObjectsList)
	{
		if (ULKAnimGraphNode_AnimVerlet* AnimVerletGraphNode = Cast<ULKAnimGraphNode_AnimVerlet>(Object.Get()))
		{
			AnimVerletGraphNode->ResetSimulation();
		}
	}

	return FReply::Handled();
}

void ULKAnimGraphNode_AnimVerlet::ConvertCollisionShapesToDataAsset()
{
	const bool bResult = Node.ConvertCollisionShapesToDataAsset();
	if (bResult)
	{
		ShowNotification(FText::FromString(TEXT("Converted to CollisionDataAsset")), true);
	}
	else
	{
		ShowNotification(FText::FromString(TEXT("CollisionDataAsset is required")), false);
	}
}
FReply ULKAnimGraphNode_AnimVerlet::ConvertToDaButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder)
{
	const TArray<TWeakObjectPtr<UObject>>& SelectedObjectsList = DetailLayoutBuilder->GetSelectedObjects();
	for (TWeakObjectPtr<UObject> Object : SelectedObjectsList)
	{
		if (ULKAnimGraphNode_AnimVerlet* AnimVerletGraphNode = Cast<ULKAnimGraphNode_AnimVerlet>(Object.Get()))
		{
			AnimVerletGraphNode->ConvertCollisionShapesToDataAsset();
		}
	}

	return FReply::Handled();
}

void ULKAnimGraphNode_AnimVerlet::ConvertCollisionShapesFromDataAsset()
{
	const bool bResult = Node.ConvertCollisionShapesFromDataAsset();
	if (bResult)
	{
		ShowNotification(FText::FromString(TEXT("Converted from CollisionDataAsset")), true);
	}
	else
	{
		ShowNotification(FText::FromString(TEXT("CollisionDataAsset is required")), false);
		return;
	}

	FLKAnimNode_AnimVerlet* PreviewNode = GetPreviewAnimVerletNode();
	if (PreviewNode != nullptr)
	{
		PreviewNode->ConvertCollisionShapesFromDataAsset();
	}
}
FReply ULKAnimGraphNode_AnimVerlet::ConvertFromDaButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder)
{
	const TArray<TWeakObjectPtr<UObject>>& SelectedObjectsList = DetailLayoutBuilder->GetSelectedObjects();
	for (TWeakObjectPtr<UObject> Object : SelectedObjectsList)
	{
		if (ULKAnimGraphNode_AnimVerlet* AnimVerletGraphNode = Cast<ULKAnimGraphNode_AnimVerlet>(Object.Get()))
		{
			AnimVerletGraphNode->ConvertCollisionShapesFromDataAsset();
		}
	}

	return FReply::Handled();
}

void ULKAnimGraphNode_AnimVerlet::ConvertPhysicsAssetToDataAsset()
{
	const bool bResult = Node.ConvertPhysicsAssetToDataAsset();
	if (bResult)
	{
		ShowNotification(FText::FromString(TEXT("Converted to CollisionDataAsset")), true);
	}
	else
	{
		ShowNotification(FText::FromString(TEXT("CollisionPhysicsAsset or CollisionDataAsset is required")), false);
	}
}
FReply ULKAnimGraphNode_AnimVerlet::ConvertFromPaToDaButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder)
{
	const TArray<TWeakObjectPtr<UObject>>& SelectedObjectsList = DetailLayoutBuilder->GetSelectedObjects();
	for (TWeakObjectPtr<UObject> Object : SelectedObjectsList)
	{
		if (ULKAnimGraphNode_AnimVerlet* AnimVerletGraphNode = Cast<ULKAnimGraphNode_AnimVerlet>(Object.Get()))
		{
			AnimVerletGraphNode->ConvertPhysicsAssetToDataAsset();
		}
	}

	return FReply::Handled();
}

void ULKAnimGraphNode_AnimVerlet::ConvertCollisionShapesFromPhysicsAsset()
{
	const bool bResult = Node.ConvertCollisionShapesFromPhysicsAsset();
	if (bResult)
	{
		ShowNotification(FText::FromString(TEXT("Converted from CollisionPhysicsAsset")), true);
	}
	else
	{
		ShowNotification(FText::FromString(TEXT("CollisionPhysicsAsset is required")), false);
		return;
	}

	FLKAnimNode_AnimVerlet* PreviewNode = GetPreviewAnimVerletNode();
	if (PreviewNode != nullptr)
	{
		PreviewNode->ConvertCollisionShapesFromPhysicsAsset();
	}
}
FReply ULKAnimGraphNode_AnimVerlet::ConvertFromPaButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder)
{
	const TArray<TWeakObjectPtr<UObject>>& SelectedObjectsList = DetailLayoutBuilder->GetSelectedObjects();
	for (TWeakObjectPtr<UObject> Object : SelectedObjectsList)
	{
		if (ULKAnimGraphNode_AnimVerlet* AnimVerletGraphNode = Cast<ULKAnimGraphNode_AnimVerlet>(Object.Get()))
		{
			AnimVerletGraphNode->ConvertCollisionShapesFromPhysicsAsset();
		}
	}

	return FReply::Handled();
}

void ULKAnimGraphNode_AnimVerlet::ApplyAnimVerletPresetType(FLKAnimNode_AnimVerlet& InNode)
{
	InNode.ApplyPresetType(PresetType);
}

FLKAnimNode_AnimVerlet* ULKAnimGraphNode_AnimVerlet::GetPreviewAnimVerletNode() const
{
	FLKAnimNode_AnimVerlet* ActivePreviewNode = nullptr;

	if (LastPreviewComponent && LastPreviewComponent->GetAnimInstance())
	{
		UAnimInstance* Instance = LastPreviewComponent->GetAnimInstance();
		if (UAnimBlueprintGeneratedClass* Class = Cast<UAnimBlueprintGeneratedClass>(Instance->GetClass()))
		{
			ActivePreviewNode = Class->GetPropertyInstance<FLKAnimNode_AnimVerlet>(Instance, NodeGuid);
		}
	}
	return ActivePreviewNode;
}

void ULKAnimGraphNode_AnimVerlet::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
}
#undef LOCTEXT_NAMESPACE