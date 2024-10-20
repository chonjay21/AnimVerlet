#include "LKAnimGraphNode_AnimVerlet.h"

#include <Animation/AnimInstance.h>
#include <AnimNodeEditModes.h>
#include <DetailLayoutBuilder.h>
#include <DetailWidgetRow.h>
#include <DetailCategoryBuilder.h>
#include <EngineGlobals.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <PropertyHandle.h>
#include <Widgets/Input/SButton.h>

#define LOCTEXT_NAMESPACE "AnimVerlet"
ULKAnimGraphNode_AnimVerlet::ULKAnimGraphNode_AnimVerlet(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText ULKAnimGraphNode_AnimVerlet::GetControllerDescription() const
{
	return LOCTEXT("AnimVerlet", "AnimVerlet");
}

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

void ULKAnimGraphNode_AnimVerlet::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	FLKAnimNode_AnimVerlet* PreviewNode = GetPreviewAnimVerletNode();
	if (PreviewNode != nullptr)
	{
		PreviewNode->ForceClearSimulateBones();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}

void ULKAnimGraphNode_AnimVerlet::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	Super::CustomizeDetails(DetailBuilder);

	IDetailCategoryBuilder& PreviewCategory = DetailBuilder.EditCategory(TEXT("Preview"));
	FDetailWidgetRow& WidgetRow = PreviewCategory.AddCustomRow(LOCTEXT("ResetButtonRow", "Reset"));

	WidgetRow
		[
			SNew(SButton)
			.Text(LOCTEXT("ResetButtonText", "Reset Simulation"))
		.ToolTipText(LOCTEXT("ResetButtonToolTip", "Resets the simulation for this node"))
		.OnClicked(FOnClicked::CreateStatic(&ULKAnimGraphNode_AnimVerlet::ResetSimulationButtonClicked, &DetailBuilder))
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
			DrawWireSphere(PDI, CurBone.Location, bSleep ? FColor::Turquoise : (CurBone.bFakeBone ? FColor::Black : FColor::Yellow), AnimVerletNode->Thickness * BoneThicknessRenderScale, 16, SDPG_Foreground);
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
				DrawWireSphere(PDI, PinConstraint.Bone->Location, FColor::Red, AnimVerletNode->Thickness * 2, 16, SDPG_Foreground);
		}

		if (bShowBallSocketConstraints)
		{
			const TArray<FLKAnimVerletConstraint_BallSocket>& BallSocketConstraints = AnimVerletNode->GetBallSocketConstraints();
			for (const FLKAnimVerletConstraint_BallSocket& CurConstraint : BallSocketConstraints)
			{
				const FTransform ParentBoneTransform = FTransform(FQuat::FindBetween(FVector::ForwardVector, CurConstraint.BoneB->PoseLocation - CurConstraint.BoneA->PoseLocation), CurConstraint.BoneA->Location);
				DrawWireCone(PDI, VertexSpaceCache, ParentBoneTransform, (CurConstraint.BoneB->PoseLocation - CurConstraint.BoneA->PoseLocation).Size(), AnimVerletNode->ConeAngle, 16, FColor::Magenta, SDPG_Foreground);
				VertexSpaceCache.Reset();
			}
			VertexSpaceCache.Reset();
		}

		if (bShowLocalCollisionConstraints)
		{
			const TArray<FLKAnimVerletConstraint_Sphere>& SphereCollisionConstraints = AnimVerletNode->GetSphereCollisionConstraints();
			for (const FLKAnimVerletConstraint_Sphere& CurConstraint : SphereCollisionConstraints)
				DrawWireSphere(PDI, CurConstraint.Location, FColor::Blue, CurConstraint.Radius, 16, SDPG_Foreground);

			const TArray<FLKAnimVerletConstraint_Capsule>& CapsuleCollisionConstraints = AnimVerletNode->GetCapsuleCollisionConstraints();
			for (const FLKAnimVerletConstraint_Capsule& CurConstraint : CapsuleCollisionConstraints)
				DrawWireCapsule(PDI, CurConstraint.Location, CurConstraint.Rotation.GetAxisX(), CurConstraint.Rotation.GetAxisY(), CurConstraint.Rotation.GetAxisZ(), FColor::Blue, CurConstraint.Radius, CurConstraint.HalfHeight, 16, SDPG_Foreground);

			const TArray<FLKAnimVerletConstraint_Box>& BoxCollisionConstraints = AnimVerletNode->GetBoxCollisionConstraints();
			for (const FLKAnimVerletConstraint_Box& CurConstraint : BoxCollisionConstraints)
			{
				const FTransform BoxT(CurConstraint.Rotation, CurConstraint.Location);
				const FMatrix BoxMat = BoxT.ToMatrixNoScale();
				const FBox Box(-CurConstraint.HalfExtents, CurConstraint.HalfExtents);
				DrawWireBox(PDI, BoxMat, Box, FColor::Blue, SDPG_Foreground);
			}

			if (bShowPlaneCollisionConstraints)
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
	}
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