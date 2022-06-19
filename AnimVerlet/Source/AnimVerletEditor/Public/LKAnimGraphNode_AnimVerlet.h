#pragma once
#include <AnimGraphNode_SkeletalControlBase.h>
#include <EdGraph/EdGraphNodeUtils.h>
#include "LKAnimNode_AnimVerlet.h"
#include "LKAnimGraphNode_AnimVerlet.generated.h"

UCLASS()
class ULKAnimGraphNode_AnimVerlet : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_BODY()

public:
	static FReply ResetSimulationButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder);

public:
	ULKAnimGraphNode_AnimVerlet(const FObjectInitializer& ObjectInitializer);

	void ResetSimulation();
	FLKAnimNode_AnimVerlet* GetPreviewAnimVerletNode() const;

public:
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FEditorModeID GetEditorMode() const override;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	virtual void Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* PreviewSkelMeshComp) const override;
	virtual void Serialize(FArchive& Ar) override;

protected:
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }

private:
	UPROPERTY(Transient)
	mutable USkeletalMeshComponent* LastPreviewComponent = nullptr;

public:
	UPROPERTY(EditAnywhere, Category = "Settings")
	FLKAnimNode_AnimVerlet Node;

	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bShowBones = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowBones", EditConditionHides, ClampMin = "0.0"))
	float BoneThicknessRenderScale = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bShowConstraints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowFixedPoints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowBallSocketConstraints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowLocalCollisionConstraints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints && bShowLocalCollisionConstraints", EditConditionHides))
	bool bShowPlaneCollisionConstraints = true;

private:
	FNodeTitleTextTable CachedNodeTitles;
	mutable TArray<FVector> VertexSpaceCache;
};