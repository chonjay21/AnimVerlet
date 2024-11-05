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
	static FReply ConvertToDaButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder);
	static FReply ConvertFromDaButtonClicked(IDetailLayoutBuilder* DetailLayoutBuilder);

public:
	ULKAnimGraphNode_AnimVerlet(const FObjectInitializer& ObjectInitializer);

	void ResetSimulation();
	void ConvertCollisionShapesToDataAsset();
	void ConvertCollisionShapesFromDataAsset();
	FLKAnimNode_AnimVerlet* GetPreviewAnimVerletNode() const;

public:
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	///virtual void PostEditChangeChainProperty(struct FPropertyChangedChainEvent& PropertyChangedEvent) override;
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FEditorModeID GetEditorMode() const override;
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	virtual void Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* PreviewSkelMeshComp) const override;
	virtual void Serialize(FArchive& Ar) override;

protected:
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }

private:
	void ShowNotification(const FText& InText, bool bSuccess);

private:
	UPROPERTY(Transient)
	mutable USkeletalMeshComponent* LastPreviewComponent = nullptr;

public:
	UPROPERTY(EditAnywhere, Category = "Settings")
	FLKAnimNode_AnimVerlet Node;

	/** Show and modify(in preview screen) collisions*/
	UPROPERTY(EditAnywhere, Category = "CollisionInput", meta = (DisplayPriority = "1"))
	bool bShowAndModifySphereCollision = true;
	UPROPERTY(EditAnywhere, Category = "CollisionInput", meta = (DisplayPriority = "1"))
	bool bShowAndModifyCapsuleCollision = true;
	UPROPERTY(EditAnywhere, Category = "CollisionInput", meta = (DisplayPriority = "1"))
	bool bShowAndModifyBoxCollision = true;
	UPROPERTY(EditAnywhere, Category = "CollisionInput", meta = (DisplayPriority = "1"))
	bool bShowAndModifyPlaneCollision = true;

	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bShowBones = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowBones", EditConditionHides, ClampMin = "0.0"))
	float BoneThicknessRenderScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowBones", EditConditionHides))
	bool bShowSleep = true;

	/** Show simulating(applying) constraints */
	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bShowConstraints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowFixedPoints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowSimulatingBallSocketConstraints = true;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowSimulatingSphereCollisionConstraints = false;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowSimulatingCapsuleCollisionConstraints = false;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowSimulatingBoxCollisionConstraints = false;
	UPROPERTY(EditAnywhere, Category = "Preview", meta = (EditCondition = "bShowConstraints", EditConditionHides))
	bool bShowSimulatingPlaneCollisionConstraints = false;

private:
	FNodeTitleTextTable CachedNodeTitles;
	mutable TArray<FVector> VertexSpaceCache;
};