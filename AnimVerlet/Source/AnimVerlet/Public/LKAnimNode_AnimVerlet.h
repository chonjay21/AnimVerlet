#pragma once
#include <CoreMinimal.h>
#include <BoneControllers/AnimNode_SkeletalControlBase.h>
#include "LKAnimverletBone.h"
#include "LKAnimVerletCollisionShape.h"
#include "LKAnimverletConstraint.h"
#include "LKAnimverletSetting.h"
#include "LKAnimNode_AnimVerlet.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct ANIMVERLET_API FLKAnimNode_AnimVerlet : public FAnimNode_SkeletalControlBase
{
	GENERATED_BODY()

public:
	FLKAnimNode_AnimVerlet();

public:
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual bool NeedsDynamicReset() const override { return true; }
	virtual void ResetDynamics(ETeleportType InTeleportType) override;

	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual void UpdateInternal(const FAnimationUpdateContext& Context) override;

protected:
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;

private:
	void InitializeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer);
	bool MakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton,
						   int32 BoneIndex, int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting);
	bool WalkChildsAndMakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton,
										int32 BoneIndex, int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting);
	void MakeFakeBoneTransform(OUT FTransform& OutTransform, int32 ParentSimulateBoneIndex) const;

	void PrepareSimulation(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer);
	void PrepareLocalCollisionConstraints(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer);
	void SimulateVerlet(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform);
	void ApplyResult(OUT TArray<FBoneTransform>& OutBoneTransforms, const FBoneContainer& BoneContainer);
	void ClearSimulateBones();
	void ResetSimulation();

public:
	const TArray<FLKAnimVerletBone>& GetSimulateBones() const { return SimulateBones; }
	const TArray<FLKAnimVerletConstraint_Distance>& GetDistanceConstraints() const { return DistanceConstraints; }
	const TArray<FLKAnimVerletConstraint_BallSocket>& GetBallSocketConstraints() const { return BallSocketConstraints; }
	const TArray<FLKAnimVerletConstraint_Pin>& GetPinConstraints() const { return PinConstraints; }
	const TArray<FLKAnimVerletConstraint_Sphere>& GetSphereCollisionConstraints() const { return SphereCollisionConstraints; }
	const TArray<FLKAnimVerletConstraint_Capsule>& GetCapsuleCollisionConstraints() const { return CapsuleCollisionConstraints; }
	const TArray<FLKAnimVerletConstraint_Box>& GetBoxCollisionConstraints() const { return BoxCollisionConstraints; }
	const TArray<FLKAnimVerletConstraint_Plane>& GetPlaneCollisionConstraints() const { return PlaneCollisionConstraints; }
	void ForceClearSimulateBones() { ClearSimulateBones(); }	/// for live editor preview

public:
	/** Input the starting bone of the cloth sequentially. */
	UPROPERTY(EditAnywhere, Category = "Setup", meta = (DisplayPriority = "1"))
	TArray<FLKAnimVerletBoneSetting> VerletBones;
	/** Insert a fake(virtual) bone in the middle of the chain.(It can affect the softness of the cloth or the collision reaction)  */
	UPROPERTY(EditAnywhere, Category = "Setup")
	bool bSubDivideBones = false;
	UPROPERTY(EditAnywhere, Category = "Setup", meta = (EditCondition = "SubDivideBones", EditConditionHides, ClampMin = "0"))
	uint8 NumSubDividedBone = 1;

	/** Magnitude of inertia due to animation pose change. */
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreReferencePose == false", EditConditionHides, ClampMin = "0.0", ClampMax = "1.0"))
	float ReferencePoseDeltaInertia = 0.03f;
	/** Magnitude of inertia scale due to animation pose change. */
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreReferencePose == false", EditConditionHides, ClampMin = "0.0"))
	float ReferencePoseDeltaInertiaScale = 1.0f;
	/** Limits the amount of inertia caused by animation pose changes. */
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreReferencePose == false", EditConditionHides))
	bool bClampReferencePoseDeltaInertia = true;
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreReferencePose == false && bClampReferencePoseDeltaInertia", EditConditionHides, ClampMin = "0.0"))
	float ReferencePoseDeltaInertiaClampMax = 0.1f;
	/** Ignores animation poses and only applies cloth simulation results. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Settings")
	bool bIgnoreReferencePose = false;
	/** The magnitude of inertia to the animation pose position.(Pulls the cloth simulation result into the animation pose position) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Settings", meta = (EditCondition = "bIgnoreReferencePose == false", EditConditionHides, ClampMin = "0.0", ClampMax = "1.0"))
	float ReferencePoseInertia = 0.03f;

	/** Adds an fake(virtual) bone to the end of the body.(may affect the rotation or collision of the end bone) */
	UPROPERTY(EditAnywhere, Category = "Settings")
	bool bMakeFakeTipBone = true;
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bMakeFakeTipBone", ClampMin = "0.1", ForceUnits = "cm"))
	float FakeTipBoneLength = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solve", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.9f;

	/** Stiffness applied to bone to bone distance when calculating verlet integration. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solve", meta = (ClampMin = "0.0"))
	float Stiffness = 0.8f;

	/** The option to keep the distance between parent and child bones in the bone chain helps to keep SolveIteration small. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bKeepLengthFromParent = true;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bKeepLengthFromParent", ClampMin = "0.0", ForceUnits = "cm"))
	float LengthFromParentMargin = 0.1f;
	/** The option to keep the distance between side bones helps to keep SolveIteration small. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bKeepSideLength = true;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bKeepSideLength", ClampMin = "0.0", ForceUnits = "cm"))
	float SideLengthMargin = 3.0f;

	/** 
		It is the number of iterations to solve Verlet Integraion. 
		As there are more Collisions or Constraints, the larger the SolveIteration the more accurate the result. 
		SolveIteration of 1 can be fine thanks to the bKeepLengthFromParent and bKeepSideLength options.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "1"))
	int32 SolveIteration = 1;

	/** Use a fixed DeltaTime instead of real delta time if > 0. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "0.0", ForceUnits = "s"))
	float FixedDeltaTime = 0.0f;

	/** Limit delta time in situations where the frame rate fluctuates. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "0.0", ForceUnits = "s"))
	float MaxDeltaTime = 0.05f;


	/** Angle to use when constraining using a cone.(Ball - Socket joint constraints) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint", meta = (PinHiddenByDefault, ClampMin = "0.0", ClampMax = "90.0", ForceUnits = "deg"))
	float ConeAngle = 0.0f;

	/** The virtual thickness of the bone to be used in calculating various collisions and constraints.(radius) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Thickness = 0.3f;

	/** Enable collision against to world.(May cause performance impact by physics sweep test for each bone and fake bone) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	FName WorldCollisionProfile = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionSphere> SphereCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionCapsule> CapsuleCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionBox> BoxCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionPlane> PlaneCollisionShapes;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity", meta = (PinHiddenByDefault, ForceUnits = "cm/s"))
	FVector Gravity = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity", meta = (PinHiddenByDefault))
	bool bGravityInWorldSpace = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forces", meta = (PinHiddenByDefault, ForceUnits = "cm/s"))
	FVector ExternalForce = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forces", meta = (PinHiddenByDefault))
	bool bExternalForceInWorldSpace = true;

	/** Adjust virtual wind to random bone.(with out UWindDirectionalSourceComponent) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault))
	FVector RandomWindDirection = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault, ClampMin = "0.0", ForceUnits = "cm/s"))
	float RandomWindSizeMin = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault, ClampMin = "0.0", ForceUnits = "cm/s"))
	float RandomWindSizeMax = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault))
	bool bRandomWindDirectionInWorldSpace = true;

	/** Adjust UWindDirectionalSourceComponent in world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault))
	bool bAdjustWindComponent = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault, EditCondition = "bAdjustWindComponent"))
	float WindComponentScale = 1.0f;

	/** The scale to be applied to the inertia caused by the component's positional movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, ClampMin = "0.0"))
	float MoveInertiaScale = 1.0f;
	/** Limits the amount of inertia caused by the component's positional movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault))
	bool bClampMoveInertia = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, EditCondition = "bClampMoveInertia", EditConditionHides, ClampMin = "0.0", ForceUnits = "cm"))
	float MoveInertiaClampMaxDistance = 300.0f;
	/** The scale to be applied to the inertia caused by the component's rotational movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, ClampMin = "0.0"))
	float RotationInertiaScale = 1.0f;
	/** Limits the amount of inertia caused by the component's rotational movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault))
	bool bClampRotationInertia = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, EditCondition = "bClampRotationInertia", EditConditionHides, ForceUnits = "deg"))
	float RotationInertiaClampDegrees = 30.0f;

private:
	float DeltaTime = 0.0f;
	FTransform PrevComponentT = FTransform::Identity;
	TArray<FLKAnimVerletBone> SimulateBones;
	///TArray<FLKAnimVerletConstraint*> Constraints;
	/// Unroll each constratins for better solve result(considering constraint`s solving order)
	TArray<FLKAnimVerletConstraint_Pin> PinConstraints;
	TArray<FLKAnimVerletConstraint_Distance> DistanceConstraints;
	TArray<FLKAnimVerletConstraint_FixedDistance> FixedDistanceConstraints;
	TArray<FLKAnimVerletConstraint_BallSocket> BallSocketConstraints;
	TArray<FLKAnimVerletConstraint_Sphere> SphereCollisionConstraints;
	TArray<FLKAnimVerletConstraint_Capsule> CapsuleCollisionConstraints;
	TArray<FLKAnimVerletConstraint_Box> BoxCollisionConstraints;
	TArray<FLKAnimVerletConstraint_Plane> PlaneCollisionConstraints;
	TArray<FLKAnimVerletConstraint_World> WorldCollisionConstraints;
	TArray<TArray<int32>> BoneChainIndexes;
};