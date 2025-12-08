#pragma once
#include <CoreMinimal.h>
#include <BoneControllers/AnimNode_SkeletalControlBase.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletCollisionShape.h"
#include "LKAnimVerletConstraint.h"
#include "LKAnimVerletConstraintType.h"
#include "LKAnimVerletSetting.h"
#include "LKAnimNode_AnimVerlet.generated.h"

#if ENABLE_ANIM_DEBUG && ENABLE_VISUAL_LOG
#define LK_ENABLE_ANIMVERLET_DEBUG	(1)
#else
#define LK_ENABLE_ANIMVERLET_DEBUG	(0)
#endif

#define LK_ENABLE_STAT	(1)


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
	void InitializeLocalCollisionConstraints(const FBoneContainer& BoneContainer);
	void InitializeAttachedShape(struct FLKAnimVerletCollisionShape& InShape, const FBoneContainer& BoneContainer);
	bool MakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton, int32 BoneIndex, 
						   int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting, bool bParentExcluded, int32 ParentExcludedBoneIndex);
	bool WalkChildsAndMakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton, int32 BoneIndex, 
										int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting, bool bParentExcluded, int32 ParentExcludedBoneIndex);
	void MakeFakeBoneTransform(OUT FTransform& OutTransform, int32 ParentSimulateBoneIndex) const;

	void UpdateDeltaTime(float InDeltaTime, float InTimeDilation);
	void PrepareSimulation(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FTransform& ComponentTransform);
	void PrepareLocalCollisionConstraints(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FTransform& ComponentTransform);
	void ConvertPhysicsAssetToShape(OUT FLKAnimVerletCollisionShapeList& OutShapeList, const class UPhysicsAsset& InPhysicsAsset, const FBoneContainer* BoneContainerNullable) const;
	void SimulateVerlet(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform);
	void PreUpdateBones(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform);
	void SolveConstraints(float InDeltaTime);
	void UpdateSleep(float InDeltaTime);
	void PostUpdateBones(float InDeltaTime);
	void ApplyResult(OUT TArray<FBoneTransform>& OutBoneTransforms, const FBoneContainer& BoneContainer);
	void ClearSimulateBones();
	void ResetSimulation();

	template <typename Predicate>
	void ForEachConstraints(Predicate Pred);

public:
	const TArray<FLKAnimVerletBone>& GetSimulateBones() const { return SimulateBones; }
	const TArray<FLKAnimVerletBoneIndicatorPair>& GetSimulateBonePairIndicators() const { return SimulateBonePairIndicators; }
	const TArray<FLKAnimVerletConstraint_Distance>& GetDistanceConstraints() const { return DistanceConstraints; }
	const TArray<FLKAnimVerletConstraint_BallSocket>& GetBallSocketConstraints() const { return BallSocketConstraints; }
	const TArray<FLKAnimVerletConstraint_Pin>& GetPinConstraints() const { return PinConstraints; }
	const TArray<FLKAnimVerletConstraint_Sphere>& GetSphereCollisionConstraints() const { return SphereCollisionConstraints; }
	const TArray<FLKAnimVerletConstraint_Capsule>& GetCapsuleCollisionConstraints() const { return CapsuleCollisionConstraints; }
	const TArray<FLKAnimVerletConstraint_Box>& GetBoxCollisionConstraints() const { return BoxCollisionConstraints; }
	const TArray<FLKAnimVerletConstraint_Plane>& GetPlaneCollisionConstraints() const { return PlaneCollisionConstraints; }

	void SetDynamicCollisionShapes(const FLKAnimVerletCollisionShapeList& InDynamicCollisionShapes) { DynamicCollisionShapes = InDynamicCollisionShapes; }
	void ForceClearSimulateBones() { ClearSimulateBones(); }	/// for live editor preview

	void ResetCollisionShapes();
	void MarkLocalColliderDirty() { bLocalColliderDirty = true; }
	void CollisionShapesToCollisionShapeList(OUT FLKAnimVerletCollisionShapeList& OutShapeList) const;
	void CollisionShapesFromCollisionShapeList(const FLKAnimVerletCollisionShapeList& InShapeList);
	bool ConvertCollisionShapesToDataAsset();
	bool ConvertCollisionShapesFromDataAsset();
	bool ConvertPhysicsAssetToDataAsset();
	bool ConvertCollisionShapesFromPhysicsAsset();
	void SyncFromOtherAnimVerletNode(const FLKAnimNode_AnimVerlet& Other);

	void DebugDrawAnimVerlet(const FComponentSpacePoseContext& Output);

public:
	/** Input the starting bone of the cloth sequentially. */
	UPROPERTY(EditAnywhere, Category = "Setup", meta = (DisplayPriority = "1"))
	TArray<FLKAnimVerletBoneSetting> VerletBones;
	/** Insert a fake(virtual) bone in the middle of the chain.(It can affect the softness of the cloth or the collision reaction)  */
	UPROPERTY(EditAnywhere, Category = "Setup")
	bool bSubDivideBones = false;
	UPROPERTY(EditAnywhere, Category = "Setup", meta = (EditCondition = "bSubDivideBones", EditConditionHides, ClampMin = "0"))
	uint8 NumSubDividedBone = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (PinShownByDefault))
	bool bActivate = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	bool bSkipUpdateOnDedicatedServer = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (PinShownByDefault))
	bool bPause = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup", meta = (PinShownByDefault, ClampMin = "0.0"))
	float PlaySpeedRate = 1.0f;

	/** Adds an fake(virtual) bone to the end of the body.(may affect the rotation or collision of the end bone) */
	UPROPERTY(EditAnywhere, Category = "Setup", meta = (EditCondition = "bLockTipBone == false"))
	bool bMakeFakeTipBone = true;
	UPROPERTY(EditAnywhere, Category = "Setup", meta = (EditCondition = "bLockTipBone == false && bMakeFakeTipBone", ClampMin = "0.1", ForceUnits = "cm"))
	float FakeTipBoneLength = 10.0f;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Setup")
	bool bLockTipBone = false;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Setup", meta = (EditCondition = "bLockTipBone", ClampMin = "0.0", ForceUnits = "cm"))
	float TipBoneLockMargin = 0.0f;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Setup", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float StartBoneLockMargin = 0.0f;


	/** Magnitude of inertia due to animation pose change. */
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false", EditConditionHides, ClampMin = "0.0", ClampMax = "1.0"))
	float AnimationPoseDeltaInertia = 0.03f;
	/** Magnitude of inertia scale due to animation pose change. */
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false", EditConditionHides, ClampMin = "0.0"))
	float AnimationPoseDeltaInertiaScale = 1.0f;
	/** Limits the amount of inertia caused by animation pose changes. */
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false", EditConditionHides))
	bool bClampAnimationPoseDeltaInertia = true;
	UPROPERTY(EditAnywhere, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false && bClampAnimationPoseDeltaInertia", EditConditionHides, ClampMin = "0.0"))
	float AnimationPoseDeltaInertiaClampMax = 0.1f;
	/** Ignores animation poses and only applies cloth simulation results. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Settings")
	bool bIgnoreAnimationPose = false;
	/** The magnitude of inertia to the animation pose position.(Pulls the cloth simulation result into the animation pose position) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false", EditConditionHides, ClampMin = "0.0", ClampMax = "1.0"))
	float AnimationPoseInertia = 0.03f;
	/** Based on AnimationPoseInertiaTargetFrameRate, the AnimationPoseInertia value at the current FrameRate is adjusted. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false", EditConditionHides))
	bool bApplyAnimationPoseInertiaCorrection = true;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Settings", meta = (EditCondition = "bIgnoreAnimationPose == false && bApplyAnimationPoseInertiaCorrection", EditConditionHides, ClampMin = "0.0"))
	float AnimationPoseInertiaTargetFrameRate = 60.0f;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solve", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.9f;
	/** Based on DampingCorrectionTargetFrameRate, the Damping value at the current FrameRate is adjusted. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solve")
	bool bApplyDampingCorrection = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solve", meta = (EditCondition = "bApplyDampingCorrection", ClampMin = "0.0"))
	float DampingCorrectionTargetFrameRate = 60.0f;

	/** Use XPBD(Extended Position Based Dynamics) or PBD(Position Based Dynamics) solving constraints */
	UPROPERTY(EditAnywhere, Category = "Solve")
	bool bUseXPBDSolver = false;
	/** Compliance is the inverse of physical stiffness when use XPBD(Extended Position Based Dynamics). Unlike stiffness in PBD, compliance in XPBD has a direct correspondence to engineering stiffness, i.e.: Young's modulus. Most real-world materials have a Young's modulus of several GPa, and because compliance is simply inverse stiffness it must be correspondingly small. (Leather = 1.0 x 10^-8) */
	UPROPERTY(EditAnywhere, Category = "Solve", meta = (EditCondition = "bUseXPBDSolver", ClampMin = "0.0"))
	uint32 InvCompliance = 100000000;
	/** Stiffness applied to bone to bone distance when calculating verlet integration. */
	UPROPERTY(EditAnywhere, Category = "Solve", meta = (EditCondition = "bUseXPBDSolver == false", ClampMin = "0.0"))
	float Stiffness = 0.8f;

	/** Sleep Simulating bone when difference is small */
	UPROPERTY(EditAnywhere, Category = "Solve")
	bool bUseSleep = true;
	UPROPERTY(EditAnywhere, Category = "Solve", meta = (EditCondition = "bUseSleep"))
	bool bIgnoreSleepWhenParentWakedUp = true;
	UPROPERTY(EditAnywhere, Category = "Solve", meta = (EditCondition = "bUseSleep", ClampMin = "0.0", ForceUnits = "cm"))
	float SleepDeltaThreshold = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Solve", meta = (EditCondition = "bUseSleep", ClampMin = "0.0", ForceUnits = "s"))
	float SleepTriggerDuration = 5.0f;
	UPROPERTY(EditAnywhere, Category = "Solve", meta = (EditCondition = "bUseSleep", ClampMin = "0.0", ForceUnits = "cm"))
	float WakeUpDeltaThreshold = 0.1f;

	/** Adjust distance constraint to diagonal directions. (This is helpful when using the bIgnoreAnimationPose option) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bConstrainRightDiagonalDistance = false;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bConstrainLeftDiagonalDistance = false;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	/** Bending constraint for inextensible surfaces(for more realistic looking when bending but may cause performance impact - BETA) */
	bool bUseIsometricBendingConstraint = false;
	/** Stiffness for Isometric Bending Constraint(XPBD) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bUseIsometricBendingConstraint && bUseXPBDSolver", ClampMin = "0.0"))
	float BendingCompliance = 100000.0f;
	/** Stiffness for Isometric Bending Constraint(PBD). */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bUseIsometricBendingConstraint && bUseXPBDSolver == false", ClampMin = "0.0"))
	float BendingStiffness = 1.0f;

	/** The option to keep the distance between parent and child bones in the bone chain helps to keep SolveIteration small. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bPreserveLengthFromParent = true;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bPreserveLengthFromParent", ClampMin = "0.0", ForceUnits = "cm"))
	float LengthFromParentMargin = 0.1f;
	/** The option to keep the distance between side bones helps to keep SolveIteration small. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bPreserveSideLength = true;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bPreserveSideLength", ClampMin = "0.0", ForceUnits = "cm"))
	float SideLengthMargin = 0.1f;
	
	/** Stretch each bone by referencing it`s animation pose */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bStretchEachBone = false;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bStretchEachBone", ClampMin = "0.0"))
	float StretchStrength = 1.0f;

	/** Stretch bended bone without referencing it`s animation pose */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve")
	bool bStraightenBendedBone = false;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bStraightenBendedBone", ClampMin = "0.0", ClampMax = "1.0"))
	float StraightenBendedBoneStrength = 0.0003f;

	/** 
		It is the number of iterations to solve Verlet Integraion. 
		As there are more Collisions or Constraints, the larger the SolveIteration the more accurate the result. 
		SolveIteration of 1 can be fine thanks to the bPreserveLengthFromParent and bPreserveSideLength options.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "1"))
	int32 SolveIteration = 4;

	/** Use a fixed DeltaTime instead of real delta time if > 0. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "0.0", ForceUnits = "s"))
	float FixedDeltaTime = 0.0f;
	/** When using FixedDeltaTime, DeltaTime is corrected based on FixedDeltaTime in the current FrameRate. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve")
	bool bApplyDeltaTimeCorrection = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (EditCondition = "bApplyDeltaTimeCorrection", ClampMin = "0.0"))
	float DeltaTimeCorrectionTargetFrameRate = 60.0f;

	/** Limit delta time in situations where the frame rate fluctuates. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "0.0", ForceUnits = "s"))
	float MinDeltaTime = KINDA_SMALL_NUMBER;
	/** Limit delta time in situations where the frame rate fluctuates. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve", meta = (ClampMin = "0.0", ForceUnits = "s"))
	float MaxDeltaTime = 0.05f;

	/** Calculate forces via real verlet integration */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, AdvancedDisplay, Category = "Solve")
	bool bUseSquaredDeltaTime = false;

	/** if true, Use grand parent to parent bone`s direction to constrain global cone angle. otherwise use animation pose to constrain global cone angle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint", meta = (PinHiddenByDefault))
	bool bConstrainConeAngleFromParent = false;
	/** Global angle to use when constraining using a cone.(Ball - Socket joint constraints) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint", meta = (PinHiddenByDefault, ClampMin = "0.0", ClampMax = "90.0", ForceUnits = "deg"))
	float ConeAngle = 0.0f;

	/** The virtual thickness of the bone to be used in calculating various collisions and constraints.(radius) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Thickness = 0.3f;
	/** The Capsule shape between the 2 bones is used to calculate various collisions and constraints.(otherwise a sphere shape is used) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	bool bUseCapsuleCollisionForChain = true;

	/** Enable collision against to world.(May cause performance impact by physics sweep test for each bone and fake bone) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	FName WorldCollisionProfile = NAME_None;
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FBoneReference> WorldCollisionExcludeBones;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionSphere> SphereCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionCapsule> CapsuleCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionBox> BoxCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletCollisionPlane> PlaneCollisionShapes;
	/** For sharing collision data from pre made data */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	class ULKAnimVerletCollisionDataAsset* CollisionDataAsset = nullptr;
	/** For sharing collision data from pre made data (Can not modify in preview window like CollisionDataAsset) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinHiddenByDefault))
	class UPhysicsAsset* CollisionPhysicsAsset = nullptr;
	/** For sharing collision data from Animation Blueprint or external source code etc every frame */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (PinShownByDefault))
	FLKAnimVerletCollisionShapeList DynamicCollisionShapes;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity", meta = (PinHiddenByDefault, ForceUnits = "cm/s"))
	FVector Gravity = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity", meta = (PinHiddenByDefault))
	bool bGravityInWorldSpace = true;

	/** Adjust force to stretch the cloth from it`s parent */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forces", meta = (PinHiddenByDefault, ForceUnits = "cm/s"))
	float StretchForce = 0.0f;
	/** Adjust force to stretch the cloth by referencing positional relationship between the roots of each bone chain.(A type of side way gravity applied to stretch the cloth from side to side) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forces", meta = (PinHiddenByDefault, ForceUnits = "cm/s"))
	float SideStraightenForce = 0.0f;
	/** Adjust force to return to the original animation pose(different from below inertia factor) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forces", meta = (PinHiddenByDefault, ForceUnits = "cm/s"))
	float ShapeMemoryForce = 0.0f;
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault))
	TArray<FLKAnimVerletRandomForceSetting> AdditionalRandomWinds;

	/** Adjust UWindDirectionalSourceComponent in world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault))
	bool bAdjustWindComponent = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (PinHiddenByDefault, EditCondition = "bAdjustWindComponent"))
	float WindComponentScale = 1.0f;

	/** The scale to be applied to the inertia caused by the component's positional movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, ClampMin = "0.0"))
	float MoveInertiaScale = 1.0f;
	/** Based on MoveInertiaScaleTargetFrameRate, the MoveInertiaScale value at the current FrameRate is adjusted. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault))
	bool bApplyMoveInertiaScaleCorrection = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, EditCondition = "bApplyMoveInertiaScaleCorrection", ClampMin = "0.0"))
	float MoveInertiaScaleTargetFrameRate = 60.0f;
	/** Limits the amount of inertia caused by the component's positional movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault))
	bool bClampMoveInertia = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, EditCondition = "bClampMoveInertia", EditConditionHides, ClampMin = "0.0", ForceUnits = "cm"))
	float MoveInertiaClampMaxDistance = 300.0f;

	/** The scale to be applied to the inertia caused by the component's rotational movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, ClampMin = "0.0"))
	float RotationInertiaScale = 1.0f;
	/** Based on RotationInertiaScaleTargetFrameRate, the RotationInertiaScale value at the current FrameRate is adjusted. (It can help to obtain a consistent result regardless of the frame rate.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault))
	bool bApplyRotationInertiaScaleCorrection = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, EditCondition = "bApplyRotationInertiaScaleCorrection", ClampMin = "0.0"))
	float RotationInertiaScaleTargetFrameRate = 60.0f;
	/** Limits the amount of inertia caused by the component's rotational movement in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault))
	bool bClampRotationInertia = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Inertia", meta = (PinHiddenByDefault, EditCondition = "bClampRotationInertia", EditConditionHides, ForceUnits = "deg"))
	float RotationInertiaClampDegrees = 30.0f;

private:
	TArray<FLKAnimVerletBone> SimulateBones;								///Simulating bones(real bones + fake virtual bones)
	TArray<FLKAnimVerletExcludedBone> ExcludedBones;						///Excluded bones in Simulating bone chain(real bones)
	TArray<FLKAnimVerletBoneIndicator> RelevantBoneIndicators;				///Simulating real bones + Excluded real bones + fake tip bone(for bone`s rotation at PostUpdate phase)
	TArray<FLKAnimVerletBoneIndicatorPair> SimulateBonePairIndicators;		///Simulating bone`s each distance constraints pair(for capsule collision) nearly same as DistanceConstraint
	FLKAnimVerletCollisionShapeList SimulatingCollisionShapes;
	///TArray<FLKAnimVerletConstraint*> Constraints;
	/// Unroll each constratins for better solve result(considering constraint`s solving order)
	TArray<FLKAnimVerletConstraint_Pin> PinConstraints;
	TArray<FLKAnimVerletConstraint_Distance> DistanceConstraints;
	TArray<FLKAnimVerletConstraint_IsometricBending> BendingConstraints;
	TArray<FLKAnimVerletConstraint_Straighten> StraightenConstraints;
	TArray<FLKAnimVerletConstraint_FixedDistance> FixedDistanceConstraints;
	TArray<FLKAnimVerletConstraint_BallSocket> BallSocketConstraints;
	TArray<FLKAnimVerletConstraint_Sphere> SphereCollisionConstraints;
	TArray<FLKAnimVerletConstraint_Capsule> CapsuleCollisionConstraints;
	TArray<FLKAnimVerletConstraint_Box> BoxCollisionConstraints;
	TArray<FLKAnimVerletConstraint_Plane> PlaneCollisionConstraints;
	TArray<FLKAnimVerletConstraint_World> WorldCollisionConstraints;
	TArray<TArray<int32>> BoneChainIndexes;								///Simulating bone`s index list per single chain
	int32 MaxBoneChainLength = 0;

private:
	bool bLocalColliderDirty = false;
	float DeltaTime = 0.0f;
	FTransform PrevComponentT = FTransform::Identity;
};