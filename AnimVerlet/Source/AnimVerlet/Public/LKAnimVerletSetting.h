#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletSetting.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct FLKAnimVerletBoneUnitSetting
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere)
	FBoneReference Bone;

	/** Overrides subdivision for segments from this bone to each simulated child. On a terminal bone, only this override can subdivide the automatic fake tip segment. */
	UPROPERTY(EditAnywhere, Category = "SubDivide")
	bool bOverrideSubDivideBones = false;
	UPROPERTY(EditAnywhere, Category = "SubDivide", meta = (EditCondition = "bOverrideSubDivideBones", EditConditionHides))
	bool bSubDivideBones = false;
	UPROPERTY(EditAnywhere, Category = "SubDivide", meta = (EditCondition = "bOverrideSubDivideBones && bSubDivideBones", EditConditionHides, ClampMin = "0"))
	uint8 NumSubDividedBone = 1;

	UPROPERTY(EditAnywhere, Category = "Constraint")
	bool bLockBone = false;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (EditCondition = "bLockBone", ClampMin = "0.0", ForceUnits = "cm"))
	float LockMargin = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Constraint")
	bool bOverrideConstrainConeAngleFromParent = false;
	/** if true, Use grand parent to parent bone`s direction to constrain each bone`s cone angle. otherwise use animation pose to constrain each bone`s cone angle. (override global cone angle) */
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (EditCondition = "bOverrideConstrainConeAngleFromParent"))
	bool bConstrainConeAngleFromParent = false;
	UPROPERTY(EditAnywhere, Category = "Constraint")
	bool bOverrideConeAngle = false;
	/** each bones`s angle to use when constraining using a cone.(Ball - Socket joint constraints, override global cone angle) */
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (EditCondition = "bOverrideConeAngle", ClampMin = "0.0", ClampMax = "90.0", ForceUnits = "deg"))
	float ConeAngle = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint")
	bool bOverrideConeAngleOffset = false;
	/** Local rotation offset applied to this bone's cone center direction. */
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (EditCondition = "bOverrideConeAngleOffset"))
	FRotator ConeAngleOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Collision")
	bool bOverrideThickness = false;
	/** The virtual thickness of the bone to be used in calculating various collisions and constraints.(radius) */
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (EditCondition = "bOverrideThickness", ClampMin = "0.0", ForceUnits = "cm"))
	float Thickness = 0.3f;
	/** For each bone unit, override the bUseCapsuleCollisionForChain option of the AnimVerlet node.(This option has no effect if bUseCapsuleCollisionForChain is unchecked.) */
	UPROPERTY(EditAnywhere, Category = "Collision")
	bool bOverrideToUseSphereCollisionForChain = false;

	UPROPERTY(EditAnywhere, Category = "Physics")
	bool bOverrideMass = false;
	UPROPERTY(EditAnywhere, Category = "Physics", meta = (EditCondition = "bOverrideMass", ClampMin = "0.01"))
	float Mass = 1.0f;

public:
	bool operator==(const FBoneReference& RHS) const { return Bone == RHS; }
};


USTRUCT(BlueprintInternalUseOnly)
struct FLKAnimVerletBoneSetting
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "BoneChain")
	FBoneReference RootBone;

	UPROPERTY(EditAnywhere, Category = "BoneChain")
	TArray<FBoneReference> ExcludeBones;

	/** Each VerletBone`s setting override(optional) */
	UPROPERTY(EditAnywhere, Category = "BoneChain")
	TArray<FLKAnimVerletBoneUnitSetting> BoneUnitSettingOverride;

	/** Stretch excluded bones by referencing their simulated parent and child bone. */
	UPROPERTY(EditAnywhere, Category = "BoneChain")
	bool bStraightenExcludedBonesByParent = true;

	/** Based on the RootBone, it helps the simulation result by creating a virtual bone at the Offset location. */
	UPROPERTY(EditAnywhere, Category = "BoneChain")
	bool bFakeBone = false;

	UPROPERTY(EditAnywhere, Category = "BoneChain", meta = (EditCondition = "bFakeBone", EditConditionHides))
	FVector FakeBoneOffsetDir = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, Category = "BoneChain", meta = (EditCondition = "bFakeBone", EditConditionHides, ClampMin = "0.1", ForceUnits = "cm"))
	float FakeBoneOffsetSize = 3.0f;

	UPROPERTY(EditAnywhere, Category = "BoneChain", meta = (ClampMin = "0.01"))
	float Mass = 1.0f;
};

/**
 * Manually adds a distance constraint between two skeletal bones.
 * - Bones already managed by AnimVerlet use their simulated particles.
 * - A bone outside the configured Verlet bone chains is treated as a pinned animation pose anchor, so it can safely constrain a simulated bone without being overwritten by the simulation.
 */
USTRUCT(BlueprintInternalUseOnly)
struct FLKAnimVerletCustomDistanceConstraintSetting
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Constraint")
	FBoneReference BoneA;

	UPROPERTY(EditAnywhere, Category = "Constraint")
	FBoneReference BoneB;

	/** Minimum allowed distance between BoneA and BoneB. */
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float MinDistance = 0.0f;

	/**
	 * Maximum allowed distance between BoneA and BoneB.
	 * When both values are zero, the animation pose distance is used for both
	 * values to preserve the previous exact-distance behavior.
	 */
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float MaxDistance = 0.0f;
};

USTRUCT(BlueprintInternalUseOnly)
struct FLKAnimVerletRandomForceSetting
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (PinHiddenByDefault))
	FVector RandomForceDirection = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (PinHiddenByDefault, ClampMin = "0.0", ForceUnits = "cm/s"))
	float RandomForceSizeMin = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (PinHiddenByDefault, ClampMin = "0.0", ForceUnits = "cm/s"))
	float RandomForceSizeMax = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (PinHiddenByDefault))
	bool bRandomForceDirectionInWorldSpace = true;
};

struct FLKAnimVerletUpdateParam
{
	bool bUseSquaredDeltaTime = false;
	float Damping = 0.0f;
	FVector ComponentMoveDiff = FVector::ZeroVector;
	FQuat ComponentRotDiff = FQuat::Identity;
	float StretchForce = 0.0f;
	bool bAlignStretchForceToGravity = false;
	float SideStraightenForce = 0.0f;
	float ShapeMemoryForce = 0.0f;
	bool bAlignShapeMemoryForceToGravity = false;
	FVector Gravity = FVector::ZeroVector;
	FVector ExternalForce = FVector::ZeroVector;
	FLKAnimVerletRandomForceSetting RandomWind;
	TArray<FLKAnimVerletRandomForceSetting, TInlineAllocator<8>> AdditionalRandomWinds;
};
