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

struct FLKAnimVerletUpdateParam
{
	bool bUseSquaredDeltaTime = false;
	float Damping = 0.0f;
	FVector ComponentMoveDiff = FVector::ZeroVector;
	FQuat ComponentRotDiff = FQuat::Identity;
	float StretchForce = 0.0f;
	float SideStraightenForce = 0.0f;
	float ShapeMemoryForce = 0.0f;
	FVector Gravity = FVector::ZeroVector;
	FVector ExternalForce = FVector::ZeroVector;
	FVector RandomWindDir = FVector::ZeroVector;
	float RandomWindSizeMin = 0.0f;
	float RandomWindSizeMax = 0.0f;
};
