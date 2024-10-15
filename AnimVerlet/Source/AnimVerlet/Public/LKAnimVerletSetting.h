#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletSetting.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct FLKAnimVerletBoneSetting
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "BoneChain")
	FBoneReference RootBone;

	UPROPERTY(EditAnywhere, Category = "BoneChain")
	TArray<FBoneReference> ExcludeBones;

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
	float ShapeMemoryForce = 0.0f;
	FVector Gravity = FVector::ZeroVector;
	FVector ExternalForce = FVector::ZeroVector;
	FVector RandomWindDir = FVector::ZeroVector;
	float RandomWindSizeMin = 0.0f;
	float RandomWindSizeMax = 0.0f;
};
