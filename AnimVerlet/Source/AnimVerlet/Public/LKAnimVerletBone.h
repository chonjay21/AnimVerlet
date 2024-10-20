#pragma once
#include <CoreMinimal.h>

///=========================================================================================================================================
/// FLKAnimVerletBone
///=========================================================================================================================================
struct FLKAnimVerletBone
{
public:
	FBoneReference BoneReference;
	int32 ParentVerletBoneIndex = INDEX_NONE;
	bool bTipBone = false;
	bool bFakeBone = false;
	bool bUseXPBDSolver = false;
	float InvMass = 1.0f;
	FVector FakeBoneLocationOffset = FVector::ZeroVector;
	FVector SideStraightenDirInLocal = FVector::ZeroVector;

	FVector PrevPoseLocation = FVector::ZeroVector;
	FVector PoseLocation = FVector::ZeroVector;
	FVector Location = FVector::ZeroVector;
	FVector PrevLocation = FVector::ZeroVector;

	FQuat PrevPoseRotation = FQuat::Identity;
	FQuat PoseRotation = FQuat::Identity;
	FQuat Rotation = FQuat::Identity;
	FQuat PrevRotation = FQuat::Identity;

	FVector PoseDirFromParent = FVector::ZeroVector;
	FVector PoseScale = FVector::ZeroVector;

	FVector MoveDelta = FVector::ZeroVector;
	FVector Velocity = FVector::ZeroVector;		///for XPBD

	bool bSleep = false;
	float SleepTriggerElapsedTime = 0.0f;

public:
	bool IsFakeBone() const { return bFakeBone; }
	bool HasBoneSetup() const { return BoneReference.HasValidSetup(); }
	bool HasParentBone() const { return ParentVerletBoneIndex != INDEX_NONE; }
	bool IsTipBone() const { return bTipBone; }
	bool IsSleep() const { return bSleep; }
	FTransform MakeCurrentTransform() const { return FTransform(Rotation, Location, PoseScale); }
	FTransform MakePoseTransform() const { return FTransform(PoseRotation, PoseLocation, PoseScale); }
	FVector MakeFakeBonePoseLocation(const FTransform& PoseT) const;
	FTransform MakeFakeBonePoseTransform(const FTransform& PoseT) const;

public:
	void InitializeTransform(const FTransform& InitialT);
	void SetFakeBoneOffset(const FVector& InLocationOffset);
	void SetSideStraightenDirInLocal(const FVector& InDir) { SideStraightenDirInLocal = InDir; }
	void PrepareSimulation(const FTransform& PoseT, const FVector& InPoseDirFromParent);
	void Update(float DeltaTime, const struct FLKAnimVerletUpdateParam& InParam);
	void PostUpdate(float DeltaTime);
	void AdjustPoseTransform(float DeltaTime, const FVector& ParentLocation, const FVector& ParentPoseLocation,
							 float AnimationPoseInertia, float AnimationPoseDeltaInertia, bool bClampAnimationPoseDeltaInertia, float AnimationPoseDeltaInertiaClampMax);

	void Sleep();
	void WakeUp();
	void ResetSimulation();
};

struct FLKAnimVerletBoneKey
{
public:
	FLKAnimVerletBoneKey(const FBoneReference& InBoneReference) : BoneReference(InBoneReference) {}

	inline bool operator==(const FLKAnimVerletBoneKey& RHS) const { return (BoneReference == RHS.BoneReference); }
	inline bool operator==(const FLKAnimVerletBone& RHS) const { return (BoneReference == RHS.BoneReference); }

public:
	const FBoneReference& BoneReference;
};
inline bool operator==(const FLKAnimVerletBoneKey& LHS, const FLKAnimVerletBone& RHS) { return (LHS.BoneReference == RHS.BoneReference); }
inline bool operator==(const FLKAnimVerletBone& LHS, const FLKAnimVerletBoneKey& RHS) { return (LHS.BoneReference == RHS.BoneReference); }
///=========================================================================================================================================