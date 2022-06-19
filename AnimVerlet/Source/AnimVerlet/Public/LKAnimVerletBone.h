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
	bool bFakeBone = false;
	FVector FakeBoneLocationOffset = FVector::ZeroVector;

	FVector PrevPoseLocation = FVector::ZeroVector;
	FVector PoseLocation = FVector::ZeroVector;
	FVector Location = FVector::ZeroVector;
	FVector PrevLocation = FVector::ZeroVector;

	FQuat PrevPoseRotation = FQuat::Identity;
	FQuat PoseRotation = FQuat::Identity;
	FQuat Rotation = FQuat::Identity;
	FQuat PrevRotation = FQuat::Identity;

	FVector PoseScale = FVector::ZeroVector;

	FVector MoveDelta = FVector::ZeroVector;

public:
	bool IsFakeBone() const { return bFakeBone; }
	bool HasBoneSetup() const { return BoneReference.HasValidSetup(); }
	bool HasParentBone() const { return ParentVerletBoneIndex != INDEX_NONE; }
	FTransform MakeCurrentTransform() const { return FTransform(Rotation, Location, PoseScale); }
	FTransform MakePoseTransform() const { return FTransform(PoseRotation, PoseLocation, PoseScale); }
	FVector MakeFakeBonePoseLocation(const FTransform& PoseT) const;
	FTransform MakeFakeBonePoseTransform(const FTransform& PoseT) const;

public:
	void InitializeTransform(const FTransform& InitialT);
	void SetFakeBoneOffset(const FVector& InLocationOffset);
	void PrepareSimulation(const FTransform& PoseT);
	void Update(float DeltaTime, const struct FLKAnimVerletUpdateParam& InParam);
	void AdjustPoseTransform(float DeltaTime, const FVector& ParentLocation, const FVector& ParentPoseLocation,
							 float ReferencePoseInertia, float ReferencePoseDeltaInertia, bool bClampReferencePoseDeltaInertia, float ReferencePoseDeltaInertiaClampMax);
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