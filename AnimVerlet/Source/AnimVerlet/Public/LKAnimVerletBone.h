#pragma once
#include <CoreMinimal.h>
#include <Animation/BoneReference.h>
#include "LKAnimVerletBound.h"

///=========================================================================================================================================
/// FLKAnimVerletBoneBase
///=========================================================================================================================================
struct FLKAnimVerletBoneBase
{
public:
	FBoneReference BoneReference;

	FVector PoseLocation = FVector::ZeroVector;
	FVector Location = FVector::ZeroVector;

	FQuat PoseRotation = FQuat::Identity;
	FQuat Rotation = FQuat::Identity;

	FVector PoseScale = FVector::ZeroVector;

public:
	FLKAnimVerletBoneBase() = default;
	FLKAnimVerletBoneBase(const FBoneReference& InBoneReference)
		: BoneReference(InBoneReference)
	{
	}

	virtual ~FLKAnimVerletBoneBase() {}
};

///=========================================================================================================================================
/// FLKAnimVerletBone
///=========================================================================================================================================
struct FLKAnimVerletBone : public FLKAnimVerletBoneBase
{
public:
	int32 ParentVerletBoneIndex = INDEX_NONE;
	TSet<int32, DefaultKeyFuncs<int32>, TInlineSetAllocator<8>> ChildVerletBoneIndexes;
	bool bTipBone = false;
	bool bFakeBone = false;
	bool bUseXPBDSolver = false;
	float InvMass = 1.0f;
	FVector FakeBoneLocationOffset = FVector::ZeroVector;
	FVector SideStraightenDirInLocal = FVector::ZeroVector;

	FVector PrevPoseLocation = FVector::ZeroVector;
	FVector PrevLocation = FVector::ZeroVector;

	FQuat PrevPoseRotation = FQuat::Identity;
	FQuat PrevRotation = FQuat::Identity;

	FVector PoseDirFromParent = FVector::ZeroVector;

	FVector MoveDelta = FVector::ZeroVector;
	FVector Velocity = FVector::ZeroVector;		///for XPBD

	bool bSleep = false;
	float SleepTriggerElapsedTime = 0.0f;

	bool bPinned = false;
	float PinMargin = 0.0f;
	bool bConstrainConeAngleFromParent = false;
	float ConeAngleConstraint = 0.0f;
	float Thickness = 0.0f;
	bool bOverrideToUseSphereCollisionForChain = false;

public:
	bool IsFakeBone() const { return bFakeBone; }
	bool HasBoneSetup() const { return BoneReference.HasValidSetup(); }
	bool HasParentBone() const { return ParentVerletBoneIndex != INDEX_NONE; }
	bool IsTipBone() const { return bTipBone; }
	bool IsSleep() const { return bSleep; }
	bool IsPinned() const { return bPinned; }
	FTransform MakeCurrentTransform() const { return FTransform(Rotation, Location, PoseScale); }
	FTransform MakePoseTransform() const { return FTransform(PoseRotation, PoseLocation, PoseScale); }
	FVector MakeFakeBonePoseLocation(const FTransform& PoseT) const;
	FTransform MakeFakeBonePoseTransform(const FTransform& PoseT) const;

	inline static FLKAnimVerletBound MakeBound(const FVector& InLocation, float InThickness) { return FLKAnimVerletBound::MakeBoundFromCenterHalfExtents(InLocation, FVector(InThickness, InThickness, InThickness)); }
	inline FLKAnimVerletBound MakeBound() const { return MakeBound(Location, Thickness); }

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
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletExcludedBone
///=========================================================================================================================================
struct FLKAnimVerletExcludedBone : public FLKAnimVerletBoneBase
{
public:
	int32 ParentVerletBoneIndex = INDEX_NONE;
	int32 ParentExcludedBoneIndex = INDEX_NONE;

	float LengthToParent = 0.0f;
	bool bStraightenExcludedBonesByParent = true;

public:
	FLKAnimVerletExcludedBone() = default;
	FLKAnimVerletExcludedBone(const FBoneReference& InBoneReference, int32 InParentVerletBoneIndex, int32 InParentExcludedBoneIndex)
		: FLKAnimVerletBoneBase(InBoneReference), ParentVerletBoneIndex(InParentVerletBoneIndex), ParentExcludedBoneIndex(InParentExcludedBoneIndex)
	{
	}

	bool HasBoneSetup() const { return BoneReference.HasValidSetup(); }
	bool HasParentBone() const { return HasVerletParentBone() || HasExcludedParentBone(); }
	bool HasVerletParentBone() const { return ParentVerletBoneIndex != INDEX_NONE; }
	bool HasExcludedParentBone() const { return ParentExcludedBoneIndex != INDEX_NONE; }

public:
	void PrepareSimulation(const FTransform& PoseT);
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletBoneIndicator
///=========================================================================================================================================
struct FLKAnimVerletBoneIndicator
{
public:
	int32 AnimVerletBoneIndex = INDEX_NONE;
	bool bExcludedBone = false;

	int32 ParentAnimVerletBoneIndex = INDEX_NONE;
	bool bParentExcludedBone = false;

public:
	FLKAnimVerletBoneIndicator() = default;
	FLKAnimVerletBoneIndicator(int32 InAnimVerletBoneIndex, bool bInExcludedBone)
		: AnimVerletBoneIndex(InAnimVerletBoneIndex), bExcludedBone(bInExcludedBone)
	{
	}
	FLKAnimVerletBoneIndicator(int32 InAnimVerletBoneIndex, bool bInExcludedBone, int32 InParentAnimVerletBoneIndex, bool bInParentExcludedBone)
		: AnimVerletBoneIndex(InAnimVerletBoneIndex), bExcludedBone(bInExcludedBone), ParentAnimVerletBoneIndex(InParentAnimVerletBoneIndex), bParentExcludedBone(bInParentExcludedBone)
	{
	}

	bool IsValidBoneIndicator() const { return AnimVerletBoneIndex != INDEX_NONE; }
	bool HasParentSimulateBone() const { return bParentExcludedBone == false && ParentAnimVerletBoneIndex != INDEX_NONE; }
	bool HasParentExcludedBone() const { return bParentExcludedBone && ParentAnimVerletBoneIndex != INDEX_NONE; }
	
	inline bool operator==(const FLKAnimVerletBoneIndicator& RHS) const { return (AnimVerletBoneIndex == RHS.AnimVerletBoneIndex && bExcludedBone == RHS.bExcludedBone && 
																				  ParentAnimVerletBoneIndex == RHS.ParentAnimVerletBoneIndex && bParentExcludedBone == RHS.bParentExcludedBone); }
	inline friend uint32 GetTypeHash(const FLKAnimVerletBoneIndicator& Indicator) { return GetTypeHash(TTuple<int32, bool, int32, bool>(Indicator.AnimVerletBoneIndex, Indicator.bExcludedBone, Indicator.ParentAnimVerletBoneIndex, Indicator.bParentExcludedBone)); }
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletBoneIndicatorPair
///=========================================================================================================================================
struct FLKAnimVerletBoneIndicatorPair
{
public:
	FLKAnimVerletBoneIndicator BoneA;
	FLKAnimVerletBoneIndicator BoneB;

public:
	FLKAnimVerletBoneIndicatorPair() = default;
	FLKAnimVerletBoneIndicatorPair(const FLKAnimVerletBoneIndicator& InBoneA, const FLKAnimVerletBoneIndicator& InBoneB)
		: BoneA(InBoneA), BoneB(InBoneB)
	{
	}

	inline bool operator==(const FLKAnimVerletBoneIndicatorPair& RHS) const { return (BoneA == RHS.BoneA && BoneB == RHS.BoneB); }
	inline friend uint32 GetTypeHash(const FLKAnimVerletBoneIndicatorPair& InPair) { return GetTypeHash(TPair<FLKAnimVerletBoneIndicator, FLKAnimVerletBoneIndicator>(InPair.BoneA, InPair.BoneB)); }
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletBoneKey
///=========================================================================================================================================
struct FLKAnimVerletBoneKey
{
public:
	FLKAnimVerletBoneKey(const FBoneReference& InBoneReference) : BoneReference(InBoneReference) {}

	inline bool operator==(const FLKAnimVerletBoneKey& RHS) const { return (BoneReference == RHS.BoneReference); }
	inline bool operator==(const FLKAnimVerletBone& RHS) const { return (BoneReference == RHS.BoneReference); }
	inline bool operator==(const FLKAnimVerletExcludedBone& RHS) const { return (BoneReference == RHS.BoneReference); }

public:
	inline friend bool operator==(const FLKAnimVerletBoneKey& LHS, const FLKAnimVerletBone& RHS) { return (LHS.BoneReference == RHS.BoneReference); }
	inline friend bool operator==(const FLKAnimVerletBoneKey& LHS, const FLKAnimVerletExcludedBone& RHS) { return (LHS.BoneReference == RHS.BoneReference); }
	inline friend bool operator==(const FLKAnimVerletBone& LHS, const FLKAnimVerletBoneKey& RHS) { return (LHS.BoneReference == RHS.BoneReference); }
	inline friend bool operator==(const FLKAnimVerletExcludedBone& LHS, const FLKAnimVerletBoneKey& RHS) { return (LHS.BoneReference == RHS.BoneReference); }

public:
	const FBoneReference& BoneReference;
};
///=========================================================================================================================================