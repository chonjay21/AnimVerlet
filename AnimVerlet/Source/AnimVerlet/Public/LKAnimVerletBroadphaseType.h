#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletBound.h"

#define LK_BROADPHASE_USE_OCTREE	(1)

///=========================================================================================================================================
/// FLKBroadphaseElement
///=========================================================================================================================================
struct FLKBroadphaseElement
{
public:
	FLKAnimVerletBoneIndicatorPair BoneIndicatorPair;
	FLKAnimVerletBound CachedBound;

public:
	FLKBroadphaseElement() = default;
	FLKBroadphaseElement(const FLKAnimVerletBone& InBone, int32 InSimulateBoneIndex, const FLKAnimVerletBound& InBound)
		: BoneIndicatorPair(FLKAnimVerletBoneIndicator(INDEX_NONE, false), FLKAnimVerletBoneIndicator(InSimulateBoneIndex, false))
		, CachedBound(InBound)
	{}
	FLKBroadphaseElement(const FLKAnimVerletBoneIndicatorPair& InBoneIndicatorPair, const FLKAnimVerletBound& InBound)
		: BoneIndicatorPair(InBoneIndicatorPair)
		, CachedBound(InBound)
	{}

	inline bool operator==(const FLKBroadphaseElement& RHS) const { return BoneIndicatorPair == RHS.BoneIndicatorPair; }
	inline bool operator==(const FLKAnimVerletBoneIndicatorPair& RHS) const { return BoneIndicatorPair == RHS; }
};


///=========================================================================================================================================
/// LKAnimVerletBroadphaseContainer search functor
///=========================================================================================================================================
struct FLKBroadphaseElementFinder
{
public:
	TArray<FLKAnimVerletBoneIndicatorPair>* BoneIndicatorSpace = nullptr;

public:
	FLKBroadphaseElementFinder(TArray<FLKAnimVerletBoneIndicatorPair>* InBoneIndicatorPairSpace) : BoneIndicatorSpace(InBoneIndicatorPairSpace) {}

	void operator()(const FLKBroadphaseElement& InElement) const
	{
		BoneIndicatorSpace->Emplace(InElement.BoneIndicatorPair);
	}
};