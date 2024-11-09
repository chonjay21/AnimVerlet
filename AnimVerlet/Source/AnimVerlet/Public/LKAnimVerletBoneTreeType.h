#pragma once
#include <CoreMinimal.h>
#include <Math/GenericOctreePublic.h>
#include <Math/GenericOctree.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletBound.h"

///=========================================================================================================================================
/// LKOctreeElement
///=========================================================================================================================================
struct FLKOctreeElement
{
public:
	FLKAnimVerletBoneIndicatorPair BoneIndicatorPair;
	FLKAnimVerletBound CachedBound;

public:
	FLKOctreeElement() = default;
	FLKOctreeElement(const FLKAnimVerletBone& InBone, int32 InSimulateBoneIndex, const FLKAnimVerletBound& InBound)
		: BoneIndicatorPair(FLKAnimVerletBoneIndicator(INDEX_NONE, false), FLKAnimVerletBoneIndicator(InSimulateBoneIndex, false))
		, CachedBound(InBound)
	{}
	FLKOctreeElement(const FLKAnimVerletBoneIndicatorPair& InBoneIndicatorPair, const FLKAnimVerletBound& InBound)
		: BoneIndicatorPair(InBoneIndicatorPair)
		, CachedBound(InBound)
	{}

	inline bool operator==(const FLKOctreeElement& RHS) const { return BoneIndicatorPair == RHS.BoneIndicatorPair; }
};

///=========================================================================================================================================
/// LKOctree search functor
///=========================================================================================================================================
struct FLKOctreeElementFinder
{
public:
	TArray<FLKAnimVerletBoneIndicatorPair>* BoneIndicatorSpace = nullptr;

public:
	FLKOctreeElementFinder(TArray<FLKAnimVerletBoneIndicatorPair>* InBoneIndicatorPairSpace) : BoneIndicatorSpace(InBoneIndicatorPairSpace) {}

	void operator()(const FLKOctreeElement& InElement) const
	{
		BoneIndicatorSpace->Emplace(InElement.BoneIndicatorPair);
	}
};

///=========================================================================================================================================
/// TOctree2 Helper
///=========================================================================================================================================
struct ANIMVERLET_API FLKOctreeSemantics
{
	enum { MaxElementsPerLeaf = 16 };
	enum { MinInclusiveElementsPerNode = 7 };
	enum { MaxNodeDepth = 12 };

	using FOctree = TOctree2<FLKOctreeElement, FLKOctreeSemantics>;
	using ElementAllocator = TInlineAllocator<MaxElementsPerLeaf>;

	FORCEINLINE static FBoxCenterAndExtent GetBoundingBox(const FLKOctreeElement& Element)
	{
		return FBoxCenterAndExtent(Element.CachedBound.GetCenter(), Element.CachedBound.GetHalfExtents());
	}

	FORCEINLINE static bool AreElementsEqual(const FLKOctreeElement& A, const FLKOctreeElement& B)
	{
		return (A.BoneIndicatorPair == B.BoneIndicatorPair);
	}

	FORCEINLINE static void ApplyOffset(FLKOctreeElement& Element, const FVector& InOffset)
	{
		verify(TEXT("Not implemented yet"));
	}

	static void SetElementId(TOctree2<FLKOctreeElement, FLKOctreeSemantics>& OctreeOwner, const FLKOctreeElement& Element, FOctreeElementId2 Id);
};