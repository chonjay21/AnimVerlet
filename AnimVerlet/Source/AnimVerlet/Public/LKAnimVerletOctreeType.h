#pragma once
#include <CoreMinimal.h>
#include <Math/GenericOctreePublic.h>
#include <Math/GenericOctree.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletBound.h"
#include "LKAnimVerletBroadphaseType.h"


///=========================================================================================================================================
/// TOctree2 Helper
///=========================================================================================================================================
struct ANIMVERLET_API FLKOctreeSemantics
{
	enum { MaxElementsPerLeaf = 16 };
	enum { MinInclusiveElementsPerNode = 7 };
	enum { MaxNodeDepth = 12 };

	using FOctree = TOctree2<FLKBroadphaseElement, FLKOctreeSemantics>;
	using ElementAllocator = TInlineAllocator<MaxElementsPerLeaf>;

	FORCEINLINE static FBoxCenterAndExtent GetBoundingBox(const FLKBroadphaseElement& Element)
	{
		return FBoxCenterAndExtent(Element.CachedBound.GetCenter(), Element.CachedBound.GetHalfExtents());
	}

	FORCEINLINE static bool AreElementsEqual(const FLKBroadphaseElement& A, const FLKBroadphaseElement& B)
	{
		return (A.BoneIndicatorPair == B.BoneIndicatorPair);
	}

	FORCEINLINE static void ApplyOffset(FLKBroadphaseElement& Element, const FVector& InOffset)
	{
		verify(TEXT("Not implemented yet"));
	}

	static void SetElementId(TOctree2<FLKBroadphaseElement, FLKOctreeSemantics>& OctreeOwner, const FLKBroadphaseElement& Element, FOctreeElementId2 Id);
};