#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBroadphaseType.h"

#if LK_BROADPHASE_USE_OCTREE
#include "LKAnimVerletOctree.h"
#else
#include "LKAnimVerletSpatialHashMap.h"
#endif


/////=========================================================================================================================================
///// LKAnimVerletBroadphaseContainer
/////=========================================================================================================================================
class ANIMVERLET_API LKAnimVerletBroadphaseContainer
{
public:
	LKAnimVerletBroadphaseContainer() = default;

	void Initialize(const FVector& InOrigin, const FVector& InHalfExtents, const FVector& InCellHalfExtents) { Container.Initialize(InOrigin, InHalfExtents, InCellHalfExtents); }
	void Destroy() { Container.Destroy(); }
	void Reset() { Container.Reset(); }

	inline bool IsInitialized() const { return Container.IsInitialized(); }
	inline int32 GetNumNodes() const { return Container.GetNumNodes(); }
	inline FLKAnimVerletBound GetRootBounds() const { return Container.GetRootBounds(); }

	inline void FindElements(const FLKAnimVerletBound& BoxBounds, const FLKBroadphaseElementFinder& Finder) const { Container.FindElements(BoxBounds, Finder); }
	inline void AddElement(const FLKBroadphaseElement& Element) { Container.AddElement(Element); }
	inline bool UpdateElement(const FLKAnimVerletBoneIndicatorPair& InIndicator, const FLKAnimVerletBound& InNewBound) { return Container.UpdateElement(InIndicator, InNewBound); }

	/** Writes stats for the octree to the log. */
	inline void DumpStats() const { Container.DumpStats(); }

private:
#if LK_BROADPHASE_USE_OCTREE
	LKAnimVerletOctree Container;
#else
	LKAnimVerletSpatialHashMap Container;
#endif
};