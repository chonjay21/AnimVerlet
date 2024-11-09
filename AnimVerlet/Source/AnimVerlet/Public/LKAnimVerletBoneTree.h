#pragma once
#include <CoreMinimal.h>
#include <Math/GenericOctreePublic.h>
#include <Math/GenericOctree.h>
#include "LKAnimVerletBoneTreeType.h"

///=========================================================================================================================================
/// TOctree2 Wrapper
///=========================================================================================================================================
class ANIMVERLET_API LKVerletBoneOctree : public TOctree2<FLKOctreeElement, FLKOctreeSemantics>
{
public:
	/// Use parent`s constructor
	using TOctree2::TOctree2;
	
	void SetElementIdImpl(const FLKOctreeElement& InElement, FOctreeElementId2 Id);
	const FOctreeElementId2* FindElementID(const FLKAnimVerletBoneIndicatorPair& InKey) const { return ObjectToOctreeID.Find(InKey); }

private:
	TMap<FLKAnimVerletBoneIndicatorPair, FOctreeElementId2> ObjectToOctreeID;
};


///=========================================================================================================================================
/// TOctree2 Wrapper
///=========================================================================================================================================
class ANIMVERLET_API LKOctree
{
public:
	LKOctree() = default;

	void InitOctree(const FVector& InOrigin, float InExtents)
	{
		verify(OctreeData == nullptr);
		OctreeData = new LKVerletBoneOctree(InOrigin, InExtents);
	}

	void DestroyOctree() 
	{ 
		if (OctreeData != nullptr)
		{
			ResetOctree();
			delete OctreeData;
		}
		OctreeData = nullptr; 
	}

	void ResetOctree()
	{
		verify(OctreeData != nullptr);
		OctreeData->Destroy();
	}

	inline int32 GetNumNodes() const { verify(OctreeData != nullptr); return OctreeData->GetNumNodes(); }

	/**
	 * this function will traverse the Octree using a fast box-box intersection this should be the preferred way of traversing the tree.
	 * @param BoxBounds - the bounds to test if a node is traversed or skipped.
	 * @param Func - Function to call with each Element for nodes that passed bounds test.
	 */
	template <typename IterateBoundsFunc>
	inline void FindElementsWithBoundsTest(const FBoxCenterAndExtent& BoxBounds, const IterateBoundsFunc& Func) const { verify(OctreeData != nullptr); OctreeData->FindElementsWithBoundsTest(BoxBounds, Func); }
	template <typename IterateBoundsFunc>
	inline void FindElementsWithBoundsTest(const FLKAnimVerletBound& BoxBounds, const IterateBoundsFunc& Func) const { verify(OctreeData != nullptr); OctreeData->FindElementsWithBoundsTest(ToOctreeBound(BoxBounds), Func); }
	inline void FindElements(const FLKAnimVerletBound& BoxBounds, const FLKOctreeElementFinder& Finder) const { verify(OctreeData != nullptr);	OctreeData->FindElementsWithBoundsTest(ToOctreeBound(BoxBounds), Finder); }

	/**
	 * this function will traverse the Octree using a fast box-box intersection and aborts traversal as soon as the Element function returns false.
	 * @param BoxBounds - the bounds to test if a node is traversed or skipped.
	 * @param Func - Function to call with each Element for nodes that passed bounds test.
	 */
	template <typename IterateBoundsFunc>
	inline void FindFirstElementWithBoundsTest(const FBoxCenterAndExtent& BoxBounds, const IterateBoundsFunc& Func) const { verify(OctreeData != nullptr);	OctreeData->FindFirstElementWithBoundsTest(BoxBounds, Func); }


	/**
	 * Adds an element to the octree.
	 * @param Element - The element to add.
	 */
	inline void AddElement(const FLKOctreeElement& Element) { verify(OctreeData != nullptr); OctreeData->AddElement(Element); }
	inline void AddElement(const FLKAnimVerletBoneIndicatorPair& InIndicator, const FLKAnimVerletBound& InBound)
	{ 
		verify(OctreeData != nullptr); 

		const FLKOctreeElement NewElem(InIndicator, InBound);
		OctreeData->AddElement(NewElem);
	}

	/**
	 * Removes an element from the octree.
	 * @param ElementId - The element to remove from the octree.
	 */
	inline void RemoveElement(FOctreeElementId2 ElementId) { verify(OctreeData != nullptr);	OctreeData->RemoveElement(ElementId); }

	inline bool UpdateElement(const FLKAnimVerletBoneIndicatorPair& InIndicator, const FLKAnimVerletBound& InNewBound)
	{ 
		verify(OctreeData != nullptr);	
		const FOctreeElementId2* FoundElementID = OctreeData->FindElementID(InIndicator);
		if (FoundElementID == nullptr)
			return false;

		OctreeData->RemoveElement(*FoundElementID);

		const FLKOctreeElement NewElem(InIndicator, InNewBound);
		OctreeData->AddElement(NewElem);
		return true;
	}

	/** Accesses an octree element by ID. */
	inline FLKOctreeElement& GetElementById(FOctreeElementId2 ElementId) { verify(OctreeData != nullptr); return OctreeData->GetElementById(ElementId); }
	inline const FLKOctreeElement& GetElementById(FOctreeElementId2 ElementId) const { verify(OctreeData != nullptr); return OctreeData->GetElementById(ElementId); }

	/**
	 * check if a FOctreeElementId2 is valid.
	 * @param ElementId - The ElementId to check.
	 */
	inline bool IsValidElementId(FOctreeElementId2 ElementId) const { return (OctreeData == nullptr) ? false : OctreeData->IsValidElementId(ElementId); }

	/** Writes stats for the octree to the log. */
	inline void DumpStats() const { if (OctreeData != nullptr) OctreeData->DumpStats(); }

	inline FLKAnimVerletBound GetRootBounds() const { verify(OctreeData != nullptr); return ToAnimVerletBound(OctreeData->GetRootBounds()); }

	inline FBoxCenterAndExtent ToOctreeBound(const FLKAnimVerletBound& InBound) const { return FBoxCenterAndExtent(InBound.GetCenter(), InBound.GetHalfExtents()); }
	inline FLKAnimVerletBound ToAnimVerletBound(const FBoxCenterAndExtent& InBound) const { return FLKAnimVerletBound::MakeBoundFromCenterHalfExtents(FVector(InBound.Center), FVector(InBound.Extent)); }

private:
	LKVerletBoneOctree* OctreeData = nullptr;
};