#pragma once
#include <CoreMinimal.h>
#include <Math/GenericOctreePublic.h>
#include <Math/GenericOctree.h>
#include "LKAnimVerletOctreeType.h"

///=========================================================================================================================================
/// TOctree2 Wrapper
///=========================================================================================================================================
class ANIMVERLET_API LKVerletBoneOctree : public TOctree2<FLKBroadphaseElement, FLKOctreeSemantics>
{
public:
	/// Use parent`s constructor
	using TOctree2::TOctree2;
	
	void SetElementIdImpl(const FLKBroadphaseElement& InElement, FOctreeElementId2 Id);
	const FOctreeElementId2* FindElementID(const FLKAnimVerletBoneIndicatorPair& InKey) const { return ObjectToOctreeID.Find(InKey); }
	void Reset() { ObjectToOctreeID.Reset(); }

private:
	TMap<FLKAnimVerletBoneIndicatorPair, FOctreeElementId2> ObjectToOctreeID;
};


///=========================================================================================================================================
/// LKAnimVerletOctree
///=========================================================================================================================================
class ANIMVERLET_API LKAnimVerletOctree
{
public:
	LKAnimVerletOctree() = default;

	void Initialize(const FVector& InOrigin, const FVector& InHalfExtents, const FVector& InCellHalfExtents)
	{
		verify(OctreeData == nullptr);
		OctreeData = new LKVerletBoneOctree(InOrigin, InHalfExtents.GetMax());
	}

	void Destroy() 
	{ 
		if (OctreeData != nullptr)
		{
			Reset();
			delete OctreeData;
		}
		OctreeData = nullptr; 
	}

	void Reset()
	{
		verify(IsInitialized());
		OctreeData->Reset();
		OctreeData->Destroy();
	}

	inline bool IsInitialized() const { return OctreeData != nullptr; }

	inline int32 GetNumNodes() const { verify(IsInitialized()); return OctreeData->GetNumNodes(); }

	/**
	 * this function will traverse the Octree using a fast box-box intersection this should be the preferred way of traversing the tree.
	 * @param BoxBounds - the bounds to test if a node is traversed or skipped.
	 * @param Func - Function to call with each Element for nodes that passed bounds test.
	 */
	template <typename IterateBoundsFunc>
	inline void FindElementsWithBoundsTest(const FBoxCenterAndExtent& BoxBounds, const IterateBoundsFunc& Func) const { verify(IsInitialized()); OctreeData->FindElementsWithBoundsTest(BoxBounds, Func); }
	template <typename IterateBoundsFunc>
	inline void FindElementsWithBoundsTest(const FLKAnimVerletBound& BoxBounds, const IterateBoundsFunc& Func) const { verify(IsInitialized()); OctreeData->FindElementsWithBoundsTest(ToOctreeBound(BoxBounds), Func); }
	inline void FindElements(const FLKAnimVerletBound& BoxBounds, const FLKBroadphaseElementFinder& Finder) const { verify(IsInitialized());	OctreeData->FindElementsWithBoundsTest(ToOctreeBound(BoxBounds), Finder); }

	/**
	 * this function will traverse the Octree using a fast box-box intersection and aborts traversal as soon as the Element function returns false.
	 * @param BoxBounds - the bounds to test if a node is traversed or skipped.
	 * @param Func - Function to call with each Element for nodes that passed bounds test.
	 */
	template <typename IterateBoundsFunc>
	inline void FindFirstElementWithBoundsTest(const FBoxCenterAndExtent& BoxBounds, const IterateBoundsFunc& Func) const { verify(IsInitialized());	OctreeData->FindFirstElementWithBoundsTest(BoxBounds, Func); }


	/**
	 * Adds an element to the octree.
	 * @param Element - The element to add.
	 */
	inline void AddElement(const FLKBroadphaseElement& Element) { verify(IsInitialized()); OctreeData->AddElement(Element); }
	inline void AddElement(const FLKAnimVerletBoneIndicatorPair& InIndicator, const FLKAnimVerletBound& InBound)
	{ 
		verify(IsInitialized()); 

		const FLKBroadphaseElement NewElem(InIndicator, InBound);
		OctreeData->AddElement(NewElem);
	}

	/**
	 * Removes an element from the octree.
	 * @param ElementId - The element to remove from the octree.
	 */
	inline void RemoveElement(FOctreeElementId2 ElementId) { verify(IsInitialized());	OctreeData->RemoveElement(ElementId); }

	inline bool UpdateElement(const FLKAnimVerletBoneIndicatorPair& InIndicator, const FLKAnimVerletBound& InNewBound)
	{ 
		verify(IsInitialized());	
		const FOctreeElementId2* FoundElementID = OctreeData->FindElementID(InIndicator);
		if (FoundElementID == nullptr)
			return false;

		OctreeData->RemoveElement(*FoundElementID);

		const FLKBroadphaseElement NewElem(InIndicator, InNewBound);
		OctreeData->AddElement(NewElem);
		return true;
	}

	/** Accesses an octree element by ID. */
	inline FLKBroadphaseElement& GetElementById(FOctreeElementId2 ElementId) { verify(IsInitialized()); return OctreeData->GetElementById(ElementId); }
	inline const FLKBroadphaseElement& GetElementById(FOctreeElementId2 ElementId) const { verify(IsInitialized()); return OctreeData->GetElementById(ElementId); }

	/*inline FLKBroadphaseElement* FindElementByIndicatorPair(const FLKAnimVerletBoneIndicatorPair& InIndicator) 
	{ 
		verify(IsInitialized()); 
		const FOctreeElementId2* FoundElementID = OctreeData->FindElementID(InIndicator);
		if (FoundElementID == nullptr)
			return nullptr;

		return &OctreeData->GetElementById(*FoundElementID);
	}
	inline const FLKBroadphaseElement* FindElementByIndicatorPair(const FLKAnimVerletBoneIndicatorPair& InIndicator) const
	{ 
		verify(IsInitialized());
		const FOctreeElementId2* FoundElementID = OctreeData->FindElementID(InIndicator);
		if (FoundElementID == nullptr)
			return nullptr;

		return &OctreeData->GetElementById(*FoundElementID);
	}*/

	/**
	 * check if a FOctreeElementId2 is valid.
	 * @param ElementId - The ElementId to check.
	 */
	inline bool IsValidElementId(FOctreeElementId2 ElementId) const { return (OctreeData == nullptr) ? false : OctreeData->IsValidElementId(ElementId); }


	inline FLKAnimVerletBound GetRootBounds() const { verify(IsInitialized()); return ToAnimVerletBound(OctreeData->GetRootBounds()); }

	inline FBoxCenterAndExtent ToOctreeBound(const FLKAnimVerletBound& InBound) const { return FBoxCenterAndExtent(InBound.GetCenter(), InBound.GetHalfExtents()); }
	inline FLKAnimVerletBound ToAnimVerletBound(const FBoxCenterAndExtent& InBound) const { return FLKAnimVerletBound::MakeBoundFromCenterHalfExtents(FVector(InBound.Center), FVector(InBound.Extent)); }

	/** Writes stats for the octree to the log. */
	inline void DumpStats() const { if (OctreeData != nullptr) OctreeData->DumpStats(); }

private:
	LKVerletBoneOctree* OctreeData = nullptr;
};