#include "LKAnimVerletBoneTree.h"

void FLKOctreeSemantics::SetElementId(TOctree2<FLKOctreeElement, FLKOctreeSemantics>& OctreeOwner, const FLKOctreeElement& Element, FOctreeElementId2 Id)
{
	LKVerletBoneOctree* CurOctree = static_cast<LKVerletBoneOctree*>(&OctreeOwner);
	CurOctree->SetElementIdImpl(Element, Id);
}

void LKVerletBoneOctree::SetElementIdImpl(const FLKOctreeElement& InElement, FOctreeElementId2 Id)
{
	ObjectToOctreeID.Emplace(InElement.BoneIndicatorPair, Id);
}