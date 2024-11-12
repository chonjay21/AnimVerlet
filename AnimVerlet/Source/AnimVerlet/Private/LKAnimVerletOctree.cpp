#include "LKAnimVerletOctree.h"

void FLKOctreeSemantics::SetElementId(TOctree2<FLKBroadphaseElement, FLKOctreeSemantics>& OctreeOwner, const FLKBroadphaseElement& Element, FOctreeElementId2 Id)
{
	LKVerletBoneOctree* CurOctree = static_cast<LKVerletBoneOctree*>(&OctreeOwner);
	CurOctree->SetElementIdImpl(Element, Id);
}

void LKVerletBoneOctree::SetElementIdImpl(const FLKBroadphaseElement& InElement, FOctreeElementId2 Id)
{
	ObjectToOctreeID.Emplace(InElement.BoneIndicatorPair, Id);
}