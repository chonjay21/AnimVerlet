#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseType.h"
#include "LKAnimVerletBVH.h"

class LKAnimVerletBroadphaseContainer
{
public:
	void InitializeFromBones(TArray<FLKAnimVerletBone>* Bones, float MaxThickness);
	void InitializeFromPairs(TArray<FLKAnimVerletBone>* Bones, TArray<FLKAnimVerletBoneIndicatorPair>* Pairs, float MaxThickness);
	void InitializeFromTriangles(TArray<FLKAnimVerletBone>* Bones, TArray<FLKAnimVerletBoneIndicatorTriangle>* Triangles, float MaxThickness);
	void Destroy();

	void Update();

	template<typename FuncType>
	void QueryAABB(const FLKAnimVerletBound& InAABB, FuncType&& InCallback) const { BroadphaseTree.QueryAABB(InAABB, InCallback); }
	void* GetUserData(LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID InID) { return BroadphaseTree.GetUserData(InID); }

private:
	void Initialize(TArray<FLKAnimVerletBone>* Bones, float MaxThickness);

private:
	TArray<FLKAnimVerletBone>* SimulatingBones = nullptr;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairsNullable = nullptr;
	TArray<FLKAnimVerletBoneIndicatorTriangle>* BoneTrianglesNullable = nullptr;

	TArray<LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID> BroadphaseIdList;
	LKAnimVerletBVH<FLKAnimVerletBpData> BroadphaseTree;
};