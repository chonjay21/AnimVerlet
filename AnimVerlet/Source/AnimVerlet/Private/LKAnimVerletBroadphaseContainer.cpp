#include "LKAnimVerletBroadphaseContainer.h"

#include "LKAnimVerletBroadphaseType.h"

void LKAnimVerletBroadphaseContainer::Initialize(TArray<FLKAnimVerletBone>* Bones, float MaxThickness)
{
	verify(Bones != nullptr);
	SimulatingBones = Bones;

	FLKAnimVerletBvhSettings BroadphaseSettings;
	{
		const float FatMargin = FMath::Max(10.0f, MaxThickness);
		BroadphaseSettings.FatExtension = FVector(FatMargin, FatMargin, FatMargin);
	}
	BroadphaseTree.Initialize(BroadphaseSettings, SimulatingBones->Num());
}

void LKAnimVerletBroadphaseContainer::InitializeFromBones(TArray<FLKAnimVerletBone>* Bones, float MaxThickness)
{
	Initialize(Bones, MaxThickness);
	BonePairsNullable = nullptr;
	BoneTrianglesNullable = nullptr;

	BroadphaseIdList.Reserve(Bones->Num());
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		FLKAnimVerletBone& CurBone = (*Bones)[i];
		const FLKAnimVerletBound CurBound = CurBone.MakeBound();
		FLKAnimVerletBpData NewData;
		{
			NewData.Type = ELKAnimVerletBpDataCategory::Bone;
			NewData.BoneA = FLKAnimVerletBoneIndicator(i, false);
			NewData.ListIndex = i;
		}
		const LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID NewID = BroadphaseTree.Insert(CurBound, NewData);
		BroadphaseIdList.Add(NewID);
	}
}

void LKAnimVerletBroadphaseContainer::InitializeFromPairs(TArray<FLKAnimVerletBone>* Bones, TArray<FLKAnimVerletBoneIndicatorPair>* Pairs, float MaxThickness)
{
	Initialize(Bones, MaxThickness);
	BonePairsNullable = Pairs;
	BoneTrianglesNullable = nullptr;

	BroadphaseIdList.Reserve(Pairs->Num());
	for (int32 i = 0; i < Pairs->Num(); ++i)
	{
		FLKAnimVerletBoneIndicatorPair& CurPair = (*Pairs)[i];
		const FLKAnimVerletBound CurBound = CurPair.MakeBound(*Bones);
		FLKAnimVerletBpData NewData;
		{
			NewData.Type = ELKAnimVerletBpDataCategory::Pair;
			NewData.BoneA = CurPair.BoneA;
			NewData.BoneB = CurPair.BoneB;
			NewData.ListIndex = i;
		}
		const LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID NewID = BroadphaseTree.Insert(CurBound, NewData);
		BroadphaseIdList.Add(NewID);
	}
}

void LKAnimVerletBroadphaseContainer::InitializeFromTriangles(TArray<FLKAnimVerletBone>* Bones, TArray<FLKAnimVerletBoneIndicatorTriangle>* Triangles, float MaxThickness)
{
	Initialize(Bones, MaxThickness);
	BonePairsNullable = nullptr;
	BoneTrianglesNullable = Triangles;

	BroadphaseIdList.Reserve(Triangles->Num());
	for (int32 i = 0; i < Triangles->Num(); ++i)
	{
		FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*Triangles)[i];
		const FLKAnimVerletBound CurBound = CurTriangle.MakeBound(*Bones);
		FLKAnimVerletBpData NewData;
		{
			NewData.Type = ELKAnimVerletBpDataCategory::Triangle;
			NewData.BoneA = CurTriangle.BoneA;
			NewData.BoneB = CurTriangle.BoneB;
			NewData.BoneC = CurTriangle.BoneC;
			NewData.ListIndex = i;
		}
		const LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID NewID = BroadphaseTree.Insert(CurBound, NewData);
		BroadphaseIdList.Add(NewID);
	}
}

void LKAnimVerletBroadphaseContainer::Destroy()
{
	SimulatingBones = nullptr;
	BonePairsNullable = nullptr;
	BoneTrianglesNullable = nullptr;

	BroadphaseIdList.Reset();
	BroadphaseTree.Destroy();
}

void LKAnimVerletBroadphaseContainer::Update()
{
	verify(SimulatingBones != nullptr);

	if (BoneTrianglesNullable != nullptr)
	{
		verify(BoneTrianglesNullable->Num() == BroadphaseIdList.Num());
		for (int32 i = 0; i < BoneTrianglesNullable->Num(); ++i)
		{
			const LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID CurBroadphaseID = BroadphaseIdList[i];

			const FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*BoneTrianglesNullable)[i];
			if (CurTriangle.BoneA.IsValidBoneIndicator() == false || CurTriangle.BoneB.IsValidBoneIndicator() == false || CurTriangle.BoneC.IsValidBoneIndicator() == false)
				continue;

			const FVector MoveDeltaA = (*SimulatingBones)[CurTriangle.BoneA.AnimVerletBoneIndex].MoveDelta;
			const FVector MoveDeltaB = (*SimulatingBones)[CurTriangle.BoneB.AnimVerletBoneIndex].MoveDelta;
			const FVector MoveDeltaC = (*SimulatingBones)[CurTriangle.BoneC.AnimVerletBoneIndex].MoveDelta;
			const FVector MoveDelta = (MoveDeltaA + MoveDeltaB + MoveDeltaC) / 3.0f;

			const FLKAnimVerletBound CurBound = CurTriangle.MakeBound(*SimulatingBones);
			BroadphaseTree.Update(CurBroadphaseID, CurBound, MoveDelta);
		}
	}
	else if (BonePairsNullable != nullptr)
	{
		verify(BonePairsNullable->Num() == BroadphaseIdList.Num());
		for (int32 i = 0; i < BonePairsNullable->Num(); ++i)
		{
			const LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID CurBroadphaseID = BroadphaseIdList[i];

			const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairsNullable)[i];
			if (CurPair.BoneA.IsValidBoneIndicator() == false || CurPair.BoneB.IsValidBoneIndicator() == false)
				continue;

			const FVector MoveDeltaA = (*SimulatingBones)[CurPair.BoneA.AnimVerletBoneIndex].MoveDelta;
			const FVector MoveDeltaB = (*SimulatingBones)[CurPair.BoneB.AnimVerletBoneIndex].MoveDelta;
			const FVector MoveDelta = (MoveDeltaA + MoveDeltaB) * 0.5f;

			const FLKAnimVerletBound CurBound = CurPair.MakeBound(*SimulatingBones);
			BroadphaseTree.Update(CurBroadphaseID, CurBound, MoveDelta);
		}
	}
	else
	{
		verify(SimulatingBones->Num() == BroadphaseIdList.Num());
		for (int32 i = 0; i < SimulatingBones->Num(); ++i)
		{
			const LKAnimVerletBVH<FLKAnimVerletBpData>::LKBvhID CurBroadphaseID = BroadphaseIdList[i];

			const FLKAnimVerletBone& CurBone = (*SimulatingBones)[i];
			const FVector MoveDelta = CurBone.MoveDelta;

			const FLKAnimVerletBound CurBound = CurBone.MakeBound();
			BroadphaseTree.Update(CurBroadphaseID, CurBound, MoveDelta);
		}
	}
}