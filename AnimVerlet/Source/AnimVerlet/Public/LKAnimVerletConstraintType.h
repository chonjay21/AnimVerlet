#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"

using TExcludeBoneBits = TBitArray<TInlineAllocator<64>>;

struct FLKAnimVerletBroadphaseSpace
{
public:
	TArray<FLKAnimVerletBoneIndicatorPair> BonePairIndicatorSpace;

public:
	void ResetBroadphaseSpace()
	{
		BonePairIndicatorSpace.Reset();
	}
};

struct FLKAnimVerletBroadphaseInput
{
public:
	bool bUseBroadphase = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* TargetBonePairIndicators = nullptr;

	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairIndicatorSpace = nullptr;
	class LKAnimVerletBroadphaseContainer* BroadphaseContainer = nullptr;

public:
	void ResetBroadphaseInput()
	{
		bUseBroadphase = false;
		TargetBonePairIndicators = nullptr;

		BonePairIndicatorSpace = nullptr;
		BroadphaseContainer = nullptr;
	}
};


struct FLKAnimVerletCollisionConstraintInput
{
public:
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* SimulateBonePairIndicators = nullptr;

	FLKAnimVerletBroadphaseInput BroadphaseInput;
	bool bUseXPBDSolver = false;
	double Compliance = 0.0;
};