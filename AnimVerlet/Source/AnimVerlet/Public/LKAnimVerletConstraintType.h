#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"

using TExcludeBoneBits = TBitArray<TInlineAllocator<64>>;


struct FLKAnimVerletCollisionConstraintInput
{
public:
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* SimulateBonePairIndicators = nullptr;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;
};