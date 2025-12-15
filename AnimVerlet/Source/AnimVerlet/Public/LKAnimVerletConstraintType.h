#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"

using TExcludeBoneBits = TBitArray<TInlineAllocator<64>>;

struct FLKAnimVerletCollisionConstraintInput
{
public:
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseBroadphase = false;
	bool bUseCapsuleCollisionForChain = false;
	bool bSingleChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* SimulateBonePairIndicators = nullptr;
	TArray<FLKAnimVerletBoneIndicatorTriangle>* SimulateBoneTriangleIndicators = nullptr;
	class LKAnimVerletBroadphaseContainer* BroadphaseContainer = nullptr;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;
};