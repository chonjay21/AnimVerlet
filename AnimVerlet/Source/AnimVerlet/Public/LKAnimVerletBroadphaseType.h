#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletBvhType.h"

enum class ELKAnimVerletBpDataCategory : uint8
{
	Bone,
	Pair,
	Triangle,
};

struct FLKAnimVerletBpData
{
	ELKAnimVerletBpDataCategory Type = ELKAnimVerletBpDataCategory::Bone;
	FLKAnimVerletBoneIndicator BoneA;
	FLKAnimVerletBoneIndicator BoneB;
	FLKAnimVerletBoneIndicator BoneC;
	int32 ListIndex = -1;
};