#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBound.h"

/// Expanded AABB parameter
struct FLKAnimVerletBvhSettings
{
	FVector FatExtension = FVector(10.0f, 10.0f, 10.0f);

	/// SweptAABB when Update
	float DisplacementMultiplier = 2.0f;

	/// for tree rotation
	int32 BalanceThreshold = 1;
};

struct FLKAnimVerletBvhRay
{
	FVector Origin = FVector::ZeroVector;
	FVector Direction = FVector(1.0f, 0.0f, 0.0f);
	float MaxT = TNumericLimits<float>::Max();
};


namespace LKAnimVerletUtil
{
	inline bool RayCastAABB(OUT float& OutTMin, const FLKAnimVerletBvhRay& Ray, const FLKAnimVerletBound& Box)
	{
		float TMin = 0.0f;
		float TMax = Ray.MaxT;

		const FVector Min = Box.Min;
		const FVector Max = Box.Max;

		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const float Origin = Ray.Origin[Axis];
			const float Dir = Ray.Direction[Axis];

			if (FMath::Abs(Dir) < KINDA_SMALL_NUMBER)
			{
				if (Origin < Min[Axis] || Origin > Max[Axis])
					return false;
			}
			else
			{
				const float InvD = 1.0f / Dir;
				float T1 = (Min[Axis] - Origin) * InvD;
				float T2 = (Max[Axis] - Origin) * InvD;
				if (T1 > T2) 
					Swap(T1, T2);

				TMin = FMath::Max(TMin, T1);
				TMax = FMath::Min(TMax, T2);

				if (TMin > TMax)
					return false;
			}
		}

		OutTMin = TMin;
		return true;
	}
};