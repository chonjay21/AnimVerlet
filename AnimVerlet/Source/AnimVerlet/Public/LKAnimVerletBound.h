#pragma once
#include <CoreMinimal.h>

struct FLKAnimVerletBound
{
public:
	FVector Min = FVector::ZeroVector;
	FVector Max = FVector::ZeroVector;

public:
	FORCEINLINE static FLKAnimVerletBound MakeBoundFromCenterHalfExtents(const FVector& InCenter, const FVector& InHalfExtents) { return FLKAnimVerletBound(InCenter - InHalfExtents, InCenter + InHalfExtents); }
	FORCEINLINE static FLKAnimVerletBound MakeBoundFromMinMax(const FVector& InMin, const FVector& InMax) { return FLKAnimVerletBound(InMin, InMax); }

public:
	FORCEINLINE FLKAnimVerletBound() = default;
	FORCEINLINE FLKAnimVerletBound(const FVector& InMin, const FVector& InMax) : Min(InMin), Max(InMax) {}

	FORCEINLINE FVector GetCenter() const { return (Max + Min) * 0.5f; }
	FORCEINLINE FVector GetHalfExtents() const { return GetExtents() * 0.5f; }
	FORCEINLINE FVector GetExtents() const { return (Max - Min); }

	FORCEINLINE const FVector& GetMin() const { return Min; }
	FORCEINLINE const FVector& GetMax() const { return Max; }

	FORCEINLINE bool IsNearlyEqual(const FLKAnimVerletBound& Other, float InEpsilon = KINDA_SMALL_NUMBER) const { return Min.Equals(Other.Min, InEpsilon) && Max.Equals(Other.Max, InEpsilon); }

	FORCEINLINE FLKAnimVerletBound& Expand(float Thickness)
	{
		Min -= FVector(Thickness);
		Max += FVector(Thickness);
		return *this;
	}
	FORCEINLINE FLKAnimVerletBound& Expand(const FVector& V)
	{
		Min = FVector(FMath::Min(Min.X, V.X), FMath::Min(Min.Y, V.Y), FMath::Min(Min.Z, V.Z));
		Max = FVector(FMath::Max(Max.X, V.X), FMath::Max(Max.Y, V.Y), FMath::Max(Max.Z, V.Z));
		return *this;
	}
	FORCEINLINE FLKAnimVerletBound& Expand(const FLKAnimVerletBound& Other)
	{
		Min = FVector(FMath::Min(Min.X, Other.Min.X), FMath::Min(Min.Y, Other.Min.Y), FMath::Min(Min.Z, Other.Min.Z));
		Max = FVector(FMath::Max(Max.X, Other.Max.X), FMath::Max(Max.Y, Other.Max.Y), FMath::Max(Max.Z, Other.Max.Z));
		return *this;
	}
	FORCEINLINE FLKAnimVerletBound& Shrink(const FLKAnimVerletBound& Other)
	{
		Min = FVector(FMath::Max(Min.X, Other.Min.X), FMath::Max(Min.Y, Other.Min.Y), FMath::Max(Min.Z, Other.Min.Z));
		Max = FVector(FMath::Min(Max.X, Other.Max.X), FMath::Min(Max.Y, Other.Max.Y), FMath::Min(Max.Z, Other.Max.Z));
		return *this;
	}

	FORCEINLINE bool IsIntersect(const FLKAnimVerletBound& Other) const
	{
		for (int32 i = 0; i < 3; ++i)
		{
			if (Other.Max[i] < Min[i] || Other.Min[i] > Max[i])
				return false;
		}
		return true;
	}

	FORCEINLINE FLKAnimVerletBound GetIntersection(const FLKAnimVerletBound& Other) const
	{
		return FLKAnimVerletBound(Min.ComponentMin(Other.Min), Max.ComponentMin(Other.Max));
	}

	FORCEINLINE bool IsContain(const FVector& Point, float Tolerance = KINDA_SMALL_NUMBER) const
	{
		for (int32 i = 0; i < 3; i++)
		{
			if (Point[i] < Min[i] - Tolerance || Point[i] > Max[i] + Tolerance)
				return false;
		}
		return true;
	}

	FORCEINLINE friend uint32 GetTypeHash(const FLKAnimVerletBound& Bound) { return HashCombine(GetTypeHash(Bound.Min), GetTypeHash(Bound.Max)); }
};