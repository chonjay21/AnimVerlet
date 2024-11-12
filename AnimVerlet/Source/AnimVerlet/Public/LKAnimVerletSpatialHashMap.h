#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseType.h"

///=========================================================================================================================================
/// LKAnimVerletSpatialHashMap
///=========================================================================================================================================
class ANIMVERLET_API LKAnimVerletSpatialHashMap
{
public:
	static const int32 NumBucket = 64;

	LKAnimVerletSpatialHashMap() = default;

	void Initialize(const FVector& InOrigin, const FVector& InHalfExtents, const FVector& InCellHalfExtents)
	{
		Origin = InOrigin;
		HalfExtents = InHalfExtents;
		CellHalfExtents = InCellHalfExtents;
		HashMap.SetNum(NumBucket);
	}

	void Destroy() 
	{ 
		HashMap.Reset();
		ObjectToBound.Reset();
		VisitedSet.Reset();
	}

	void Reset()
	{
		verify(IsInitialized());
		HashMap.Reset();
		HashMap.SetNum(NumBucket);
		
		VisitedSet.Reset();
		ObjectToBound.Reset();
	}

	inline bool IsInitialized() const { return CellHalfExtents.IsNearlyZero() == false; }
	inline int32 GetNumNodes() const { verify(IsInitialized()); return HashMap.Num(); }

	inline void FindElements(const FLKAnimVerletBound& Bounds, const FLKBroadphaseElementFinder& Finder) const 
	{ 
		verify(IsInitialized());
		
		VisitedSet.Reset();
		const FVector& BoundMin = Bounds.GetMin();
		const FVector& BoundMax = Bounds.GetMax();
		const FIntVector LocalMin = LocationToCellLocalCoords(BoundMin);
		const FIntVector LocalMax = LocationToCellLocalCoords(BoundMax);
		for (int32 CurX = LocalMin.X; CurX <= LocalMax.X; ++CurX)
		{
			for (int32 CurY = LocalMin.Y; CurY <= LocalMax.Y; ++CurY)
			{
				for (int32 CurZ = LocalMin.Z; CurZ <= LocalMax.Z; ++CurZ)
				{
					const FIntVector CurLocalCoords(CurX, CurY, CurZ);
					const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
					const TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
					for (const FLKBroadphaseElement& Found : CurList)
					{
						bool bAlreadyVisit = false;
						VisitedSet.Emplace(Found.BoneIndicatorPair, &bAlreadyVisit);
						if (bAlreadyVisit)
							continue;

						Finder(Found);
					}
				}
			}
		}
	}

	inline void AddElement(const FLKBroadphaseElement& Element) 
	{ 
		verify(IsInitialized()); 
		
		const FVector& BoundMin = Element.CachedBound.GetMin();
		const FVector& BoundMax = Element.CachedBound.GetMax();
		const FIntVector LocalMin = LocationToCellLocalCoords(BoundMin);
		const FIntVector LocalMax = LocationToCellLocalCoords(BoundMax);
		for (int32 CurX = LocalMin.X; CurX <= LocalMax.X; ++CurX)
		{
			for (int32 CurY = LocalMin.Y; CurY <= LocalMax.Y; ++CurY)
			{
				for (int32 CurZ = LocalMin.Z; CurZ <= LocalMax.Z; ++CurZ)
				{
					const FIntVector CurLocalCoords(CurX, CurY, CurZ);
					const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
					TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
					CurList.Emplace(Element);
					ObjectToBound.Emplace(Element.BoneIndicatorPair, Element.CachedBound);
				}
			}
		}
	}

	inline bool UpdateElement(const FLKAnimVerletBoneIndicatorPair& InIndicator, const FLKAnimVerletBound& InNewBound)
	{ 
		verify(IsInitialized());	

		const FLKAnimVerletBound* OldBound = ObjectToBound.Find(InIndicator);
		if (OldBound == nullptr)
			return false;

		const FVector& OldMin = OldBound->GetMin();
		const FVector& OldMax = OldBound->GetMax();
		const FIntVector OldLocalMin = LocationToCellLocalCoords(OldMin);
		const FIntVector OldLocalMax = LocationToCellLocalCoords(OldMax);

		const FVector& NewMin = InNewBound.GetMin();
		const FVector& NewMax = InNewBound.GetMax();
		const FIntVector NewLocalMin = LocationToCellLocalCoords(NewMin);
		const FIntVector NewLocalMax = LocationToCellLocalCoords(NewMax);

		/*if (OldBound->IsIntersect(InNewBound))
		{
			for (int32 CurX = OldLocalMin.X; CurX < NewLocalMin.X; ++CurX)
			{
				for (int32 CurY = OldLocalMin.Y; CurY < NewLocalMin.Y; ++CurY)
				{
					for (int32 CurZ = OldLocalMin.Z; CurZ < NewLocalMin.Z; ++CurZ)
					{
						const FIntVector CurLocalCoords(CurX, CurY, CurZ);
						const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
						TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
						const int32 FoundIndex = CurList.IndexOfByKey(InIndicator);
						if (FoundIndex != INDEX_NONE)
							CurList.RemoveAtSwap(FoundIndex, 1, false);
					}
				}
			}
			for (int32 CurX = OldLocalMax.X + 1; CurX <= NewLocalMax.X; ++CurX)
			{
				for (int32 CurY = OldLocalMax.Y + 1; CurY <= NewLocalMax.Y; ++CurY)
				{
					for (int32 CurZ = OldLocalMax.Z + 1; CurZ <= NewLocalMax.Z; ++CurZ)
					{
						const FIntVector CurLocalCoords(CurX, CurY, CurZ);
						const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
						TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
						CurList.Emplace(InIndicator, InNewBound);
						ObjectToBound.Emplace(InIndicator, InNewBound);
					}
				}
			}

			for (int32 CurX = NewLocalMin.X; CurX < OldLocalMin.X; ++CurX)
			{
				for (int32 CurY = NewLocalMin.Y; CurY < OldLocalMin.Y; ++CurY)
				{
					for (int32 CurZ = NewLocalMin.Z; CurZ < OldLocalMin.Z; ++CurZ)
					{
						const FIntVector CurLocalCoords(CurX, CurY, CurZ);
						const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
						TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
						CurList.Emplace(InIndicator, InNewBound);
						ObjectToBound.Emplace(InIndicator, InNewBound);
					}
				}
			}
			for (int32 CurX = NewLocalMax.X + 1; CurX <= OldLocalMax.X; ++CurX)
			{
				for (int32 CurY = NewLocalMax.Y + 1; CurY <= OldLocalMax.Y; ++CurY)
				{
					for (int32 CurZ = NewLocalMax.Z + 1; CurZ <= OldLocalMax.Z; ++CurZ)
					{
						const FIntVector CurLocalCoords(CurX, CurY, CurZ);
						const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
						TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
						const int32 FoundIndex = CurList.IndexOfByKey(InIndicator);
						if (FoundIndex != INDEX_NONE)
							CurList.RemoveAtSwap(FoundIndex, 1, false);
					}
				}
			}
		}
		else*/
		{
			for (int32 CurX = OldLocalMin.X; CurX <= OldLocalMax.X; ++CurX)
			{
				for (int32 CurY = OldLocalMin.Y; CurY <= OldLocalMax.Y; ++CurY)
				{
					for (int32 CurZ = OldLocalMin.Z; CurZ <= OldLocalMax.Z; ++CurZ)
					{
						const FIntVector CurLocalCoords(CurX, CurY, CurZ);
						const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
						TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
						const int32 FoundIndex = CurList.IndexOfByKey(InIndicator);
						if (FoundIndex != INDEX_NONE)
							CurList.RemoveAtSwap(FoundIndex, 1, false);
					}
				}
			}

			for (int32 CurX = NewLocalMin.X; CurX <= NewLocalMax.X; ++CurX)
			{
				for (int32 CurY = NewLocalMin.Y; CurY <= NewLocalMax.Y; ++CurY)
				{
					for (int32 CurZ = NewLocalMin.Z; CurZ <= NewLocalMax.Z; ++CurZ)
					{
						const FIntVector CurLocalCoords(CurX, CurY, CurZ);
						const int32 HashMapIndex = LocalCoordsToHashMapIndex(CurLocalCoords);
						TArray<FLKBroadphaseElement>& CurList = HashMap[HashMapIndex];	verify(HashMap.IsValidIndex(HashMapIndex));
						CurList.Emplace(InIndicator, InNewBound);
						ObjectToBound.Emplace(InIndicator, InNewBound);
					}
				}
			}
		}
		return true;
	}

	inline FLKAnimVerletBound GetRootBounds() const { verify(IsInitialized()); return FLKAnimVerletBound::MakeBoundFromCenterHalfExtents(Origin, HalfExtents); }
	inline void DumpStats() const { /** Not yet implemented */ }

private:
	inline FIntVector LocationToCellLocalCoords(const FVector& InLocation) const
	{
		const FIntVector LocalCoords(FMath::Floor(InLocation.X / CellHalfExtents.X), FMath::Floor(InLocation.Y / CellHalfExtents.Y), FMath::Floor(InLocation.Z / CellHalfExtents.Z));
		return LocalCoords;
	}

	inline int64 LocalCoordsToHash(const FIntVector& InLocalCoords) const
	{
		const int64 Hashed = (InLocalCoords.X * 92837111) ^ (InLocalCoords.Y * 689287499) ^ (InLocalCoords.Z * 283923481);
		return Hashed;
	}

	inline int32 HashToHashMapIndex(int64 InHash) const
	{
		return FMath::Abs(InHash) % NumBucket;
	}

	inline int32 LocalCoordsToHashMapIndex(const FIntVector& InLocalCoords) const
	{
		const int64 NewHash = LocalCoordsToHash(InLocalCoords);
		return HashToHashMapIndex(NewHash);
	}

private:
	FVector Origin = FVector::ZeroVector;
	FVector HalfExtents = FVector::ZeroVector;
	FVector CellHalfExtents = FVector::ZeroVector;
	TArray<TArray<FLKBroadphaseElement>> HashMap;
	TMap<FLKAnimVerletBoneIndicatorPair, FLKAnimVerletBound> ObjectToBound;
	mutable TSet<FLKAnimVerletBoneIndicatorPair> VisitedSet;
};