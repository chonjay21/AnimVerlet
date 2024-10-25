#pragma once
#include <CoreMinimal.h>
#include <Engine/DataAsset.h>
#include "LKAnimVerletCollisionData.generated.h"

///=========================================================================================================================================
/// FLKAnimVerletCollisionData(Base Interface)
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionData
{
	GENERATED_BODY()

public:
	/** Use the LocationOffset and RotationOffset as the world transform for this collision shape.(This variable can be used to set collision that exists outside of a skeletal mesh every frame by an animation blueprint or source code.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (DisplayPriority = "1"))
	bool bUseAbsoluteWorldTransform = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (DisplayPriority = "2", EditCondition = "bUseAbsoluteWorldTransform == false"))
	FName AttachBoneName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (DisplayPriority = "3"))
	TArray<FName> ExcludeBones;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "4"))
	FVector LocationOffset = FVector::ZeroVector;

public:
	virtual ~FLKAnimVerletCollisionData() {}
	virtual bool IsSphere() const { return false; }
	virtual bool IsCapsule() const { return false; }
	virtual bool IsBox() const { return false; }
	virtual bool IsPlane() const { return false; }

protected:
	void ConvertToShapeBase(OUT struct FLKAnimVerletCollisionShape& OutShapeBase) const;
	void ConvertFromShapeBase(const struct FLKAnimVerletCollisionShape& InShapeBase);
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletCollisionDataSphere
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionDataSphere : public FLKAnimVerletCollisionData
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Radius = 0.0f;

public:
	virtual bool IsSphere() const override final { return true; }

public:
	void ConvertToShape(OUT struct FLKAnimVerletCollisionSphere& OutSphere) const;
	void ConvertFromShape(const struct FLKAnimVerletCollisionSphere& InSphere);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataCapsule
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionDataCapsule : public FLKAnimVerletCollisionData
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "5"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Radius = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float HalfHeight = 0.0f;

public:
	virtual bool IsCapsule() const override final { return true; }

public:
	void ConvertToShape(OUT struct FLKAnimVerletCollisionCapsule& OutCapsule) const;
	void ConvertFromShape(const struct FLKAnimVerletCollisionCapsule& InCapsule);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataBox
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionDataBox : public FLKAnimVerletCollisionData
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "5"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	FVector HalfExtents = FVector::ZeroVector;

public:
	virtual bool IsBox() const override final { return true; }

public:
	void ConvertToShape(OUT struct FLKAnimVerletCollisionBox& OutBox) const;
	void ConvertFromShape(const struct FLKAnimVerletCollisionBox& InBox);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataPlane
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionDataPlane : public FLKAnimVerletCollisionData
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "5"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	bool bFinitePlane = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (EditCondition = "bFinitePlane"))
	FVector2D FinitePlaneHalfExtents = FVector2D::ZeroVector;

public:
	virtual bool IsPlane() const override final { return true; }

public:
	void ConvertToShape(OUT struct FLKAnimVerletCollisionPlane& OutPlane) const;
	void ConvertFromShape(const struct FLKAnimVerletCollisionPlane& InPlane);
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletCollisionDataList
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionDataList
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionDataSphere> SphereCollisionData;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionDataCapsule> CapsuleCollisionData;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionDataBox> BoxCollisionData;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionDataPlane> PlaneCollisionData;

public:
	void ConvertToShape(OUT struct FLKAnimVerletCollisionShapeList& OutShapeList) const;
	void ConvertFromShape(const struct FLKAnimVerletCollisionShapeList& InShapeList);
	void Reset();
};
///=========================================================================================================================================


///=========================================================================================================================================
/// ULKAnimVerletCollisionDataAsset
///=========================================================================================================================================
UCLASS(Blueprintable, BlueprintType)
class ANIMVERLET_API ULKAnimVerletCollisionDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	FLKAnimVerletCollisionDataList CollisionDataList;

public:
	void ConvertToShape(OUT struct FLKAnimVerletCollisionShapeList& OutShapeList) const;
	void ConvertFromShape(const struct FLKAnimVerletCollisionShapeList& InShapeList);
	void Reset();
};
///=========================================================================================================================================