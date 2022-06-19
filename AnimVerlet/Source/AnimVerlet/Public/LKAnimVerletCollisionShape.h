#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletConstraint.h"
#include "LKAnimVerletCollisionShape.generated.h"

///=========================================================================================================================================
/// FLKAnimVerletCollisionShape(Base Interface)
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionShape
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (DisplayPriority = "1"))
	FBoneReference AttachedBone;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (DisplayPriority = "2"))
	TArray<FBoneReference> ExcludeBones;

	UPROPERTY(EditAnywhere, Category = "Offset", meta = (DisplayPriority = "3"))
	FVector LocationOffset = FVector::ZeroVector;

public:
	TExcludeBoneBits ExcludeBoneBits;

public:
	virtual ~FLKAnimVerletCollisionShape() {}
	virtual bool IsSphere() const { return false; }
	virtual bool IsCapsule() const { return false; }
	virtual bool IsBox() const { return false; }
	virtual bool IsPlane() const { return false; }
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletCollisionSphere
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionSphere : public FLKAnimVerletCollisionShape
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Radius = 0.0f;

public:
	virtual bool IsSphere() const override final { return true; }
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionCapsule
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionCapsule : public FLKAnimVerletCollisionShape
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Offset", meta = (DisplayPriority = "4"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Radius = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float HalfHeight = 0.0f;

public:
	virtual bool IsCapsule() const override final { return true; }
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionBox
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionBox : public FLKAnimVerletCollisionShape
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Offset", meta = (DisplayPriority = "4"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Collision")
	FVector HalfExtents = FVector::ZeroVector;

public:
	virtual bool IsBox() const override final { return true; }
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionPlane
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionPlane : public FLKAnimVerletCollisionShape
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Offset", meta = (DisplayPriority = "4"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Collision")
	bool bFinitePlane = false;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (EditCondition = "bFinitePlane"))
	FVector2D FinitePlaneHalfExtents = FVector2D::ZeroVector;

public:
	virtual bool IsPlane() const override final { return true; }
};
///=========================================================================================================================================