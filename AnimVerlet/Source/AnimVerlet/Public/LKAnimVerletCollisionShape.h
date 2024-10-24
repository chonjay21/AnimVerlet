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
	/** Use the LocationOffset and RotationOffset as the world transform for this collision shape.(This variable can be used to set collision that exists outside of a skeletal mesh every frame by an animation blueprint or source code.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (DisplayPriority = "1"))
	bool bUseAbsoluteWorldTransform = false;
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (DisplayPriority = "2", EditCondition = "bUseAbsoluteWorldTransform == false"))
	FBoneReference AttachedBone;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (DisplayPriority = "3"))
	TArray<FBoneReference> ExcludeBones;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "4"))
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "5"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
	float Radius = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (ClampMin = "0.0", ForceUnits = "cm"))
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "5"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Offset", meta = (DisplayPriority = "5"))
	FRotator RotationOffset = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	bool bFinitePlane = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision", meta = (EditCondition = "bFinitePlane"))
	FVector2D FinitePlaneHalfExtents = FVector2D::ZeroVector;

public:
	virtual bool IsPlane() const override final { return true; }
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletCollisionShapeList
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionShapeList
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionSphere> SphereCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionCapsule> CapsuleCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionBox> BoxCollisionShapes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
	TArray<FLKAnimVerletCollisionPlane> PlaneCollisionShapes;

public:
	void ResetCollisionShapeList()
	{
		SphereCollisionShapes.Reset();
		CapsuleCollisionShapes.Reset();
		BoxCollisionShapes.Reset();
		PlaneCollisionShapes.Reset();
	}
};
///=========================================================================================================================================