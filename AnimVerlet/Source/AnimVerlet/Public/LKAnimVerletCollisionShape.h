/**
	FBoneReference cannot be included in DataAsset due to limitations in UE5.
	So I split AnimVerletCollider into AnimVerletCollisionShape and AnimVerletCollisionData.
*/

#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletConstraint.h"
#include "LKAnimVerletCollisionShapeType.h"
#include "LKAnimVerletCollisionShape.generated.h"

///=========================================================================================================================================
/// FLKAnimVerletCollisionShape(Base Interface)
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletCollisionShape : public FLKAnimVerletColliderInterface
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
	virtual ELKAnimVerletCollider GetColliderType() const override { return ELKAnimVerletCollider::None; }
	virtual bool IsUseAbsoluteWorldTransform() const override { return bUseAbsoluteWorldTransform; }
	virtual FName GetAttachBoneName() const override { return bUseAbsoluteWorldTransform ? NAME_None : AttachedBone.BoneName; }
	virtual FVector GetLocation() const override { return LocationOffset; }
	virtual void SetLocation(const FVector& InLocation) override { LocationOffset = InLocation; }

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
	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Sphere; }
	virtual bool IsSphere() const override final { return true; }

	virtual FVector GetHalfExtents() const override final { return FVector(Radius, Radius, Radius); }
	virtual void SetHalfExtents(const FVector& InHalfExtents) override final { Radius = InHalfExtents.X; }
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
	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Capsule; }
	virtual bool IsCapsule() const override final { return true; }
	virtual FRotator GetRotation() const override final { return RotationOffset; }
	virtual void SetRotation(const FRotator& InRotator) override { RotationOffset = InRotator; }

	virtual FVector GetHalfExtents() const override final { return FVector(Radius, Radius, HalfHeight); }
	virtual void SetHalfExtents(const FVector& InHalfExtents) override final { Radius = InHalfExtents.X; HalfHeight = InHalfExtents.Z; }
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
	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Box; }
	virtual bool IsBox() const override final { return true; }
	virtual FRotator GetRotation() const override final { return RotationOffset; }
	virtual void SetRotation(const FRotator& InRotator) override { RotationOffset = InRotator; }

	virtual FVector GetHalfExtents() const override final { return HalfExtents; }
	virtual void SetHalfExtents(const FVector& InHalfExtents) override final { HalfExtents = InHalfExtents; }
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
	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Plane; }
	virtual bool IsPlane() const override final { return true; }
	virtual FRotator GetRotation() const override final { return RotationOffset; }
	virtual void SetRotation(const FRotator& InRotator) override { RotationOffset = InRotator; }

	virtual FVector GetHalfExtents() const override final { return bFinitePlane ? FVector(FinitePlaneHalfExtents, 0.0f) : FVector::ZeroVector; }
	virtual void SetHalfExtents(const FVector& InHalfExtents) override final { FinitePlaneHalfExtents = bFinitePlane ? FVector2D(InHalfExtents.X, InHalfExtents.Y) : FVector2D::ZeroVector; }
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