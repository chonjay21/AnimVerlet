/**
	PhysicsAsset`s Sphape wrapper
*/

#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletConstraint.h"
#include "LKAnimVerletCollisionShapeType.h"
#include "LKAnimVerletCollisionPhysicsAsset.generated.h"

///=========================================================================================================================================
/// FLKAnimVerletCollisionPAShape(Base Interface)
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionPAShape : public FLKAnimVerletColliderInterface
{
	GENERATED_BODY()

public:
	FLKAnimVerletCollisionPAShape() = default;
	FLKAnimVerletCollisionPAShape(class UPhysicsAsset* InCollisionPhysicsAsset, int32 InBodySetupIndex, int32 InColliderListIndex) : CollisionPhysicsAsset(InCollisionPhysicsAsset), BodySetupIndex(InBodySetupIndex), ColliderListIndex(InColliderListIndex) {}
	virtual ~FLKAnimVerletCollisionPAShape() {}

	virtual ELKAnimVerletCollider GetColliderType() const override { return ELKAnimVerletCollider::None; }
	virtual bool IsUseAbsoluteWorldTransform() const override final { return false; }
	virtual FName GetAttachBoneName() const override final;

	virtual bool IsSphere() const { return false; }
	virtual bool IsCapsule() const { return false; }
	virtual bool IsBox() const { return false; }
	virtual bool IsPlane() const { return false; }

public:
	bool IsValidPAShape() const;
	struct FKAggregateGeom* GetGeometry() const;

protected:
	TWeakObjectPtr<class UPhysicsAsset> CollisionPhysicsAsset = nullptr;
	int32 BodySetupIndex = INDEX_NONE;
	int32 ColliderListIndex = INDEX_NONE;
};
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletCollisionPASphere
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionPASphere : public FLKAnimVerletCollisionPAShape
{
	GENERATED_BODY()

public:
	FLKAnimVerletCollisionPASphere() = default;
	FLKAnimVerletCollisionPASphere(class UPhysicsAsset* InCollisionPhysicsAsset, int32 InBodySetupIndex, int32 InColliderListIndex) : FLKAnimVerletCollisionPAShape(InCollisionPhysicsAsset, InBodySetupIndex, InColliderListIndex) {}

	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Sphere; }
	virtual bool IsSphere() const override final { return true; }

	virtual FVector GetLocation() const override final;
	virtual void SetLocation(const FVector& InLocation) override final;

	virtual FVector GetHalfExtents() const override final;
	virtual void SetHalfExtents(const FVector& InHalfExtents);

	/// Thread unsafe
	virtual void DebugDrawCollider(const class UWorld* InWorld, const class USkeletalMeshComponent* InMeshNullable, float LifeTime = -1.0f) const override;
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionPACapsule
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionPACapsule : public FLKAnimVerletCollisionPAShape
{
	GENERATED_BODY()

public:
	FLKAnimVerletCollisionPACapsule() = default;
	FLKAnimVerletCollisionPACapsule(class UPhysicsAsset* InCollisionPhysicsAsset, int32 InBodySetupIndex, int32 InColliderListIndex) : FLKAnimVerletCollisionPAShape(InCollisionPhysicsAsset, InBodySetupIndex, InColliderListIndex) {}

	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Capsule; }
	virtual bool IsCapsule() const override final { return true; }

	virtual FVector GetLocation() const override final;
	virtual void SetLocation(const FVector& InLocation) override final;

	virtual FRotator GetRotation() const override final;
	virtual void SetRotation(const FRotator& InRotator) override;

	virtual FVector GetHalfExtents() const override final;
	virtual void SetHalfExtents(const FVector& InHalfExtents) override final;

	/// Thread unsafe
	virtual void DebugDrawCollider(const class UWorld* InWorld, const class USkeletalMeshComponent* InMeshNullable, float LifeTime = -1.0f) const override;
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletCollisionPABox
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct ANIMVERLET_API FLKAnimVerletCollisionPABox : public FLKAnimVerletCollisionPAShape
{
	GENERATED_BODY()

public:
	FLKAnimVerletCollisionPABox() = default;
	FLKAnimVerletCollisionPABox(class UPhysicsAsset* InCollisionPhysicsAsset, int32 InBodySetupIndex, int32 InColliderListIndex) : FLKAnimVerletCollisionPAShape(InCollisionPhysicsAsset, InBodySetupIndex, InColliderListIndex) {}

	virtual ELKAnimVerletCollider GetColliderType() const override final { return ELKAnimVerletCollider::Box; }
	virtual bool IsBox() const override final { return true; }

	virtual FVector GetLocation() const override final;
	virtual void SetLocation(const FVector& InLocation) override final;

	virtual FRotator GetRotation() const override final;
	virtual void SetRotation(const FRotator& InRotator) override;

	virtual FVector GetHalfExtents() const override final;
	virtual void SetHalfExtents(const FVector& InHalfExtents) override final;

	/// Thread unsafe
	virtual void DebugDrawCollider(const class UWorld* InWorld, const class USkeletalMeshComponent* InMeshNullable, float LifeTime = -1.0f) const override;
};
///=========================================================================================================================================
