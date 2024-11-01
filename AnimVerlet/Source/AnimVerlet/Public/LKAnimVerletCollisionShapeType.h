#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletCollisionShapeType.generated.h"

UENUM()
enum class ELKAnimVerletCollider : uint8
{
	None,

	Sphere,
	Capsule,
	Box,
	Plane,

	Count
};


///=========================================================================================================================================
/// FLKAnimVerletColliderInterface(Base Interface)
///=========================================================================================================================================
USTRUCT(BlueprintType)
struct FLKAnimVerletColliderInterface
{
	GENERATED_BODY()

public:
	virtual ~FLKAnimVerletColliderInterface() {}

	virtual ELKAnimVerletCollider GetColliderType() const PURE_VIRTUAL(, return ELKAnimVerletCollider::None;);
	virtual bool IsUseAbsoluteWorldTransform() const PURE_VIRTUAL(, return false;);
	virtual FName GetAttachBoneName() const PURE_VIRTUAL(, return NAME_None;);
	virtual FVector GetHalfExtents() const PURE_VIRTUAL(, return FVector::ZeroVector;);
	virtual void SetHalfExtents(const FVector& InHalfExtents) PURE_VIRTUAL(, ;);

	virtual FVector GetLocation() const { return FVector::ZeroVector; }
	virtual FRotator GetRotation() const { return FRotator::ZeroRotator; }
	virtual FVector GetScale() const { return FVector::OneVector; }
	virtual FTransform GetTransform() const { return FTransform(GetRotation(), GetLocation(), GetScale()); }
	virtual void SetLocation(const FVector& InLocation) {}
	virtual void SetRotation(const FRotator& InRotator) {}
	virtual void SetScale(const FVector& InScale) {}
	virtual void SetTransform(const FTransform& InTransform) { SetLocation(InTransform.GetLocation()); SetRotation(InTransform.Rotator()); SetScale(InTransform.GetScale3D()); }

	virtual void DebugDrawCollider(const class UWorld* InWorld, const class USkeletalMeshComponent* InMeshNullable, float LifeTime = -1.0f) const {}
};
///=========================================================================================================================================