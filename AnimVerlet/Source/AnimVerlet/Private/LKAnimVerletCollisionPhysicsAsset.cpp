#include "LKAnimVerletCollisionPhysicsAsset.h"

#include <DrawDebugHelpers.h>
#if (ENGINE_MINOR_VERSION >= 5)
#include <PhysicsEngine/SkeletalBodySetup.h>
#endif

///=========================================================================================================================================
/// FLKAnimVerletCollisionPAShape
///=========================================================================================================================================
bool FLKAnimVerletCollisionPAShape::IsValidPAShape() const
{
	if (BodySetupIndex == INDEX_NONE || ColliderListIndex == INDEX_NONE)
		return false;

	const UPhysicsAsset* CurPhysicsAsset = CollisionPhysicsAsset.Get();
	if (CurPhysicsAsset == nullptr)
		return false;

	const TObjectPtr<USkeletalBodySetup>& BodySetup = CurPhysicsAsset->SkeletalBodySetups[BodySetupIndex];
	if (BodySetup == nullptr)
		return false;

	return true;
}

FKAggregateGeom* FLKAnimVerletCollisionPAShape::GetGeometry() const
{
	if (IsValidPAShape() == false)
		return nullptr;

	UPhysicsAsset* CurPhysicsAsset = CollisionPhysicsAsset.Get();
	TObjectPtr<USkeletalBodySetup>& BodySetup = CurPhysicsAsset->SkeletalBodySetups[BodySetupIndex];
	return &BodySetup->AggGeom;
}

FName FLKAnimVerletCollisionPAShape::GetAttachBoneName() const 
{ 
	if (IsValidPAShape() == false)
		return NAME_None;

	const UPhysicsAsset* CurPhysicsAsset = CollisionPhysicsAsset.Get();
	const TObjectPtr<USkeletalBodySetup>& BodySetup = CurPhysicsAsset->SkeletalBodySetups[BodySetupIndex];
	return BodySetup->BoneName;
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionPASphere
///=========================================================================================================================================
FVector FLKAnimVerletCollisionPASphere::GetLocation() const 
{ 
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FVector::ZeroVector;

	if (CurGeometry->SphereElems.IsValidIndex(ColliderListIndex) == false)
		return FVector::ZeroVector;

	const FKSphereElem& SphereElem = CurGeometry->SphereElems[ColliderListIndex];
	return SphereElem.Center;
}

void FLKAnimVerletCollisionPASphere::SetLocation(const FVector& InLocation) 
{ 
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->SphereElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKSphereElem& SphereElem = CurGeometry->SphereElems[ColliderListIndex];
	SphereElem.Center = InLocation;
}

FVector FLKAnimVerletCollisionPASphere::GetHalfExtents() const 
{ 
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FVector::ZeroVector;

	if (CurGeometry->SphereElems.IsValidIndex(ColliderListIndex) == false)
		return FVector::ZeroVector;

	const FKSphereElem& SphereElem = CurGeometry->SphereElems[ColliderListIndex];
	return FVector(SphereElem.Radius, SphereElem.Radius, SphereElem.Radius);
}

void FLKAnimVerletCollisionPASphere::SetHalfExtents(const FVector& InHalfExtents) 
{ 
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->SphereElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKSphereElem& SphereElem = CurGeometry->SphereElems[ColliderListIndex];
	SphereElem.Radius = InHalfExtents.X;
}

void FLKAnimVerletCollisionPASphere::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(GetAttachBoneName(), ERelativeTransformSpace::RTS_World);
		DrawDebugSphere(InWorld, WorldT.GetLocation(), GetHalfExtents().X, 16, FColor::Blue, false, LifeTime);
	}
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionPACapsule
///=========================================================================================================================================
FVector FLKAnimVerletCollisionPACapsule::GetLocation() const
{
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FVector::ZeroVector;

	if (CurGeometry->SphylElems.IsValidIndex(ColliderListIndex) == false)
		return FVector::ZeroVector;

	const FKSphylElem& CapsuleElem = CurGeometry->SphylElems[ColliderListIndex];
	return CapsuleElem.Center;
}

void FLKAnimVerletCollisionPACapsule::SetLocation(const FVector& InLocation)
{
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->SphylElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKSphylElem& CapsuleElem = CurGeometry->SphylElems[ColliderListIndex];
	CapsuleElem.Center = InLocation;
}

FRotator FLKAnimVerletCollisionPACapsule::GetRotation() const 
{
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FRotator::ZeroRotator;

	if (CurGeometry->SphylElems.IsValidIndex(ColliderListIndex) == false)
		return FRotator::ZeroRotator;

	const FKSphylElem& CapsuleElem = CurGeometry->SphylElems[ColliderListIndex];
	return CapsuleElem.Rotation;
}

void FLKAnimVerletCollisionPACapsule::SetRotation(const FRotator& InRotator) 
{ 
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->SphylElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKSphylElem& CapsuleElem = CurGeometry->SphylElems[ColliderListIndex];
	CapsuleElem.Rotation = InRotator; 
}

FVector FLKAnimVerletCollisionPACapsule::GetHalfExtents() const 
{ 
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FVector::ZeroVector;

	if (CurGeometry->SphylElems.IsValidIndex(ColliderListIndex) == false)
		return FVector::ZeroVector;

	const FKSphylElem& CapsuleElem = CurGeometry->SphylElems[ColliderListIndex];
	return FVector(CapsuleElem.Radius, CapsuleElem.Radius, (CapsuleElem.Length * 0.5f));
}

void FLKAnimVerletCollisionPACapsule::SetHalfExtents(const FVector& InHalfExtents) 
{ 
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->SphylElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKSphylElem& CapsuleElem = CurGeometry->SphylElems[ColliderListIndex];
	CapsuleElem.Radius = InHalfExtents.X;
	CapsuleElem.Length = (InHalfExtents.Z * 2.0f);
}

void FLKAnimVerletCollisionPACapsule::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(GetAttachBoneName(), ERelativeTransformSpace::RTS_World);
		const FVector HalfExtents = GetHalfExtents();
		DrawDebugCapsule(InWorld, WorldT.GetLocation(), HalfExtents.Z + HalfExtents.X, HalfExtents.X, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
	}
}


///=========================================================================================================================================
/// FLKAnimVerletCollisionPABox
///=========================================================================================================================================
FVector FLKAnimVerletCollisionPABox::GetLocation() const
{
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FVector::ZeroVector;

	if (CurGeometry->BoxElems.IsValidIndex(ColliderListIndex) == false)
		return FVector::ZeroVector;

	const FKBoxElem& BoxElem = CurGeometry->BoxElems[ColliderListIndex];
	return BoxElem.Center;
}

void FLKAnimVerletCollisionPABox::SetLocation(const FVector& InLocation)
{
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->BoxElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKBoxElem& BoxElem = CurGeometry->BoxElems[ColliderListIndex];
	BoxElem.Center = InLocation;
}

FRotator FLKAnimVerletCollisionPABox::GetRotation() const 
{ 
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FRotator::ZeroRotator;

	if (CurGeometry->BoxElems.IsValidIndex(ColliderListIndex) == false)
		return FRotator::ZeroRotator;

	const FKBoxElem& BoxElem = CurGeometry->BoxElems[ColliderListIndex];
	return BoxElem.Rotation;
}

void FLKAnimVerletCollisionPABox::SetRotation(const FRotator& InRotator) 
{ 
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->BoxElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKBoxElem& BoxElem = CurGeometry->BoxElems[ColliderListIndex];
	BoxElem.Rotation = InRotator;
}

FVector FLKAnimVerletCollisionPABox::GetHalfExtents() const 
{ 
	const FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return FVector::ZeroVector;

	if (CurGeometry->BoxElems.IsValidIndex(ColliderListIndex) == false)
		return FVector::ZeroVector;

	const FKBoxElem& BoxElem = CurGeometry->BoxElems[ColliderListIndex];
	return FVector(BoxElem.X, BoxElem.Y, BoxElem.Z) * 0.5f;
}

void FLKAnimVerletCollisionPABox::SetHalfExtents(const FVector& InHalfExtents) 
{ 
	FKAggregateGeom* CurGeometry = GetGeometry();
	if (CurGeometry == nullptr)
		return;

	if (CurGeometry->BoxElems.IsValidIndex(ColliderListIndex) == false)
		return;

	FKBoxElem& BoxElem = CurGeometry->BoxElems[ColliderListIndex];
	BoxElem.X = InHalfExtents.X * 0.5f;
	BoxElem.Y = InHalfExtents.Y * 0.5f;
	BoxElem.Z = InHalfExtents.Z * 0.5f;
}

void FLKAnimVerletCollisionPABox::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(GetAttachBoneName(), ERelativeTransformSpace::RTS_World);
		DrawDebugBox(InWorld, WorldT.GetLocation(), GetHalfExtents(), WorldT.GetRotation(), FColor::Blue, false, LifeTime);
	}
}