#include "LKAnimVerletCollisionShape.h"

#include <DrawDebugHelpers.h>

///=========================================================================================================================================
/// FLKAnimVerletCollisionSphere
///=========================================================================================================================================
void FLKAnimVerletCollisionSphere::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		DrawDebugSphere(InWorld, GetLocation(), Radius, 16, FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachedBone.BoneName, ERelativeTransformSpace::RTS_World);
		DrawDebugSphere(InWorld, WorldT.GetLocation(), Radius, 16, FColor::Blue, false, LifeTime);
	}
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionCapsule
///=========================================================================================================================================
void FLKAnimVerletCollisionCapsule::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		DrawDebugCapsule(InWorld, GetLocation(), HalfHeight + Radius, Radius, GetRotation().Quaternion(), FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachedBone.BoneName, ERelativeTransformSpace::RTS_World);
		DrawDebugCapsule(InWorld, WorldT.GetLocation(), HalfHeight + Radius, Radius, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
	}
}


///=========================================================================================================================================
/// FLKAnimVerletCollisionBox
///=========================================================================================================================================
void FLKAnimVerletCollisionBox::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		DrawDebugBox(InWorld, GetLocation(), HalfExtents, GetRotation().Quaternion(), FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachedBone.BoneName, ERelativeTransformSpace::RTS_World);
		DrawDebugBox(InWorld, WorldT.GetLocation(), HalfExtents, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
	}
}


///=========================================================================================================================================
/// FLKAnimVerletCollisionPlane
///=========================================================================================================================================
void FLKAnimVerletCollisionPlane::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		const FVector HalfExtents3D = (FinitePlaneHalfExtents.IsNearlyZero() == false) ? FVector(FinitePlaneHalfExtents, 1.0f) : FVector(100.0f, 100.0f, 1.0f);
		DrawDebugBox(InWorld, GetLocation(), HalfExtents3D, GetRotation().Quaternion(), FColor::Blue, false, LifeTime);
		DrawDebugDirectionalArrow(InWorld, GetLocation(), GetLocation() + GetRotation().Quaternion().GetAxisZ() * 30.f, 10.0f, FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachedBone.BoneName, ERelativeTransformSpace::RTS_World);

		const FVector HalfExtents3D = (FinitePlaneHalfExtents.IsNearlyZero() == false) ? FVector(FinitePlaneHalfExtents, 1.0f) : FVector(100.0f, 100.0f, 1.0f);
		DrawDebugBox(InWorld, WorldT.GetLocation(), HalfExtents3D, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
		DrawDebugDirectionalArrow(InWorld, WorldT.GetLocation(), WorldT.GetLocation() + WorldT.GetRotation().GetAxisZ() * 30.f, 10.0f, FColor::Blue, false, LifeTime);
	}
}


///=========================================================================================================================================
/// FLKAnimVerletCollisionShapeList
///=========================================================================================================================================
bool FLKAnimVerletCollisionShapeList::AddShape(const FLKAnimVerletCollisionShape& NewShape)
{
	const ELKAnimVerletCollider NewColliderType = NewShape.GetColliderType();
	switch (NewColliderType)
	{
		case ELKAnimVerletCollider::Sphere: 
		{
			const FLKAnimVerletCollisionSphere* SphereShape = static_cast<const FLKAnimVerletCollisionSphere*>(&NewShape); 
			SphereCollisionShapes.Emplace(*SphereShape);
			return true; 
		}

		case ELKAnimVerletCollider::Capsule:
		{
			const FLKAnimVerletCollisionCapsule* CapsuleShape = static_cast<const FLKAnimVerletCollisionCapsule*>(&NewShape);
			CapsuleCollisionShapes.Emplace(*CapsuleShape);
			return true;
		}

		case ELKAnimVerletCollider::Box:
		{
			const FLKAnimVerletCollisionBox* BoxShape = static_cast<const FLKAnimVerletCollisionBox*>(&NewShape);
			BoxCollisionShapes.Emplace(*BoxShape);
			return true;
		}

		case ELKAnimVerletCollider::Plane:
		{
			const FLKAnimVerletCollisionPlane* PlaneShape = static_cast<const FLKAnimVerletCollisionPlane*>(&NewShape);
			PlaneCollisionShapes.Emplace(*PlaneShape);
			return true;
		}

		default:
			break;
	}
	return false;
}

void FLKAnimVerletCollisionShapeList::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	ForEachCollisionShapeConst([InWorld, InMeshNullable, LifeTime](const FLKAnimVerletColliderInterface& CurCollider) {
		CurCollider.DebugDrawCollider(InWorld, InMeshNullable, LifeTime);
	});
}