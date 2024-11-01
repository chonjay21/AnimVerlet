#include "LKAnimVerletCollisionData.h"

#include <DrawDebugHelpers.h>
#include "LKAnimVerletCollisionShape.h"

///=========================================================================================================================================
/// FLKAnimVerletCollisionData(Base Interface)
///=========================================================================================================================================
void FLKAnimVerletCollisionData::ConvertToShapeBase(OUT FLKAnimVerletCollisionShape& OutShapeBase) const
{
	OutShapeBase.bUseAbsoluteWorldTransform = bUseAbsoluteWorldTransform;
	if (bUseAbsoluteWorldTransform == false)
		OutShapeBase.AttachedBone.BoneName = AttachBoneName;
	
	OutShapeBase.ExcludeBones.SetNum(ExcludeBones.Num());
	for (int32 i = 0; i < ExcludeBones.Num(); ++i)
	{
		const FName& CurBoneName = ExcludeBones[i];
		OutShapeBase.ExcludeBones[i].BoneName = CurBoneName;
	}
	OutShapeBase.LocationOffset = LocationOffset;
}

void FLKAnimVerletCollisionData::ConvertFromShapeBase(const FLKAnimVerletCollisionShape& InShapeBase)
{
	bUseAbsoluteWorldTransform = InShapeBase.bUseAbsoluteWorldTransform;
	if (bUseAbsoluteWorldTransform == false)
		AttachBoneName = InShapeBase.AttachedBone.BoneName;

	ExcludeBones.SetNum(InShapeBase.ExcludeBones.Num());
	for (int32 i = 0; i < InShapeBase.ExcludeBones.Num(); ++i)
	{
		const FName& CurBoneName = InShapeBase.ExcludeBones[i].BoneName;
		ExcludeBones[i] = CurBoneName;
	}
	LocationOffset = InShapeBase.LocationOffset;
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataSphere
///=========================================================================================================================================
void FLKAnimVerletCollisionDataSphere::ConvertToShape(OUT FLKAnimVerletCollisionSphere& OutSphere) const
{
	ConvertToShapeBase(OUT OutSphere);

	OutSphere.Radius = Radius;
}

void FLKAnimVerletCollisionDataSphere::ConvertFromShape(const FLKAnimVerletCollisionSphere& InSphere)
{
	ConvertFromShapeBase(InSphere);

	Radius = InSphere.Radius;
}

void FLKAnimVerletCollisionDataSphere::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		DrawDebugSphere(InWorld, GetLocation(), Radius, 16, FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachBoneName, ERelativeTransformSpace::RTS_World);
		DrawDebugSphere(InWorld, WorldT.GetLocation(), Radius, 16, FColor::Blue, false, LifeTime);
	}
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataCapsule
///=========================================================================================================================================
void FLKAnimVerletCollisionDataCapsule::ConvertToShape(OUT FLKAnimVerletCollisionCapsule& OutCapsule) const
{
	ConvertToShapeBase(OUT OutCapsule);

	OutCapsule.RotationOffset = RotationOffset;
	OutCapsule.Radius = Radius;
	OutCapsule.HalfHeight = HalfHeight;
}

void FLKAnimVerletCollisionDataCapsule::ConvertFromShape(const FLKAnimVerletCollisionCapsule& InCapsule)
{
	ConvertFromShapeBase(InCapsule);

	RotationOffset = InCapsule.RotationOffset;
	Radius = InCapsule.Radius;
	HalfHeight = InCapsule.HalfHeight;
}

void FLKAnimVerletCollisionDataCapsule::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		DrawDebugCapsule(InWorld, GetLocation(), HalfHeight + Radius, Radius, GetRotation().Quaternion(), FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachBoneName, ERelativeTransformSpace::RTS_World);
		DrawDebugCapsule(InWorld, WorldT.GetLocation(), HalfHeight + Radius, Radius, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
	}
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataBox
///=========================================================================================================================================
void FLKAnimVerletCollisionDataBox::ConvertToShape(OUT FLKAnimVerletCollisionBox& OutBox) const
{
	ConvertToShapeBase(OUT OutBox);

	OutBox.RotationOffset = RotationOffset;
	OutBox.HalfExtents = HalfExtents;
}

void FLKAnimVerletCollisionDataBox::ConvertFromShape(const FLKAnimVerletCollisionBox& InBox)
{
	ConvertFromShapeBase(InBox);

	RotationOffset = InBox.RotationOffset;
	HalfExtents = InBox.HalfExtents;
}

void FLKAnimVerletCollisionDataBox::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		DrawDebugBox(InWorld, GetLocation(), HalfExtents, GetRotation().Quaternion(), FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachBoneName, ERelativeTransformSpace::RTS_World);
		DrawDebugBox(InWorld, WorldT.GetLocation(), HalfExtents, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
	}
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataPlane
///=========================================================================================================================================
void FLKAnimVerletCollisionDataPlane::ConvertToShape(OUT FLKAnimVerletCollisionPlane& OutPlane) const
{
	ConvertToShapeBase(OUT OutPlane);

	OutPlane.RotationOffset = RotationOffset;
	OutPlane.bFinitePlane = bFinitePlane;
	OutPlane.FinitePlaneHalfExtents = FinitePlaneHalfExtents;
}

void FLKAnimVerletCollisionDataPlane::ConvertFromShape(const FLKAnimVerletCollisionPlane& InPlane)
{
	ConvertFromShapeBase(InPlane);

	RotationOffset = InPlane.RotationOffset;
	bFinitePlane = InPlane.bFinitePlane;
	FinitePlaneHalfExtents = InPlane.FinitePlaneHalfExtents;
}

void FLKAnimVerletCollisionDataPlane::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	if (IsUseAbsoluteWorldTransform())
	{
		const FVector HalfExtents3D = (FinitePlaneHalfExtents.IsNearlyZero() == false) ? FVector(FinitePlaneHalfExtents, 1.0f) : FVector(100.0f, 100.0f, 1.0f);
		DrawDebugBox(InWorld, GetLocation(), HalfExtents3D, GetRotation().Quaternion(), FColor::Blue, false, LifeTime);
		DrawDebugDirectionalArrow(InWorld, GetLocation(), GetLocation() + GetRotation().Quaternion().GetAxisZ() * 30.f, 10.0f, FColor::Blue, false, LifeTime);
	}
	else if (InMeshNullable != nullptr)
	{
		const FTransform WorldT = GetTransform() * InMeshNullable->GetSocketTransform(AttachBoneName, ERelativeTransformSpace::RTS_World);

		const FVector HalfExtents3D = (FinitePlaneHalfExtents.IsNearlyZero() == false) ? FVector(FinitePlaneHalfExtents, 1.0f) : FVector(100.0f, 100.0f, 1.0f);
		DrawDebugBox(InWorld, WorldT.GetLocation(), HalfExtents3D, WorldT.GetRotation(), FColor::Blue, false, LifeTime);
		DrawDebugDirectionalArrow(InWorld, WorldT.GetLocation(), WorldT.GetLocation() + WorldT.GetRotation().GetAxisZ() * 30.f, 10.0f, FColor::Blue, false, LifeTime);
	}
}

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataList
///=========================================================================================================================================
void FLKAnimVerletCollisionDataList::ConvertToShape(OUT FLKAnimVerletCollisionShapeList& OutShapeList) const
{
	OutShapeList.SphereCollisionShapes.Reserve(OutShapeList.SphereCollisionShapes.Num() + SphereCollisionData.Num());
	for (const FLKAnimVerletCollisionDataSphere& CurData : SphereCollisionData)
	{
		FLKAnimVerletCollisionSphere& SphereShape = OutShapeList.SphereCollisionShapes.Emplace_GetRef();
		CurData.ConvertToShape(OUT SphereShape);
	}

	OutShapeList.CapsuleCollisionShapes.Reserve(OutShapeList.CapsuleCollisionShapes.Num() + CapsuleCollisionData.Num());
	for (const FLKAnimVerletCollisionDataCapsule& CurData : CapsuleCollisionData)
	{
		FLKAnimVerletCollisionCapsule& CapsuleShape = OutShapeList.CapsuleCollisionShapes.Emplace_GetRef();
		CurData.ConvertToShape(OUT CapsuleShape);
	}

	OutShapeList.BoxCollisionShapes.Reserve(OutShapeList.BoxCollisionShapes.Num() + BoxCollisionData.Num());
	for (const FLKAnimVerletCollisionDataBox& CurData : BoxCollisionData)
	{
		FLKAnimVerletCollisionBox& BoxShape = OutShapeList.BoxCollisionShapes.Emplace_GetRef();
		CurData.ConvertToShape(OUT BoxShape);
	}

	OutShapeList.PlaneCollisionShapes.Reserve(OutShapeList.PlaneCollisionShapes.Num() + PlaneCollisionData.Num());
	for (const FLKAnimVerletCollisionDataPlane& CurData : PlaneCollisionData)
	{
		FLKAnimVerletCollisionPlane& PlaneShape = OutShapeList.PlaneCollisionShapes.Emplace_GetRef();
		CurData.ConvertToShape(OUT PlaneShape);
	}
}

void FLKAnimVerletCollisionDataList::ConvertFromShape(const FLKAnimVerletCollisionShapeList& InShapeList)
{
	SphereCollisionData.Reserve(SphereCollisionData.Num() + InShapeList.SphereCollisionShapes.Num());
	for (const FLKAnimVerletCollisionSphere& CurData : InShapeList.SphereCollisionShapes)
	{
		FLKAnimVerletCollisionDataSphere& SphereShape = SphereCollisionData.Emplace_GetRef();
		SphereShape.ConvertFromShape(CurData);
	}

	CapsuleCollisionData.Reserve(CapsuleCollisionData.Num() + InShapeList.CapsuleCollisionShapes.Num());
	for (const FLKAnimVerletCollisionCapsule& CurData : InShapeList.CapsuleCollisionShapes)
	{
		FLKAnimVerletCollisionDataCapsule& CapsuleShape = CapsuleCollisionData.Emplace_GetRef();
		CapsuleShape.ConvertFromShape(CurData);
	}

	BoxCollisionData.Reserve(BoxCollisionData.Num() + InShapeList.BoxCollisionShapes.Num());
	for (const FLKAnimVerletCollisionBox& CurData : InShapeList.BoxCollisionShapes)
	{
		FLKAnimVerletCollisionDataBox& BoxShape = BoxCollisionData.Emplace_GetRef();
		BoxShape.ConvertFromShape(CurData);
	}

	PlaneCollisionData.Reserve(PlaneCollisionData.Num() + InShapeList.PlaneCollisionShapes.Num());
	for (const FLKAnimVerletCollisionPlane& CurData : InShapeList.PlaneCollisionShapes)
	{
		FLKAnimVerletCollisionDataPlane& PlaneShape = PlaneCollisionData.Emplace_GetRef();
		PlaneShape.ConvertFromShape(CurData);
	}
}

bool FLKAnimVerletCollisionDataList::AddData(const FLKAnimVerletCollisionData& NewData)
{
	const ELKAnimVerletCollider NewColliderType = NewData.GetColliderType();
	switch (NewColliderType)
	{
		case ELKAnimVerletCollider::Sphere:
		{
			const FLKAnimVerletCollisionDataSphere* SphereData = static_cast<const FLKAnimVerletCollisionDataSphere*>(&NewData);
			SphereCollisionData.Emplace(*SphereData);
			return true;
		}

		case ELKAnimVerletCollider::Capsule:
		{
			const FLKAnimVerletCollisionDataCapsule* CapsuleData = static_cast<const FLKAnimVerletCollisionDataCapsule*>(&NewData);
			CapsuleCollisionData.Emplace(*CapsuleData);
			return true;
		}

		case ELKAnimVerletCollider::Box:
		{
			const FLKAnimVerletCollisionDataBox* BoxData = static_cast<const FLKAnimVerletCollisionDataBox*>(&NewData);
			BoxCollisionData.Emplace(*BoxData);
			return true;
		}

		case ELKAnimVerletCollider::Plane:
		{
			const FLKAnimVerletCollisionDataPlane* PlaneData = static_cast<const FLKAnimVerletCollisionDataPlane*>(&NewData);
			PlaneCollisionData.Emplace(*PlaneData);
			return true;
		}

		default:
			break;
	}
	return false;
}

void FLKAnimVerletCollisionDataList::Reset()
{
	SphereCollisionData.Reset();
	CapsuleCollisionData.Reset();
	BoxCollisionData.Reset();
	PlaneCollisionData.Reset();
}

void FLKAnimVerletCollisionDataList::DebugDrawCollider(const UWorld* InWorld, const USkeletalMeshComponent* InMeshNullable, float LifeTime) const
{
	ForEachCollisionDataConst([InWorld, InMeshNullable, LifeTime](const FLKAnimVerletColliderInterface& CurCollider) {
		CurCollider.DebugDrawCollider(InWorld, InMeshNullable, LifeTime);
	});
}

///=========================================================================================================================================
/// ULKAnimVerletCollisionDataAsset
///=========================================================================================================================================
void ULKAnimVerletCollisionDataAsset::ConvertToShape(OUT FLKAnimVerletCollisionShapeList& OutShapeList) const
{
	CollisionDataList.ConvertToShape(OUT OutShapeList);
}

void ULKAnimVerletCollisionDataAsset::ConvertFromShape(const FLKAnimVerletCollisionShapeList& InShapeList)
{
	CollisionDataList.ConvertFromShape(InShapeList);
}

void ULKAnimVerletCollisionDataAsset::Reset()
{
	CollisionDataList.Reset();
}