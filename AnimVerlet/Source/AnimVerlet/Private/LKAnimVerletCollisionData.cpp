#include "LKAnimVerletCollisionData.h"

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

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataSphere
///=========================================================================================================================================
void FLKAnimVerletCollisionDataSphere::ConvertToShape(OUT FLKAnimVerletCollisionSphere& OutSphere) const
{
	ConvertToShapeBase(OUT OutSphere);

	OutSphere.Radius = Radius;
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

///=========================================================================================================================================
/// FLKAnimVerletCollisionDataBox
///=========================================================================================================================================
void FLKAnimVerletCollisionDataBox::ConvertToShape(OUT FLKAnimVerletCollisionBox& OutBox) const
{
	ConvertToShapeBase(OUT OutBox);

	OutBox.RotationOffset = RotationOffset;
	OutBox.HalfExtents = HalfExtents;
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

///=========================================================================================================================================
/// ULKAnimVerletCollisionDataAsset
///=========================================================================================================================================
void ULKAnimVerletCollisionDataAsset::ConvertToShape(OUT FLKAnimVerletCollisionShapeList& OutShapeList) const
{
	CollisionDataList.ConvertToShape(OUT OutShapeList);
}