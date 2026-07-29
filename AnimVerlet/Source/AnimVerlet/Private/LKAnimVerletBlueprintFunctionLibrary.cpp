#include "LKAnimVerletBlueprintFunctionLibrary.h"

#include <Components/PrimitiveComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <PhysicsEngine/AggregateGeom.h>
#include <PhysicsEngine/BodyInstance.h>
#include <PhysicsEngine/BodySetup.h>
#include <PhysicsEngine/BoxElem.h>
#include <PhysicsEngine/PhysicsAsset.h>
#include <PhysicsEngine/SkeletalBodySetup.h>
#include <PhysicsEngine/SphereElem.h>
#include <PhysicsEngine/SphylElem.h>
#include "LKAnimVerletCollisionData.h"

namespace LkAnimVerlet
{
	FTransform GetBodyWorldTransform(const FBodyInstance* BodyInstance, const FTransform& FallbackWorldTransform)
	{
		FTransform BodyWorldTransform = FallbackWorldTransform;
		if (BodyInstance != nullptr && BodyInstance->IsValidBodyInstance())
			BodyWorldTransform = BodyInstance->GetUnrealWorldTransform();

		/// Shape dimensions are scaled separately via GetFinalScaled.
		BodyWorldTransform.SetScale3D(FVector::OneVector);
		return BodyWorldTransform;
	}

	void AddBodySetupShapes(FLKAnimVerletCollisionShapeList& OutShapeList, const UBodySetup& BodySetup, const FVector& BodyScale, const FTransform& BodyWorldTransform)
	{
		const FKAggregateGeom& AggregateGeometry = BodySetup.AggGeom;

		OutShapeList.SphereCollisionShapes.Reserve(OutShapeList.SphereCollisionShapes.Num() + AggregateGeometry.SphereElems.Num());
		for (const FKSphereElem& SphereElem : AggregateGeometry.SphereElems)
		{
			const FKSphereElem ScaledSphereElem = SphereElem.GetFinalScaled(BodyScale, FTransform::Identity);

			FLKAnimVerletCollisionSphere& SphereShape = OutShapeList.SphereCollisionShapes.Emplace_GetRef();
			{
				SphereShape.bUseAbsoluteWorldTransform = true;
				SphereShape.LocationOffset = BodyWorldTransform.TransformPosition(ScaledSphereElem.Center);
				SphereShape.Radius = ScaledSphereElem.Radius;
			}
		}

		OutShapeList.CapsuleCollisionShapes.Reserve(OutShapeList.CapsuleCollisionShapes.Num() + AggregateGeometry.SphylElems.Num());
		for (const FKSphylElem& CapsuleElem : AggregateGeometry.SphylElems)
		{
			const FKSphylElem ScaledCapsuleElem = CapsuleElem.GetFinalScaled(BodyScale, FTransform::Identity);
			const FTransform CapsuleWorldTransform = ScaledCapsuleElem.GetTransform() * BodyWorldTransform;

			FLKAnimVerletCollisionCapsule& CapsuleShape = OutShapeList.CapsuleCollisionShapes.Emplace_GetRef();
			{
				CapsuleShape.bUseAbsoluteWorldTransform = true;
				CapsuleShape.LocationOffset = CapsuleWorldTransform.GetLocation();
				CapsuleShape.RotationOffset = CapsuleWorldTransform.Rotator();
				CapsuleShape.Radius = ScaledCapsuleElem.Radius;
				CapsuleShape.HalfHeight = ScaledCapsuleElem.Length * 0.5f;
			}
		}

		OutShapeList.BoxCollisionShapes.Reserve(OutShapeList.BoxCollisionShapes.Num() + AggregateGeometry.BoxElems.Num());
		for (const FKBoxElem& BoxElem : AggregateGeometry.BoxElems)
		{
			const FKBoxElem ScaledBoxElem = BoxElem.GetFinalScaled(BodyScale, FTransform::Identity);
			const FTransform BoxWorldTransform = ScaledBoxElem.GetTransform() * BodyWorldTransform;

			FLKAnimVerletCollisionBox& BoxShape = OutShapeList.BoxCollisionShapes.Emplace_GetRef();
			{
				BoxShape.bUseAbsoluteWorldTransform = true;
				BoxShape.LocationOffset = BoxWorldTransform.GetLocation();
				BoxShape.RotationOffset = BoxWorldTransform.Rotator();
				BoxShape.HalfExtents = FVector(ScaledBoxElem.X, ScaledBoxElem.Y, ScaledBoxElem.Z) * 0.5f;
			}
		}
	}

	bool GetBoneWorldTransform(FTransform& OutBoneWorldTransform, FVector& OutBoneWorldScale, const USkeletalMeshComponent& SkeletalMeshComponent, const FName& BoneName)
	{
		if (BoneName == NAME_None || SkeletalMeshComponent.GetBoneIndex(BoneName) == INDEX_NONE)
			return false;

		OutBoneWorldTransform = SkeletalMeshComponent.GetSocketTransform(BoneName, ERelativeTransformSpace::RTS_World);
		OutBoneWorldScale = OutBoneWorldTransform.GetScale3D();
		OutBoneWorldTransform.SetScale3D(FVector::OneVector);
		return true;
	}

	void ConvertShapeListToWorld(FLKAnimVerletCollisionShapeList& OutShapeList, const USkeletalMeshComponent& SkeletalMeshComponent, const FLKAnimVerletCollisionShapeList& InShapeList)
	{
		OutShapeList.ResetCollisionShapeList();

		OutShapeList.SphereCollisionShapes.Reserve(InShapeList.SphereCollisionShapes.Num());
		for (const FLKAnimVerletCollisionSphere& InSphere : InShapeList.SphereCollisionShapes)
		{
			if (InSphere.bUseAbsoluteWorldTransform)
			{
				OutShapeList.SphereCollisionShapes.Emplace(InSphere);
				continue;
			}

			FTransform BoneWorldTransform = FTransform::Identity;
			FVector BoneWorldScale = FVector::ZeroVector;
			if (GetBoneWorldTransform(BoneWorldTransform, BoneWorldScale, SkeletalMeshComponent, InSphere.AttachedBone.BoneName) == false)
				continue;

			FKSphereElem SphereElem;
			{
				SphereElem.Center = InSphere.LocationOffset;
				SphereElem.Radius = InSphere.Radius;
			}

			FLKAnimVerletCollisionSphere& OutSphere = OutShapeList.SphereCollisionShapes.Emplace_GetRef(InSphere);
			{
				OutSphere.bUseAbsoluteWorldTransform = true;

				const FKSphereElem ScaledSphereElem = SphereElem.GetFinalScaled(BoneWorldScale, FTransform::Identity);
				OutSphere.LocationOffset = BoneWorldTransform.TransformPosition(ScaledSphereElem.Center);
				OutSphere.Radius = ScaledSphereElem.Radius;
			}
		}

		OutShapeList.CapsuleCollisionShapes.Reserve(InShapeList.CapsuleCollisionShapes.Num());
		for (const FLKAnimVerletCollisionCapsule& InCapsule : InShapeList.CapsuleCollisionShapes)
		{
			if (InCapsule.bUseAbsoluteWorldTransform)
			{
				OutShapeList.CapsuleCollisionShapes.Emplace(InCapsule);
				continue;
			}

			FTransform BoneWorldTransform = FTransform::Identity;
			FVector BoneWorldScale = FVector::ZeroVector;
			if (GetBoneWorldTransform(BoneWorldTransform, BoneWorldScale, SkeletalMeshComponent, InCapsule.AttachedBone.BoneName) == false)
				continue;

			FKSphylElem CapsuleElem;
			{
				CapsuleElem.Center = InCapsule.LocationOffset;
				CapsuleElem.Rotation = InCapsule.RotationOffset;
				CapsuleElem.Radius = InCapsule.Radius;
				CapsuleElem.Length = InCapsule.HalfHeight * 2.0f;
			}

			FLKAnimVerletCollisionCapsule& OutCapsule = OutShapeList.CapsuleCollisionShapes.Emplace_GetRef(InCapsule);
			{
				OutCapsule.bUseAbsoluteWorldTransform = true;

				const FKSphylElem ScaledCapsuleElem = CapsuleElem.GetFinalScaled(BoneWorldScale, FTransform::Identity);
				const FTransform CapsuleWorldTransform = ScaledCapsuleElem.GetTransform() * BoneWorldTransform;
				OutCapsule.LocationOffset = CapsuleWorldTransform.GetLocation();
				OutCapsule.RotationOffset = CapsuleWorldTransform.Rotator();
				OutCapsule.Radius = ScaledCapsuleElem.Radius;
				OutCapsule.HalfHeight = ScaledCapsuleElem.Length * 0.5f;
			}
		}

		OutShapeList.BoxCollisionShapes.Reserve(InShapeList.BoxCollisionShapes.Num());
		for (const FLKAnimVerletCollisionBox& InBox : InShapeList.BoxCollisionShapes)
		{
			if (InBox.bUseAbsoluteWorldTransform)
			{
				OutShapeList.BoxCollisionShapes.Emplace(InBox);
				continue;
			}

			FTransform BoneWorldTransform = FTransform::Identity;
			FVector BoneWorldScale = FVector::ZeroVector;
			if (GetBoneWorldTransform(BoneWorldTransform, BoneWorldScale, SkeletalMeshComponent, InBox.AttachedBone.BoneName) == false)
				continue;

			FKBoxElem BoxElem;
			{
				BoxElem.Center = InBox.LocationOffset;
				BoxElem.Rotation = InBox.RotationOffset;
				BoxElem.X = InBox.HalfExtents.X * 2.0f;
				BoxElem.Y = InBox.HalfExtents.Y * 2.0f;
				BoxElem.Z = InBox.HalfExtents.Z * 2.0f;
			}

			FLKAnimVerletCollisionBox& OutBox = OutShapeList.BoxCollisionShapes.Emplace_GetRef(InBox);
			{
				OutBox.bUseAbsoluteWorldTransform = true;

				const FKBoxElem ScaledBoxElem = BoxElem.GetFinalScaled(BoneWorldScale, FTransform::Identity);
				const FTransform BoxWorldTransform = ScaledBoxElem.GetTransform() * BoneWorldTransform;
				OutBox.LocationOffset = BoxWorldTransform.GetLocation();
				OutBox.RotationOffset = BoxWorldTransform.Rotator();
				OutBox.HalfExtents = FVector(ScaledBoxElem.X, ScaledBoxElem.Y, ScaledBoxElem.Z) * 0.5f;
			}
		}

		OutShapeList.PlaneCollisionShapes.Reserve(InShapeList.PlaneCollisionShapes.Num());
		for (const FLKAnimVerletCollisionPlane& InPlane : InShapeList.PlaneCollisionShapes)
		{
			if (InPlane.bUseAbsoluteWorldTransform)
			{
				OutShapeList.PlaneCollisionShapes.Emplace(InPlane);
				continue;
			}

			FTransform BoneWorldTransform = FTransform::Identity;
			FVector BoneWorldScale = FVector::ZeroVector;
			if (GetBoneWorldTransform(BoneWorldTransform, BoneWorldScale, SkeletalMeshComponent, InPlane.AttachedBone.BoneName) == false)
				continue;

			FKBoxElem PlaneElem;
			{
				PlaneElem.Center = InPlane.LocationOffset;
				PlaneElem.Rotation = InPlane.RotationOffset;
				PlaneElem.X = InPlane.FinitePlaneHalfExtents.X * 2.0f;
				PlaneElem.Y = InPlane.FinitePlaneHalfExtents.Y * 2.0f;
				PlaneElem.Z = 0.0f;
			}

			FLKAnimVerletCollisionPlane& OutPlane = OutShapeList.PlaneCollisionShapes.Emplace_GetRef(InPlane);
			{
				OutPlane.bUseAbsoluteWorldTransform = true;

				const FKBoxElem ScaledPlaneElem = PlaneElem.GetFinalScaled(BoneWorldScale, FTransform::Identity);
				const FTransform PlaneWorldTransform = ScaledPlaneElem.GetTransform() * BoneWorldTransform;
				OutPlane.LocationOffset = PlaneWorldTransform.GetLocation();
				OutPlane.RotationOffset = PlaneWorldTransform.Rotator();
				if (OutPlane.bFinitePlane)
					OutPlane.FinitePlaneHalfExtents = FVector2D(ScaledPlaneElem.X, ScaledPlaneElem.Y) * 0.5f;
			}
		}
	}

	void AddPhysicsAssetShapes(FLKAnimVerletCollisionShapeList& OutShapeList, const UPhysicsAsset& PhysicsAsset)
	{
		for (const TObjectPtr<USkeletalBodySetup>& BodySetup : PhysicsAsset.SkeletalBodySetups)
		{
			if (BodySetup == nullptr)
				continue;

			const FName BoneName = BodySetup->BoneName;
			const FKAggregateGeom& AggregateGeometry = BodySetup->AggGeom;

			OutShapeList.SphereCollisionShapes.Reserve(OutShapeList.SphereCollisionShapes.Num() + AggregateGeometry.SphereElems.Num());
			for (const FKSphereElem& SphereElem : AggregateGeometry.SphereElems)
			{
				FLKAnimVerletCollisionSphere& SphereShape = OutShapeList.SphereCollisionShapes.Emplace_GetRef();
				{
					SphereShape.AttachedBone.BoneName = BoneName;
					SphereShape.LocationOffset = SphereElem.Center;
					SphereShape.Radius = SphereElem.Radius;
				}
			}

			OutShapeList.CapsuleCollisionShapes.Reserve(OutShapeList.CapsuleCollisionShapes.Num() + AggregateGeometry.SphylElems.Num());
			for (const FKSphylElem& CapsuleElem : AggregateGeometry.SphylElems)
			{
				FLKAnimVerletCollisionCapsule& CapsuleShape = OutShapeList.CapsuleCollisionShapes.Emplace_GetRef();
				{
					CapsuleShape.AttachedBone.BoneName = BoneName;
					CapsuleShape.LocationOffset = CapsuleElem.Center;
					CapsuleShape.RotationOffset = CapsuleElem.Rotation;
					CapsuleShape.Radius = CapsuleElem.Radius;
					CapsuleShape.HalfHeight = CapsuleElem.Length * 0.5f;
				}
			}

			OutShapeList.BoxCollisionShapes.Reserve(OutShapeList.BoxCollisionShapes.Num() + AggregateGeometry.BoxElems.Num());
			for (const FKBoxElem& BoxElem : AggregateGeometry.BoxElems)
			{
				FLKAnimVerletCollisionBox& BoxShape = OutShapeList.BoxCollisionShapes.Emplace_GetRef();
				{
					BoxShape.AttachedBone.BoneName = BoneName;
					BoxShape.LocationOffset = BoxElem.Center;
					BoxShape.RotationOffset = BoxElem.Rotation;
					BoxShape.HalfExtents = FVector(BoxElem.X, BoxElem.Y, BoxElem.Z) * 0.5f;
				}
			}
		}
	}
}

void ULKAnimVerletBlueprintFunctionLibrary::MakeCollisionShapeListFromPrimitiveComponent(FLKAnimVerletCollisionShapeList& OutShapeList, UPrimitiveComponent* PrimitiveComponent)
{
	OutShapeList.ResetCollisionShapeList();
	if (IsValid(PrimitiveComponent) == false)
		return;

	const FBodyInstance* BodyInstance = PrimitiveComponent->GetBodyInstance(NAME_None, false);
	const UBodySetup* BodySetup = BodyInstance != nullptr ? BodyInstance->GetBodySetup() : PrimitiveComponent->GetBodySetup();
	if (BodySetup == nullptr)
		return;

	const FTransform ComponentWorldTransform = PrimitiveComponent->GetComponentTransform();
	const FVector BodyScale = BodyInstance != nullptr ? BodyInstance->Scale3D : ComponentWorldTransform.GetScale3D();
	const FTransform BodyWorldTransform = LkAnimVerlet::GetBodyWorldTransform(BodyInstance, ComponentWorldTransform);
	LkAnimVerlet::AddBodySetupShapes(OutShapeList, *BodySetup, BodyScale, BodyWorldTransform);
}

void ULKAnimVerletBlueprintFunctionLibrary::MakeCollisionShapeListFromSkeletalMeshComponent(FLKAnimVerletCollisionShapeList& OutShapeList, const USkeletalMeshComponent* SkeletalMeshComponent)
{
	OutShapeList.ResetCollisionShapeList();
	if (IsValid(SkeletalMeshComponent) == false)
		return;

	for (const FBodyInstance* BodyInstance : SkeletalMeshComponent->Bodies)
	{
		if (BodyInstance == nullptr)
			continue;

		const UBodySetup* BodySetup = BodyInstance->GetBodySetup();
		if (BodySetup == nullptr)
			continue;

		FTransform FallbackWorldTransform = SkeletalMeshComponent->GetComponentTransform();
		if (const USkeletalBodySetup* SkeletalBodySetup = Cast<USkeletalBodySetup>(BodySetup))
			FallbackWorldTransform = SkeletalMeshComponent->GetSocketTransform(SkeletalBodySetup->BoneName, ERelativeTransformSpace::RTS_World);

		const FTransform BodyWorldTransform = LkAnimVerlet::GetBodyWorldTransform(BodyInstance, FallbackWorldTransform);
		LkAnimVerlet::AddBodySetupShapes(OutShapeList, *BodySetup, BodyInstance->Scale3D, BodyWorldTransform);
	}
}

void ULKAnimVerletBlueprintFunctionLibrary::MakeCollisionShapeListFromCollisionDataAsset(FLKAnimVerletCollisionShapeList& OutShapeList, const USkeletalMeshComponent* SkeletalMeshComponent, const ULKAnimVerletCollisionDataAsset* CollisionDataAsset)
{
	OutShapeList.ResetCollisionShapeList();
	if (IsValid(SkeletalMeshComponent) == false || IsValid(CollisionDataAsset) == false)
		return;

	FLKAnimVerletCollisionShapeList SourceShapeList;
	CollisionDataAsset->ConvertToShape(OUT SourceShapeList);
	LkAnimVerlet::ConvertShapeListToWorld(OutShapeList, *SkeletalMeshComponent, SourceShapeList);
}

void ULKAnimVerletBlueprintFunctionLibrary::MakeCollisionShapeListFromPhysicsAsset(FLKAnimVerletCollisionShapeList& OutShapeList, const USkeletalMeshComponent* SkeletalMeshComponent, const UPhysicsAsset* PhysicsAsset)
{
	OutShapeList.ResetCollisionShapeList();
	if (IsValid(SkeletalMeshComponent) == false || IsValid(PhysicsAsset) == false)
		return;

	FLKAnimVerletCollisionShapeList SourceShapeList;
	LkAnimVerlet::AddPhysicsAssetShapes(SourceShapeList, *PhysicsAsset);
	LkAnimVerlet::ConvertShapeListToWorld(OutShapeList, *SkeletalMeshComponent, SourceShapeList);
}

void ULKAnimVerletBlueprintFunctionLibrary::MakeCollisionShapeListFromCollisionShapeList(FLKAnimVerletCollisionShapeList& OutShapeList, const USkeletalMeshComponent* SkeletalMeshComponent, const FLKAnimVerletCollisionShapeList& CollisionShapeList)
{
	const FLKAnimVerletCollisionShapeList SourceShapeList = CollisionShapeList;
	OutShapeList.ResetCollisionShapeList();
	if (IsValid(SkeletalMeshComponent) == false)
		return;

	LkAnimVerlet::ConvertShapeListToWorld(OutShapeList, *SkeletalMeshComponent, SourceShapeList);
}
