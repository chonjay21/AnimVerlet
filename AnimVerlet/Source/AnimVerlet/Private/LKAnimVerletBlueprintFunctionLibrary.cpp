#include "LKAnimVerletBlueprintFunctionLibrary.h"

#include <Components/PrimitiveComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <PhysicsEngine/AggregateGeom.h>
#include <PhysicsEngine/BodyInstance.h>
#include <PhysicsEngine/BodySetup.h>
#include <PhysicsEngine/BoxElem.h>
#include <PhysicsEngine/SkeletalBodySetup.h>
#include <PhysicsEngine/SphereElem.h>
#include <PhysicsEngine/SphylElem.h>

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
