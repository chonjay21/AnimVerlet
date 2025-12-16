#include "LKAnimVerletConstraint_Collision.h"

#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseContainer.h"
#include "LKAnimVerletConstraintUtil.h"


///=========================================================================================================================================
/// FLKAnimVerletConstraint_World
///=========================================================================================================================================
FLKAnimVerletConstraint_World::FLKAnimVerletConstraint_World(const UWorld* InWorld, UPrimitiveComponent* InSelfComponent, const FName& InCollisionProfileName, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: WorldPtr(InWorld)
	, SelfComponentPtr(InSelfComponent)
	, WorldCollisionProfileName(InCollisionProfileName)
	, Bones(InCollisionInput.Bones)
	, ExcludeBones(InCollisionInput.ExcludeBones)
{
	verify(WorldPtr.IsValid());
	verify(SelfComponentPtr.IsValid());
	verify(WorldCollisionProfileName != NAME_None);
	verify(Bones != nullptr);
}

void FLKAnimVerletConstraint_World::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	verify(Bones != nullptr);

	if (WorldPtr.IsValid() == false || SelfComponentPtr.IsValid() == false)
		return;

	if (bUseCapsuleCollisionForChain)
		CheckWorldCapsule(DeltaTime, bInitialUpdate, bFinalize);
	else
		CheckWorldSphere(DeltaTime, bInitialUpdate, bFinalize);
}

bool FLKAnimVerletConstraint_World::CheckWorldSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, const UWorld* World, 
													 const FCollisionQueryParams& CollisionQueryParams, const FTransform& ComponentTransform, int32 LambdaIndex)
{
	verify(World != nullptr);

	if (CurVerletBone.IsPinned())
		return false;

	const FVector PrevWorldLoc = ComponentTransform.TransformPosition(CurVerletBone.PrevLocation);
	const FVector CurWorldLoc = ComponentTransform.TransformPosition(CurVerletBone.Location);

	FHitResult HitResult;
	const bool bHit = World->SweepSingleByProfile(OUT HitResult, PrevWorldLoc, CurWorldLoc, FQuat::Identity, WorldCollisionProfileName, FCollisionShape::MakeSphere(CurVerletBone.Thickness), CollisionQueryParams);
	if (bHit)
	{
		if (HitResult.bStartPenetrating && HitResult.PenetrationDepth > 0.0f)
		{
			const FVector ResolvedLocationInWorld = HitResult.Location + (HitResult.Normal * HitResult.PenetrationDepth);
			CurVerletBone.Location = ComponentTransform.InverseTransformPosition(ResolvedLocationInWorld);
		}
		else
		{
			const FVector ResolvedLocationInWorld = HitResult.Location;
			CurVerletBone.Location = ComponentTransform.InverseTransformPosition(ResolvedLocationInWorld);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_World::CheckWorldSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const UWorld* World = WorldPtr.Get();
	UPrimitiveComponent* SelfComponent = SelfComponentPtr.Get();

	FCollisionQueryParams CollisionQueryParams(SCENE_QUERY_STAT(LKAnimVerlet));
	CollisionQueryParams.AddIgnoredComponent(SelfComponent);
	if (SelfComponent->GetOwner() != nullptr)
		CollisionQueryParams.AddIgnoredActor(SelfComponent->GetOwner());

	const FTransform ComponentTransform = SelfComponent->GetComponentTransform();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		CheckWorldSphere(CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
	}
}

bool FLKAnimVerletConstraint_World::CheckWorldCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, 
													  const UWorld* World, const FCollisionQueryParams& CollisionQueryParams, const FTransform& ComponentTransform, int32 LambdaIndex)
{
	FVector DirFromParent = FVector::ZeroVector;
	float DistFromParent = 0.0f;
	(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);
	const float CapsuleHalfHeight = DistFromParent * 0.5f + CurVerletBone.Thickness;

	const FVector VerletBoneCenter = (CurVerletBone.Location + ParentVerletBone.Location) * 0.5f;
	const FVector PrevVerletBoneCenter = (CurVerletBone.PrevLocation + ParentVerletBone.PrevLocation) * 0.5f;

	const FVector PrevWorldLoc = ComponentTransform.TransformPosition(PrevVerletBoneCenter);
	const FVector CurWorldLoc = ComponentTransform.TransformPosition(VerletBoneCenter);

	FHitResult HitResult;
	const bool bHit = World->SweepSingleByProfile(OUT HitResult, PrevWorldLoc, CurWorldLoc, FRotationMatrix::MakeFromZ(-DirFromParent).ToQuat(), WorldCollisionProfileName, FCollisionShape::MakeCapsule(CurVerletBone.Thickness, CapsuleHalfHeight), CollisionQueryParams);
	if (bHit)
	{
		float T = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (HitResult.Location - ParentVerletBone.Location).Dot(DirFromParent) / DistFromParent;
		T = FMath::Clamp(T, 0.0f, 1.0f);

		if (ParentVerletBone.IsPinned())
			T = 1.0f;
		if (CurVerletBone.IsPinned())
			T = 0.0f;

		/// Barycentric Weights
		const float B0 = 1.0f - T;
		const float B1 = T;
		const float W0 = ParentVerletBone.InvMass * B0 * B0;
		const float W1 = CurVerletBone.InvMass * B1 * B1;
		const double Denom = (W0 + W1);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		if (HitResult.bStartPenetrating && HitResult.PenetrationDepth > 0.0f)
		{
			const float DeltaLambda = HitResult.PenetrationDepth / Denom;
			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (HitResult.Normal * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (HitResult.Normal * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		else
		{
			const FVector ResolvedLocationInWorld = HitResult.Location;
			const FVector NewLocation = ComponentTransform.InverseTransformPosition(ResolvedLocationInWorld);

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (NewLocation - VerletBoneCenter) * B0 * ParentVerletBone.InvMass;
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (NewLocation - VerletBoneCenter) * B1 * CurVerletBone.InvMass;
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_World::CheckWorldCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const UWorld* World = WorldPtr.Get();
	UPrimitiveComponent* SelfComponent = SelfComponentPtr.Get();

	FCollisionQueryParams CollisionQueryParams(SCENE_QUERY_STAT(LKAnimVerlet));
	CollisionQueryParams.AddIgnoredComponent(SelfComponent);
	if (SelfComponent->GetOwner() != nullptr)
		CollisionQueryParams.AddIgnoredActor(SelfComponent->GetOwner());

	const FTransform ComponentTransform = SelfComponent->GetComponentTransform();
	for (int32 i = 0; i < BonePairs->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
		if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
			continue;

		verify(CurPair.BoneB.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
		if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
		{
			CheckWorldSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		CheckWorldCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
	}
}
///=========================================================================================================================================