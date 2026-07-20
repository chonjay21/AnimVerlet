#include "LKAnimVerletConstraint_Collision.h"

#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseContainer.h"
#include "LKAnimVerletCollisionRigidUtil.h"
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
	, bUseCapsuleCollisionForChain(InCollisionInput.bUseCapsuleCollisionForChain)
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, FrictionCoefficient(InCollisionInput.FrictionCoefficient)
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
		const FVector LocationBeforeCorrection = CurVerletBone.Location;
		const FVector CollisionNormal = ComponentTransform.InverseTransformVectorNoScale(HitResult.Normal).GetSafeNormal();
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

		const float NormalCorrectionMagnitude = FMath::Abs((CurVerletBone.Location - LocationBeforeCorrection).Dot(CollisionNormal));
		LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT CurVerletBone, CollisionNormal, NormalCorrectionMagnitude, FrictionCoefficient);
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
	if (ParentVerletBone.IsPinned() && CurVerletBone.IsPinned())
		return false;

	FVector DirFromParent = FVector::ZeroVector;
	float DistFromParent = 0.0f;
	(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);
	const float CapsuleHalfHeight = DistFromParent * 0.5f + CurVerletBone.Thickness;

	const FVector VerletBoneCenter = (CurVerletBone.Location + ParentVerletBone.Location) * 0.5f;
	const FVector PrevVerletBoneCenter = (CurVerletBone.PrevLocation + ParentVerletBone.PrevLocation) * 0.5f;

	const FVector PrevWorldLoc = ComponentTransform.TransformPosition(PrevVerletBoneCenter);
	const FVector CurWorldLoc = ComponentTransform.TransformPosition(VerletBoneCenter);

	FHitResult HitResult;
	const FVector LocalCapsuleDirection = DirFromParent.IsNearlyZero(KINDA_SMALL_NUMBER) ? FVector::UpVector : DirFromParent;
	const FVector WorldCapsuleDirection = ComponentTransform.TransformVectorNoScale(LocalCapsuleDirection).GetSafeNormal();
	const bool bHit = World->SweepSingleByProfile(OUT HitResult, PrevWorldLoc, CurWorldLoc, FRotationMatrix::MakeFromZ(-WorldCapsuleDirection).ToQuat(), WorldCollisionProfileName, FCollisionShape::MakeCapsule(CurVerletBone.Thickness, CapsuleHalfHeight), CollisionQueryParams);
	if (bHit)
	{
		const FVector CollisionNormal = ComponentTransform.InverseTransformVectorNoScale(HitResult.Normal).GetSafeNormal();
		const FVector ContactPoint = ComponentTransform.InverseTransformPosition(HitResult.ImpactPoint);
		const float ContactT = FMath::Clamp(FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ContactPoint - ParentVerletBone.Location).Dot(DirFromParent) / DistFromParent, 0.0f, 1.0f);
		float ParticleT = ContactT;
		if (ParentVerletBone.IsPinned())
			ParticleT = 1.0f;
		if (CurVerletBone.IsPinned())
			ParticleT = 0.0f;

		const float B0 = 1.0f - ParticleT;
		const float B1 = ParticleT;
		const float W0 = ParentVerletBone.InvMass * B0 * B0;
		const float W1 = CurVerletBone.InvMass * B1 * B1;

		FVector Correction = FVector::ZeroVector;
		if (HitResult.bStartPenetrating && HitResult.PenetrationDepth > 0.0f)
		{
			Correction = CollisionNormal * HitResult.PenetrationDepth;
		}
		else
		{
			const FVector ResolvedCenter = ComponentTransform.InverseTransformPosition(HitResult.Location);
			Correction = ResolvedCenter - VerletBoneCenter;
		}

		float CorrectionDistance = 0.0f;
		FVector CorrectionNormal = FVector::ZeroVector;
		Correction.ToDirectionAndLength(OUT CorrectionNormal, OUT CorrectionDistance);
		if (CorrectionDistance <= KINDA_SMALL_NUMBER)
			return true;

		LkAnimVerletCollision::FLkRigidCapsuleContact RigidContact;
		const bool bApplyRigidResponse = LkAnimVerletCollision::MakeRigidCapsuleContact(OUT RigidContact, ParentVerletBone, CurVerletBone, ContactT, CorrectionNormal);
		const float GeneralizedInverseMass = bApplyRigidResponse ? RigidContact.GeneralizedInverseMass : W0 + W1;
		if (GeneralizedInverseMass <= KINDA_SMALL_NUMBER)
			return false;
		const float FrictionB0 = bApplyRigidResponse ? 1.0f - ContactT : B0;
		const float FrictionB1 = bApplyRigidResponse ? ContactT : B1;

		const float DeltaLambda = CorrectionDistance / GeneralizedInverseMass;
		LkAnimVerletCollision::ApplyNormalCorrectionTwoBone(IN OUT ParentVerletBone, IN OUT CurVerletBone, RigidContact, CorrectionNormal, bApplyRigidResponse, DeltaLambda, B0, B1);

		const float NormalCorrectionMagnitude = FMath::Abs(Correction.Dot(CollisionNormal));
		LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT ParentVerletBone, IN OUT CurVerletBone, FrictionB0, FrictionB1, CollisionNormal, NormalCorrectionMagnitude, FrictionCoefficient);
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