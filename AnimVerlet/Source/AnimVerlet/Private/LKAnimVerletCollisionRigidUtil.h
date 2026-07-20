#pragma once
#include "LKAnimVerletBone.h"

namespace LkAnimVerletCollision
{
	FVector MakePBDCollisionFrictionDelta(const FVector& ContactDisplacement, const FVector& CollisionNormal, float NormalCorrectionMagnitude, float FrictionCoefficient);
	void ApplyPBDCollisionFriction(IN OUT FLKAnimVerletBone& Bone, const FVector& CollisionNormal, float NormalCorrectionMagnitude, float FrictionCoefficient);
	void ApplyPBDCollisionFriction(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, float WeightA, float WeightB,
		const FVector& CollisionNormal, float NormalCorrectionMagnitude, float FrictionCoefficient);
	void ApplyPBDCollisionFriction(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC,
		float WeightA, float WeightB, float WeightC, const FVector& CollisionNormal, float NormalCorrectionMagnitude, float FrictionCoefficient);

	struct FLkRigidCapsuleContact
	{
		FVector Center = FVector::ZeroVector;
		FVector ParentOffset = FVector::ZeroVector;
		FVector CurOffset = FVector::ZeroVector;
		FVector AngularJacobian = FVector::ZeroVector;
		float InverseTotalMass = 0.0f;
		float InversePerpendicularInertia = 0.0f;
		float GeneralizedInverseMass = 0.0f;
	};

	inline bool MakeRigidCapsuleContact(OUT FLkRigidCapsuleContact& OutContact, const FLKAnimVerletBone& ParentBone, const FLKAnimVerletBone& CurBone, float SegmentT, const FVector& CollisionNormal)
	{
		OutContact = FLkRigidCapsuleContact();
		if (ParentBone.IsPinned() && CurBone.IsPinned())
			return false;

		float PerpendicularInertia = 0.0f;
		if (ParentBone.IsPinned())
		{
			if (CurBone.InvMass <= KINDA_SMALL_NUMBER)
				return false;

			OutContact.Center = ParentBone.Location;
			OutContact.ParentOffset = FVector::ZeroVector;
			OutContact.CurOffset = CurBone.Location - OutContact.Center;
			PerpendicularInertia = OutContact.CurOffset.SizeSquared() / CurBone.InvMass;
		}
		else if (CurBone.IsPinned())
		{
			if (ParentBone.InvMass <= KINDA_SMALL_NUMBER)
				return false;

			OutContact.Center = CurBone.Location;
			OutContact.ParentOffset = ParentBone.Location - OutContact.Center;
			OutContact.CurOffset = FVector::ZeroVector;
			PerpendicularInertia = OutContact.ParentOffset.SizeSquared() / ParentBone.InvMass;
		}
		else
		{
			if (ParentBone.InvMass <= KINDA_SMALL_NUMBER || CurBone.InvMass <= KINDA_SMALL_NUMBER)
				return false;

			const float ParentMass = 1.0f / ParentBone.InvMass;
			const float CurMass = 1.0f / CurBone.InvMass;
			const float TotalMass = ParentMass + CurMass;
			OutContact.Center = (ParentBone.Location * ParentMass + CurBone.Location * CurMass) / TotalMass;
			OutContact.ParentOffset = ParentBone.Location - OutContact.Center;
			OutContact.CurOffset = CurBone.Location - OutContact.Center;
			OutContact.InverseTotalMass = 1.0f / TotalMass;
			PerpendicularInertia = ParentMass * OutContact.ParentOffset.SizeSquared() + CurMass * OutContact.CurOffset.SizeSquared();
		}

		if (PerpendicularInertia <= KINDA_SMALL_NUMBER)
			return false;

		const FVector ContactPoint = FMath::Lerp(ParentBone.Location, CurBone.Location, FMath::Clamp(SegmentT, 0.0f, 1.0f));
		OutContact.AngularJacobian = (ContactPoint - OutContact.Center).Cross(CollisionNormal);
		OutContact.InversePerpendicularInertia = 1.0f / PerpendicularInertia;
		OutContact.GeneralizedInverseMass = OutContact.InverseTotalMass + (OutContact.AngularJacobian.SizeSquared() * OutContact.InversePerpendicularInertia);
		return (OutContact.GeneralizedInverseMass > KINDA_SMALL_NUMBER);
	}

	inline void ApplyRigidCapsuleCorrection(IN OUT FLKAnimVerletBone& ParentBone, IN OUT FLKAnimVerletBone& CurBone, const FLkRigidCapsuleContact& Contact, const FVector& CollisionNormal, float DeltaLambda)
	{
		const FVector CenterDelta = CollisionNormal * (DeltaLambda * Contact.InverseTotalMass);
		const FVector AngularDelta = Contact.AngularJacobian * (DeltaLambda * Contact.InversePerpendicularInertia);
		const float AngularDistance = AngularDelta.Size();
		const FQuat RotationDelta = (AngularDistance > KINDA_SMALL_NUMBER ? FQuat(AngularDelta / AngularDistance, AngularDistance) : FQuat::Identity);

		const FVector NewCenter = Contact.Center + CenterDelta;
		if (ParentBone.IsPinned() == false)
			ParentBone.Location = NewCenter + RotationDelta.RotateVector(Contact.ParentOffset);
		if (CurBone.IsPinned() == false)
			CurBone.Location = NewCenter + RotationDelta.RotateVector(Contact.CurOffset);
	}

	inline void ApplyNormalCorrectionTwoBone(IN OUT FLKAnimVerletBone& ParentVerletBone, IN OUT FLKAnimVerletBone& CurVerletBone, const FLkRigidCapsuleContact& RigidContact, 
											 const FVector& InNormal, bool bApplyRigidResponse, float DeltaLambda, float B0, float B1)
	{
		if (bApplyRigidResponse)
		{
			ApplyRigidCapsuleCorrection(IN OUT ParentVerletBone, IN OUT CurVerletBone, RigidContact, InNormal, DeltaLambda);
		}
		else
		{
			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location += InNormal * DeltaLambda * B0 * ParentVerletBone.InvMass;
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location += InNormal * DeltaLambda * B1 * CurVerletBone.InvMass;
		}
	};
}
