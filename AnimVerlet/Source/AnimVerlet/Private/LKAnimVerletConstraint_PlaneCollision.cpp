#include "LKAnimVerletConstraint_Collision.h"

#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseContainer.h"
#include "LKAnimVerletCollisionRigidUtil.h"
#include "LKAnimVerletConstraintUtil.h"


///=========================================================================================================================================
/// FLKAnimVerletConstraint_Plane
///=========================================================================================================================================
FLKAnimVerletConstraint_Plane::FLKAnimVerletConstraint_Plane(const FVector& InPlaneBase, const FVector& InPlaneNormal, const FQuat& InRotation, 
															 const FVector2D& InPlaneHalfExtents, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: PlaneBase(InPlaneBase)
	, PlaneNormal(InPlaneNormal)
	, Rotation(InRotation)
	, PlaneHalfExtents(InPlaneHalfExtents)
	, Bones(InCollisionInput.Bones)
	, ExcludeBones(InCollisionInput.ExcludeBones)
	, bUseCapsuleCollisionForChain(InCollisionInput.bUseCapsuleCollisionForChain)
	, bSingleChain(InCollisionInput.bSingleChain)
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, BoneTriangles(InCollisionInput.SimulateBoneTriangleIndicators)
	, bUseXPBDSolver(InCollisionInput.bUseXPBDSolver)
	, Compliance(InCollisionInput.Compliance)
	, FrictionCoefficient(InCollisionInput.FrictionCoefficient)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			if (bSingleChain)
			{
				verify(BonePairs != nullptr);
				Lambdas.Reserve(BonePairs->Num());
			}
			else
			{
				verify(BoneTriangles != nullptr);
				Lambdas.Reserve(BoneTriangles->Num());
			}
		}
		else
		{
			Lambdas.Reserve(Bones->Num());
		}
	}
}

void FLKAnimVerletConstraint_Plane::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			if (bSingleChain)
			{
				verify(BonePairs != nullptr);
				if (Lambdas.Num() != BonePairs->Num())
				{
					#if	(ENGINE_MINOR_VERSION >= 5)
					Lambdas.SetNum(BonePairs->Num(), EAllowShrinking::No);
					#else
					Lambdas.SetNum(BonePairs->Num(), false);
					#endif
				}
			}
			else
			{
				verify(BoneTriangles != nullptr);
				if (Lambdas.Num() != BoneTriangles->Num())
				{
					#if	(ENGINE_MINOR_VERSION >= 5)
					Lambdas.SetNum(BoneTriangles->Num(), EAllowShrinking::No);
					#else
					Lambdas.SetNum(BoneTriangles->Num(), false);
					#endif
				}
			}
		}
		else
		{
			if (Lambdas.Num() != Bones->Num())
			{
				#if	(ENGINE_MINOR_VERSION >= 5)
				Lambdas.SetNum(Bones->Num(), EAllowShrinking::No);
				#else
				Lambdas.SetNum(Bones->Num(), false);
				#endif
			}
		}
	}

	if (bUseCapsuleCollisionForChain)
	{
		if (bSingleChain)
			CheckPlaneCapsule(DeltaTime, bInitialUpdate, bFinalize);
		else
			CheckPlaneTriangle(DeltaTime, bInitialUpdate, bFinalize);
	}
	else
	{
		CheckPlaneSphere(DeltaTime, bInitialUpdate, bFinalize);
	}
}

bool FLKAnimVerletConstraint_Plane::CheckPlaneSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex)
{
	if (CurVerletBone.IsPinned())
		return false;

	const float DistToPlane = FVector::PointPlaneDist(CurVerletBone.Location, PlaneBase, PlaneNormal);
	if (DistToPlane < CurVerletBone.Thickness)
	{
		if (bFinitePlane)
		{
			const FVector ProjectedLocationOnPlane = CurVerletBone.Location - (DistToPlane * PlaneNormal);
			const FVector BoneLocationInPlaneLocal = InvRotation.RotateVector(ProjectedLocationOnPlane - PlaneBase);
			if (BoneLocationInPlaneLocal.X > PlaneHalfExtents.X + CurVerletBone.Thickness || BoneLocationInPlaneLocal.X < -PlaneHalfExtents.X + CurVerletBone.Thickness ||
				BoneLocationInPlaneLocal.Y > PlaneHalfExtents.Y + CurVerletBone.Thickness || BoneLocationInPlaneLocal.Y < -PlaneHalfExtents.Y + CurVerletBone.Thickness)
				return false;
		}

		const float PenetrationDepth = (CurVerletBone.Thickness - DistToPlane);
		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (CurVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location += (PlaneNormal * DeltaLambda);

			LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT CurVerletBone, PlaneNormal, static_cast<float>(FMath::Abs(DeltaLambda)), FrictionCoefficient);
		}
		else
		{
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location += (PlaneNormal * PenetrationDepth);

			LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT CurVerletBone, PlaneNormal, PenetrationDepth, FrictionCoefficient);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Plane::CheckPlaneSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		CheckPlaneSphere(CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
	}
}

bool FLKAnimVerletConstraint_Plane::CheckPlaneCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex)
{
	FVector DirFromParent = FVector::ZeroVector;
	float DistFromParent = 0.0f;
	(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);
	const FVector VerletBoneCenter = (CurVerletBone.Location + ParentVerletBone.Location) * 0.5f;

	FVector ClosestOnBone = FVector::ZeroVector;
	const float CapsuleDotPlane = DirFromParent.Dot(PlaneNormal);
	if (FMath::IsNearlyZero(CapsuleDotPlane, KINDA_SMALL_NUMBER))
		ClosestOnBone = VerletBoneCenter;
	else if (CapsuleDotPlane < 0.0f)
		ClosestOnBone = CurVerletBone.Location;
	else
		ClosestOnBone = ParentVerletBone.Location;

	const float DistToPlane = FVector::PointPlaneDist(ClosestOnBone, PlaneBase, PlaneNormal);
	if (DistToPlane < CurVerletBone.Thickness)
	{
		if (bFinitePlane)
		{
			const FVector ProjectedLocationOnPlane = ClosestOnBone - (DistToPlane * PlaneNormal);
			const FVector BoneLocationInPlaneLocal = InvRotation.RotateVector(ProjectedLocationOnPlane - PlaneBase);
			if (BoneLocationInPlaneLocal.X > PlaneHalfExtents.X + CurVerletBone.Thickness || BoneLocationInPlaneLocal.X < -PlaneHalfExtents.X + CurVerletBone.Thickness ||
				BoneLocationInPlaneLocal.Y > PlaneHalfExtents.Y + CurVerletBone.Thickness || BoneLocationInPlaneLocal.Y < -PlaneHalfExtents.Y + CurVerletBone.Thickness)
				return false;
		}

		const float PenetrationDepth = (CurVerletBone.Thickness - DistToPlane);
		const float ContactT = FMath::Clamp(FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Dot(DirFromParent) / DistFromParent, 0.0f, 1.0f);
		float ParticleT = ContactT;
		if (ParentVerletBone.IsPinned())
			ParticleT = 1.0f;
		if (CurVerletBone.IsPinned())
			ParticleT = 0.0f;

		const float B0 = 1.0f - ParticleT;
		const float B1 = ParticleT;
		const float W0 = ParentVerletBone.InvMass * B0 * B0;
		const float W1 = CurVerletBone.InvMass * B1 * B1;

		LkAnimVerletCollision::FLkRigidCapsuleContact RigidContact;
		const bool bApplyRigidResponse = LkAnimVerletCollision::MakeRigidCapsuleContact(OUT RigidContact, ParentVerletBone, CurVerletBone, ContactT, PlaneNormal);
		const float GeneralizedInverseMass = bApplyRigidResponse ? RigidContact.GeneralizedInverseMass : W0 + W1;
		if (GeneralizedInverseMass <= KINDA_SMALL_NUMBER)
			return false;
		const float FrictionB0 = bApplyRigidResponse ? 1.0f - ContactT : B0;
		const float FrictionB1 = bApplyRigidResponse ? ContactT : B1;

		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = GeneralizedInverseMass + Alpha;
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double OldLambda = CurLambda;
			const double RawDeltaLambda = -(C + Alpha * OldLambda) / Denom;
			CurLambda = FMath::Max(OldLambda + RawDeltaLambda, 0.0);
			const float AppliedDeltaLambda = static_cast<float>(CurLambda - OldLambda);
			LkAnimVerletCollision::ApplyNormalCorrectionTwoBone(IN OUT ParentVerletBone, IN OUT CurVerletBone, RigidContact, PlaneNormal, bApplyRigidResponse, AppliedDeltaLambda, B0, B1);
			LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT ParentVerletBone, IN OUT CurVerletBone, FrictionB0, FrictionB1, PlaneNormal, FMath::Abs(AppliedDeltaLambda) * GeneralizedInverseMass, FrictionCoefficient);
		}
		else
		{
			const float DeltaLambda = PenetrationDepth / GeneralizedInverseMass;
			LkAnimVerletCollision::ApplyNormalCorrectionTwoBone(IN OUT ParentVerletBone, IN OUT CurVerletBone, RigidContact, PlaneNormal, bApplyRigidResponse, DeltaLambda, B0, B1);
			LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT ParentVerletBone, IN OUT CurVerletBone, FrictionB0, FrictionB1, PlaneNormal, FMath::Abs(DeltaLambda) * GeneralizedInverseMass, FrictionCoefficient);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Plane::CheckPlaneCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
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
			CheckPlaneSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		CheckPlaneCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
	}
}

bool FLKAnimVerletConstraint_Plane::CheckPlaneTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bInitialUpdate, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex)
{
	FVector N = PlaneNormal;
	const float NSq = N.SizeSquared();

	if (NSq <= KINDA_SMALL_NUMBER)
	{
		// Fallback: local + Z(0,0,1) becomes a plane normal candidate by returning it to world.
		const FQuat PlaneRot = InvRotation.Inverse();
		N = PlaneRot.RotateVector(FVector::UpVector);
	}

	N = N.GetSafeNormal();
	if (N.IsNearlyZero(KINDA_SMALL_NUMBER))
		return false;

	/// Signed distances to plane
	const float SA = (BoneA.Location - PlaneBase).Dot(N);
	const float SB = (BoneB.Location - PlaneBase).Dot(N);
	const float SC = (BoneC.Location - PlaneBase).Dot(N);

	/// Minimum distance point on triangle w.r.t. plane normal
	const float SMin = FMath::Min3(SA, SB, SC);

	/// Consider triangle thickness
	const float TriThickness = FMath::Max3(BoneA.Thickness, BoneB.Thickness, BoneC.Thickness);

	/// Constraint: C = sMin - TriThickness
	const float Cval = SMin - TriThickness;
	if (Cval >= 0.0f)
		return false;

	/// Choose weights for the "active" (deepest) vertex/vertices. (If multiple vertices tie within epsilon, distribute evenly)
	float WA = 0.0f;
	float WB = 0.0f;
	float WC = 0.0f;
	int Count = 0;

	if (FMath::Abs(SA - SMin) <= KINDA_SMALL_NUMBER) 
		++Count;
	if (FMath::Abs(SB - SMin) <= KINDA_SMALL_NUMBER)
		++Count;
	if (FMath::Abs(SC - SMin) <= KINDA_SMALL_NUMBER)
		++Count;

	if (Count <= 0)
	{
		/// Should not happen, but keep safe
		WA = 1.0f;
	}
	else
	{
		const float W = 1.0f / (float)Count;
		if (FMath::Abs(SA - SMin) <= KINDA_SMALL_NUMBER) 
			WA = W;
		if (FMath::Abs(SB - SMin) <= KINDA_SMALL_NUMBER) 
			WB = W;
		if (FMath::Abs(SC - SMin) <= KINDA_SMALL_NUMBER) 
			WC = W;
	}

	if (bFinitePlane)
	{
		const FVector ProjectedLocationsOnPlane[3] = { BoneA.Location - (SA * N), BoneB.Location - (SB * N), BoneC.Location - (SC * N) };
		for (int32 i = 0; i < 3; ++i)
		{
			const FVector ProjectedLocationOnPlane = ProjectedLocationsOnPlane[i];
			const FVector BoneLocationInPlaneLocal = InvRotation.RotateVector(ProjectedLocationOnPlane - PlaneBase);
			if (BoneLocationInPlaneLocal.X > PlaneHalfExtents.X + TriThickness || BoneLocationInPlaneLocal.X < -PlaneHalfExtents.X + TriThickness ||
				BoneLocationInPlaneLocal.Y > PlaneHalfExtents.Y + TriThickness || BoneLocationInPlaneLocal.Y < -PlaneHalfExtents.Y + TriThickness)
				return false;
		}
	}

	if (bUseXPBDSolver && bFinalize == false)
	{
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return false;

		double& CurLambda = Lambdas[LambdaIndex];
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const float SumGrad = BoneA.InvMass * (WA * WA) + BoneB.InvMass * (WB * WB) + BoneC.InvMass * (WC * WC);
		const float Denom = SumGrad + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(Cval + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		if (BoneA.IsPinned() == false)
			BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (WA * N);
		if (BoneB.IsPinned() == false)
			BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (WB * N);
		if (BoneC.IsPinned() == false)
			BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (WC * N);

		LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT BoneA, IN OUT BoneB, IN OUT BoneC, WA, WB, WC, N, static_cast<float>(FMath::Abs(DeltaLambda) * SumGrad), FrictionCoefficient);
	}
	else
	{
		const float W0 = BoneA.InvMass * (WA * WA);
		const float W1 = BoneB.InvMass * (WB * WB);
		const float W2 = BoneC.InvMass * (WC * WC);
		const double Denom = (W0 + W1 + W2);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const float DeltaLambda = -Cval / Denom;

		if (BoneA.IsPinned() == false)
			BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (WA * N);
		if (BoneB.IsPinned() == false)
			BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (WB * N);
		if (BoneC.IsPinned() == false)
			BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (WC * N);

		LkAnimVerletCollision::ApplyPBDCollisionFriction(IN OUT BoneA, IN OUT BoneB, IN OUT BoneC, WA, WB, WC, N, FMath::Abs(DeltaLambda) * (W0 + W1 + W2), FrictionCoefficient);
	}
	return true;
}

void FLKAnimVerletConstraint_Plane::CheckPlaneTriangle(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < BoneTriangles->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*BoneTriangles)[i];
		if (ExcludeBones.IsValidIndex(CurTriangle.BoneA.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneA.AnimVerletBoneIndex])
			continue;
		if (ExcludeBones.IsValidIndex(CurTriangle.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneB.AnimVerletBoneIndex])
			continue;
		if (ExcludeBones.IsValidIndex(CurTriangle.BoneC.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneC.AnimVerletBoneIndex])
			continue;

		verify(CurTriangle.BoneA.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurTriangle.BoneA.AnimVerletBoneIndex));

		verify(CurTriangle.BoneB.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurTriangle.BoneB.AnimVerletBoneIndex));

		verify(CurTriangle.BoneC.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurTriangle.BoneC.AnimVerletBoneIndex));

		FLKAnimVerletBone& AVerletBone = (*Bones)[CurTriangle.BoneA.AnimVerletBoneIndex];
		FLKAnimVerletBone& BVerletBone = (*Bones)[CurTriangle.BoneB.AnimVerletBoneIndex];
		FLKAnimVerletBone& CVerletBone = (*Bones)[CurTriangle.BoneC.AnimVerletBoneIndex];
		if (CurTriangle.BoneA.IsValidBoneIndicator() == false || AVerletBone.bOverrideToUseSphereCollisionForChain ||
			CurTriangle.BoneB.IsValidBoneIndicator() == false || BVerletBone.bOverrideToUseSphereCollisionForChain ||
			CurTriangle.BoneC.IsValidBoneIndicator() == false || CVerletBone.bOverrideToUseSphereCollisionForChain)
		{
			CheckPlaneSphere(IN OUT AVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
			CheckPlaneSphere(IN OUT BVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
			CheckPlaneSphere(IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
			continue;
		}

		CheckPlaneTriangle(IN OUT AVerletBone, IN OUT BVerletBone, IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, bFinitePlane, InvRotation, i);
	}
}
///=========================================================================================================================================