#include "LKAnimVerletConstraint_Collision.h"

#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseContainer.h"
#include "LKAnimVerletConstraintUtil.h"


///=========================================================================================================================================
/// FLKAnimVerletConstraint_Self
///=========================================================================================================================================
FLKAnimVerletConstraint_Self::FLKAnimVerletConstraint_Self(bool InbUseTriangleSelfCollision, float InAdditionalMargin, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: AdditionalMargin(InAdditionalMargin)
	, Bones(InCollisionInput.Bones)
	, bUseTriangleSelfCollision(InbUseTriangleSelfCollision)
	, bUseBroadphase(InCollisionInput.bUseBroadphase)
	, bUseCapsuleCollisionForChain(InCollisionInput.bUseCapsuleCollisionForChain)
	, bSingleChain(InCollisionInput.bSingleChain)
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, BoneTriangles(InCollisionInput.SimulateBoneTriangleIndicators)
	, BroadphaseContainer(InCollisionInput.BroadphaseContainer)
	, bUseXPBDSolver(InCollisionInput.bUseXPBDSolver)
	, Compliance(InCollisionInput.Compliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			if (bSingleChain)
			{
				verify(BonePairs != nullptr);
				Lambdas.Reserve(BonePairs->Num() * BonePairs->Num());
			}
			else
			{
				verify(BoneTriangles != nullptr);

				if (bUseTriangleSelfCollision)
					Lambdas.Reserve(BoneTriangles->Num() * BoneTriangles->Num());		///Triangle-Triangle check
				else
					Lambdas.Reserve(BoneTriangles->Num() * Bones->Num());				///Sphere-Triangle check
			}
		}
		else
		{
			Lambdas.Reserve(Bones->Num() * Bones->Num());
		}
	}
}

void FLKAnimVerletConstraint_Self::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			if (bSingleChain)
			{
				verify(BonePairs != nullptr);
				if (Lambdas.Num() != BonePairs->Num() * BonePairs->Num())
				{
					#if	(ENGINE_MINOR_VERSION >= 5)
					Lambdas.SetNum(BonePairs->Num() * BonePairs->Num(), EAllowShrinking::No);
					#else
					Lambdas.SetNum(BonePairs->Num() * BonePairs->Num(), false);
					#endif
				}
			}
			else
			{
				verify(BoneTriangles != nullptr);

				/// Triangle-Triangle check
				if (bUseTriangleSelfCollision)
				{
					if (Lambdas.Num() != BoneTriangles->Num() * BoneTriangles->Num())
					{
						#if	(ENGINE_MINOR_VERSION >= 5)
						Lambdas.SetNum(BoneTriangles->Num() * BoneTriangles->Num(), EAllowShrinking::No);
						#else
						Lambdas.SetNum(BoneTriangles->Num() * BoneTriangles->Num(), false);
						#endif
					}
				}
				/// Sphere-Triangle check
				else
				{
					if (Lambdas.Num() != BoneTriangles->Num() * Bones->Num())
					{
						#if	(ENGINE_MINOR_VERSION >= 5)
						Lambdas.SetNum(BoneTriangles->Num() * Bones->Num(), EAllowShrinking::No);
						#else
						Lambdas.SetNum(BoneTriangles->Num() * Bones->Num(), false);
						#endif
					}
				}
			}
		}
		else
		{
			if (Lambdas.Num() != Bones->Num() * Bones->Num())
			{
				#if	(ENGINE_MINOR_VERSION >= 5)
				Lambdas.SetNum(Bones->Num() * Bones->Num(), EAllowShrinking::No);
				#else
				Lambdas.SetNum(Bones->Num() * Bones->Num(), false);
				#endif
			}
		}
	}

	if (bUseBroadphase)
	{
		if (bUseCapsuleCollisionForChain)
		{
			if (bSingleChain)
			{
				#if	(ENGINE_MINOR_VERSION >= 5)
				BroadphaseTargetCache.SetNum(BonePairs->Num(), EAllowShrinking::No);
				#else
				BroadphaseTargetCache.SetNum(BonePairs->Num(), false);
				#endif
			}
			else
			{
				/// Triangle - Triangle checks
				if (bUseTriangleSelfCollision)
				{
					#if	(ENGINE_MINOR_VERSION >= 5)
					BroadphaseTargetCache.SetNum(BoneTriangles->Num(), EAllowShrinking::No);
					#else
					BroadphaseTargetCache.SetNum(BoneTriangles->Num(), false);
					#endif
				}
				/// Sphere - Triangle checks
				else
				{
					#if	(ENGINE_MINOR_VERSION >= 5)
					BroadphaseTargetCache.SetNum(Bones->Num(), EAllowShrinking::No);
					#else
					BroadphaseTargetCache.SetNum(Bones->Num(), false);
					#endif
				}
			}
		}
		else
		{
			#if	(ENGINE_MINOR_VERSION >= 5)
			BroadphaseTargetCache.SetNum(Bones->Num(), EAllowShrinking::No);
			#else
			BroadphaseTargetCache.SetNum(Bones->Num(), false);
			#endif
		}
	}

	if (bUseCapsuleCollisionForChain)
	{
		if (bSingleChain)
			CheckCapsuleCapsule(DeltaTime, bInitialUpdate, bFinalize);
		else if (bUseTriangleSelfCollision)
			CheckTriangleTriangle(DeltaTime, bInitialUpdate, bFinalize);
		else
			CheckSphereTriangle(DeltaTime, bInitialUpdate, bFinalize);
	}
	else
	{
		CheckSphereSphere(DeltaTime, bInitialUpdate, bFinalize);
	}
}

bool FLKAnimVerletConstraint_Self::CheckSphereSphere(IN OUT FLKAnimVerletBone& BoneP, IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	if (BoneP.IsPinned() || CurVerletBone.IsPinned())
		return false;

	const float ConstraintDistance = CurVerletBone.Thickness + BoneP.Thickness + AdditionalMargin;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector SphereToBoneVec = (CurVerletBone.Location - BoneP.Location);
	const float SphereToBoneSQ = SphereToBoneVec.SizeSquared();
	if (SphereToBoneSQ < ConstraintDistanceSQ)
	{
		const float SphereToBoneDist = FMath::Sqrt(SphereToBoneSQ);
		const FVector SphereToBoneDir = SphereToBoneDist > KINDA_SMALL_NUMBER ? (SphereToBoneVec / SphereToBoneDist) : FVector::ZeroVector;
		const float PenetrationDepth = ConstraintDistance - SphereToBoneDist;
		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (BoneP.InvMass + CurVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (BoneP.IsPinned() == false)
				BoneP.Location = BoneP.Location - (SphereToBoneDir * DeltaLambda * BoneP.InvMass);

			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * CurVerletBone.InvMass);
		}
		else
		{
			const float Denom = (BoneP.InvMass + CurVerletBone.InvMass);
			const float DeltaLambda = PenetrationDepth / Denom;

			if (BoneP.IsPinned() == false)
				BoneP.Location = BoneP.Location - (SphereToBoneDir * DeltaLambda * BoneP.InvMass);

			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * CurVerletBone.InvMass);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Self::CheckSphereSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);
		verify(BroadphaseTargetCache.Num() == Bones->Num());
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			FLKAnimVerletBone& CurBone = (*Bones)[i];
			if (bInitialUpdate)
			{
				verify(BroadphaseTargetCache[i].Num() == 0);
				const FLKAnimVerletBound MyBound = CurBone.MakeBound();
				BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurUserData) {
					verify(CurUserData.Type == ELKAnimVerletBpDataCategory::Bone);

					if (i == CurUserData.ListIndex)
						return true;

					BroadphaseTargetCache[i].Emplace(CurUserData);

					FLKAnimVerletBone& OtherBone = (*Bones)[CurUserData.ListIndex];
					CheckSphereSphere(CurBone, OtherBone, DeltaTime, bInitialUpdate, bFinalize, i * Bones->Num() + CurUserData.ListIndex);
					return true;
				});
			}
			else
			{
				for (const FLKAnimVerletBpData& CurUserData : BroadphaseTargetCache[i])
				{
					FLKAnimVerletBone& OtherBone = (*Bones)[CurUserData.ListIndex];
					CheckSphereSphere(CurBone, OtherBone, DeltaTime, bInitialUpdate, bFinalize, i * Bones->Num() + CurUserData.ListIndex);
				}
			}
		}
	}
	else
	{
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			for (int32 j = i + 1; j < Bones->Num(); ++j)
			{
				if (i == j)
					continue;

				FLKAnimVerletBone& CurBone = (*Bones)[i];
				FLKAnimVerletBone& OtherBone = (*Bones)[j];
				CheckSphereSphere(CurBone, OtherBone, DeltaTime, bInitialUpdate, bFinalize, i * Bones->Num() + j);
			}
		}
	}
}

bool FLKAnimVerletConstraint_Self::CheckSphereCapsule(IN OUT FLKAnimVerletBone& BoneP, IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	const float ConstraintDistance = CurVerletBone.Thickness + BoneP.Thickness + AdditionalMargin;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector ClosestOnBone = FMath::ClosestPointOnSegment(BoneP.Location, ParentVerletBone.Location, CurVerletBone.Location);
	const FVector SphereToBoneVec = (ClosestOnBone - BoneP.Location);
	const float SphereToBoneSQ = SphereToBoneVec.SizeSquared();
	if (SphereToBoneSQ < ConstraintDistanceSQ)
	{
		FVector DirFromParent = FVector::ZeroVector;
		float DistFromParent = 0.0f;
		(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);

		const float SphereToBoneDist = FMath::Sqrt(SphereToBoneSQ);
		const FVector SphereToBoneDir = SphereToBoneDist > KINDA_SMALL_NUMBER ? (SphereToBoneVec / SphereToBoneDist) : FVector::ZeroVector;
		const float PenetrationDepth = ConstraintDistance - SphereToBoneDist;
		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			float T = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Dot(DirFromParent) / DistFromParent;
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

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = BoneP.InvMass + (W0 + W1 + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (BoneP.IsPinned() == false)
				BoneP.Location = ParentVerletBone.Location - (SphereToBoneDir * DeltaLambda * BoneP.InvMass);

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		else
		{
			float T = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Dot(DirFromParent) / DistFromParent;
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
			const double Denom = BoneP.InvMass + (W0 + W1);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const float DeltaLambda = PenetrationDepth / Denom;

			if (BoneP.IsPinned() == false)
				BoneP.Location = ParentVerletBone.Location - (SphereToBoneDir * DeltaLambda * BoneP.InvMass);

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		return true;
	}
	return false;
}

template <typename T>
void FLKAnimVerletConstraint_Self::CheckSphereCapsule(IN OUT FLKAnimVerletBone& BoneP, const T& CurPair, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	verify(CurPair.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
	FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
	if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
	{
		CheckSphereSphere(IN OUT BoneP, IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		return;
	}

	verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
	FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
	CheckSphereCapsule(IN OUT BoneP, IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Self::CheckSphereCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);
		verify(BroadphaseTargetCache.Num() == Bones->Num());
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			FLKAnimVerletBone& CurBone = (*Bones)[i];

			if (bInitialUpdate)
			{
				verify(BroadphaseTargetCache[i].Num() == 0);

				const FLKAnimVerletBound MyBound = CurBone.MakeBound();
				BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurPair) {
					verify(CurPair.Type == ELKAnimVerletBpDataCategory::Pair);

					const FLKAnimVerletBoneIndicatorPair& CurPairIndicator = (*BonePairs)[CurPair.ListIndex];
					if (i == CurPairIndicator.BoneA.AnimVerletBoneIndex || i == CurPairIndicator.BoneB.AnimVerletBoneIndex)
						return true;

					BroadphaseTargetCache[i].Emplace(CurPair);
					CheckSphereCapsule(CurBone, CurPair, DeltaTime, bInitialUpdate, bFinalize, i * BonePairs->Num() + CurPair.ListIndex);
					return true;
				});
			}
			else
			{
				for (const FLKAnimVerletBpData& CurPair : BroadphaseTargetCache[i])
					CheckSphereCapsule(CurBone, CurPair, DeltaTime, bInitialUpdate, bFinalize, i * BonePairs->Num() + CurPair.ListIndex);
			}
		}
	}
	else
	{
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			FLKAnimVerletBone& CurBone = (*Bones)[i];
			for (int32 j = 0; j < BonePairs->Num(); ++j)
			{
				const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[j];
				if (i == CurPair.BoneA.AnimVerletBoneIndex || i == CurPair.BoneB.AnimVerletBoneIndex)
					continue;

				CheckSphereCapsule(IN OUT CurBone, CurPair, DeltaTime, bInitialUpdate, bFinalize, i * BonePairs->Num() + j);
			}
		}
	}
}

bool FLKAnimVerletConstraint_Self::CheckCapsuleCapsule(IN OUT FLKAnimVerletBone& BoneP1, IN OUT FLKAnimVerletBone& BoneP2, IN OUT FLKAnimVerletBone& BoneA1, IN OUT FLKAnimVerletBone& BoneA2, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	float S = 0.0f;
	float T = 0.0f;
	FVector ClosestOnP = FVector::ZeroVector;
	FVector ClosestOnA = FVector::ZeroVector;
	LKAnimVerletUtil::ClosestPointsSegmentSegment(OUT S, OUT T, OUT ClosestOnP, OUT ClosestOnA, BoneP1.Location, BoneP2.Location, BoneA1.Location, BoneA2.Location);

	const FVector Diff = ClosestOnP - ClosestOnA;
	const float DistSq = Diff.SizeSquared();
	const float Target = FMath::Max(BoneA1.Thickness + BoneA2.Thickness) + FMath::Max(BoneP1.Thickness, BoneP2.Thickness) + AdditionalMargin;
	const float TargetSQ = FMath::Square(Target);
	if (DistSq >= TargetSQ)
		return false;

	const float Dist = FMath::Sqrt(FMath::Max(DistSq, KINDA_SMALL_NUMBER));
	FVector N = Diff / Dist;

	/// Fallback
	if (N.IsNormalized() == false)
	{
		const FVector DP = (BoneP2.Location - BoneP1.Location);
		const FVector DA = (BoneA2.Location - BoneA1.Location);
		N = DP.Cross(DA);
		if (N.SizeSquared() < KINDA_SMALL_NUMBER)
			N = FVector::UpVector;

		N.Normalize();
	}

	const float C = Target - Dist;

	if (bUseXPBDSolver && bFinalize == false)
	{
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return false;

		const float wP1 = (1.0f - S);
		const float wP2 = S;
		const float wA1 = (1.0f - T);
		const float wA2 = T;

		const FVector GradP1 = -wP1 * N;
		const FVector GradP2 = -wP2 * N;
		const FVector GradA1 = +wA1 * N;
		const FVector GradA2 = +wA2 * N;

		double& CurLambda = Lambdas[LambdaIndex];
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double Denom = (BoneP1.InvMass * GradP1.SizeSquared() + BoneP2.InvMass * GradP2.SizeSquared()) + (BoneA1.InvMass * GradA1.SizeSquared() + BoneA2.InvMass * GradA2.SizeSquared()) + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		if (BoneP1.IsPinned() == false)
			BoneP1.Location = BoneP1.Location + (DeltaLambda * GradP1 * BoneP1.InvMass);
		if (BoneP2.IsPinned() == false)
			BoneP2.Location = BoneP2.Location + (DeltaLambda * GradP2 * BoneP2.InvMass);

		if (BoneA1.IsPinned() == false)
			BoneA1.Location = BoneA1.Location + (DeltaLambda * GradA1 * BoneA1.InvMass);
		if (BoneA2.IsPinned() == false)
			BoneA2.Location = BoneA2.Location + (DeltaLambda * GradA2 * BoneA2.InvMass);
	}
	else
	{
		const float wP1 = (1.0f - S);
		const float wP2 = S;
		const float wA1 = (1.0f - T);
		const float wA2 = T;

		const FVector GradP1 = -wP1 * N;
		const FVector GradP2 = -wP2 * N;
		const FVector GradA1 = +wA1 * N;
		const FVector GradA2 = +wA2 * N;

		const float Denom = (BoneP1.InvMass * GradP1.SizeSquared() + BoneP2.InvMass * GradP2.SizeSquared()) + (BoneA1.InvMass * GradA1.SizeSquared() + BoneA2.InvMass * GradA2.SizeSquared());
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const float DeltaLambda = -C / Denom;

		if (BoneP1.IsPinned() == false)
			BoneP1.Location = BoneP1.Location + (DeltaLambda * GradP1 * BoneP1.InvMass);
		if (BoneP2.IsPinned() == false)
			BoneP2.Location = BoneP2.Location + (DeltaLambda * GradP2 * BoneP2.InvMass);

		if (BoneA1.IsPinned() == false)
			BoneA1.Location = BoneA1.Location + (DeltaLambda * GradA1 * BoneA1.InvMass);
		if (BoneA2.IsPinned() == false)
			BoneA2.Location = BoneA2.Location + (DeltaLambda * GradA2 * BoneA2.InvMass);
	}
	return true;
}

template <typename T>
void FLKAnimVerletConstraint_Self::CheckCapsuleCapsule(IN OUT FLKAnimVerletBone& BoneP1, IN OUT FLKAnimVerletBone& BoneP2, const T& CurPair, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	verify(CurPair.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
	FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
	if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
	{
		///CheckSphereCapsule(IN OUT BoneP1, IN OUT BoneP2, IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		return;
	}

	verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
	FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
	CheckCapsuleCapsule(BoneP1, BoneP2, ParentVerletBone, CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Self::CheckCapsuleCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);
		verify(BroadphaseTargetCache.Num() == BonePairs->Num());
		for (int32 i = 0; i < BonePairs->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorPair& Pair1 = (*BonePairs)[i];
			if (Pair1.BoneA.IsValidBoneIndicator() == false || Pair1.BoneB.IsValidBoneIndicator() == false)
				continue;

			verify(Bones->IsValidIndex(Pair1.BoneA.AnimVerletBoneIndex));
			verify(Bones->IsValidIndex(Pair1.BoneB.AnimVerletBoneIndex));
			FLKAnimVerletBone& BoneP1 = (*Bones)[Pair1.BoneA.AnimVerletBoneIndex];
			FLKAnimVerletBone& BoneP2 = (*Bones)[Pair1.BoneB.AnimVerletBoneIndex];

			if (bInitialUpdate)
			{
				verify(BroadphaseTargetCache[i].Num() == 0);

				const FLKAnimVerletBound MyBound = Pair1.MakeBound(*Bones);
				BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurPair) {
					verify(CurPair.Type == ELKAnimVerletBpDataCategory::Pair);

					if (i == CurPair.ListIndex)
						return true;

					const FLKAnimVerletBoneIndicatorPair& CurPairIndicator = (*BonePairs)[CurPair.ListIndex];
					if (Pair1.BoneA.AnimVerletBoneIndex == CurPair.BoneA.AnimVerletBoneIndex
						|| Pair1.BoneA.AnimVerletBoneIndex == CurPair.BoneB.AnimVerletBoneIndex
						|| Pair1.BoneB.AnimVerletBoneIndex == CurPair.BoneA.AnimVerletBoneIndex
						|| Pair1.BoneB.AnimVerletBoneIndex == CurPair.BoneB.AnimVerletBoneIndex)
						return true;

					BroadphaseTargetCache[i].Emplace(CurPair);
					CheckCapsuleCapsule(BoneP1, BoneP2, CurPair, DeltaTime, bInitialUpdate, bFinalize, i * BonePairs->Num() + CurPair.ListIndex);
					return true;
				});
			}
			else
			{
				for (const FLKAnimVerletBpData& CurPair : BroadphaseTargetCache[i])
					CheckCapsuleCapsule(BoneP1, BoneP2, CurPair, DeltaTime, bInitialUpdate, bFinalize, i * BonePairs->Num() + CurPair.ListIndex);
			}
		}
	}
	else
	{
		for (int32 i = 0; i < BonePairs->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorPair& Pair1 = (*BonePairs)[i];
			for (int32 j = i + 1; j < BonePairs->Num(); ++j)
			{
				if (i == j)
					continue;

				const FLKAnimVerletBoneIndicatorPair& Pair2 = (*BonePairs)[j];
				if (Pair1.BoneA.AnimVerletBoneIndex == Pair2.BoneA.AnimVerletBoneIndex 
					|| Pair1.BoneA.AnimVerletBoneIndex == Pair2.BoneB.AnimVerletBoneIndex
					|| Pair1.BoneB.AnimVerletBoneIndex == Pair2.BoneA.AnimVerletBoneIndex
					|| Pair1.BoneB.AnimVerletBoneIndex == Pair2.BoneB.AnimVerletBoneIndex)
					continue;

				if (Pair1.BoneA.IsValidBoneIndicator() == false || Pair1.BoneB.IsValidBoneIndicator() == false)
					continue;

				verify(Bones->IsValidIndex(Pair1.BoneA.AnimVerletBoneIndex));
				verify(Bones->IsValidIndex(Pair1.BoneB.AnimVerletBoneIndex));
				FLKAnimVerletBone& BoneP1 = (*Bones)[Pair1.BoneA.AnimVerletBoneIndex];
				FLKAnimVerletBone& BoneP2 = (*Bones)[Pair1.BoneB.AnimVerletBoneIndex];
				CheckCapsuleCapsule(BoneP1, BoneP2, Pair2, DeltaTime, bInitialUpdate, bFinalize, i * BonePairs->Num() + j);
			}
		}
	}
}

bool FLKAnimVerletConstraint_Self::CheckSphereTriangle(IN OUT FLKAnimVerletBone& BoneP, IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	float WA = 0.0f;
	float WB = 0.0f;
	float WC = 0.0f;
	const FVector Q = LKAnimVerletUtil::ClosestPointOnTriangleWeights(OUT WA, OUT WB, OUT WC, BoneP.Location, BoneA.Location, BoneB.Location, BoneC.Location);

	const FVector D = BoneP.Location - Q;
	float Dist = D.Size();

	/// Consider triangle thickness
	const float TriThickness = FMath::Max3(BoneA.Thickness, BoneB.Thickness, BoneC.Thickness);
	const float Target = BoneP.Thickness + AdditionalMargin + TriThickness;

	/// CollisionNormal
	FVector N = FVector::ZeroVector;
	if (Dist > KINDA_SMALL_NUMBER)
	{
		N = D / Dist;	///from triangle to sphere center
	}
	else
	{
		// Fallback: triangle normal if possible
		const FVector TriN = (BoneB.Location - BoneA.Location).Cross(BoneC.Location - BoneA.Location);
		N = (TriN.SizeSquared() > KINDA_SMALL_NUMBER) ? TriN.GetSafeNormal() : FVector::UpVector;
		Dist = 0.0f;
	}

	/// C = d - (R + T) (if penetrate then C < 0)
	const float Cval = Dist - Target;
	if (Cval >= 0.0f)
		return false;

	if (bUseXPBDSolver && bFinalize == false)
	{
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return false;

		double& CurLambda = Lambdas[LambdaIndex];
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const float SumGrad = BoneP.InvMass + BoneA.InvMass * (WA * WA) + BoneB.InvMass * (WB * WB) + BoneC.InvMass * (WC * WC);
		const float Denom = SumGrad + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(Cval + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		if (BoneP.IsPinned() == false)
			BoneP.Location = BoneP.Location - (BoneP.InvMass * DeltaLambda) * -N;

		if (BoneA.IsPinned() == false)
			BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * N);
		if (BoneB.IsPinned() == false)
			BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * N);
		if (BoneC.IsPinned() == false)
			BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * N);
	}
	else
	{
		const float W0 = BoneA.InvMass * (WA * WA);
		const float W1 = BoneB.InvMass * (WB * WB);
		const float W2 = BoneC.InvMass * (WC * WC);
		const double Denom = BoneP.InvMass + (W0 + W1 + W2);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const float DeltaLambda = -Cval / Denom;

		if (BoneP.IsPinned() == false)
			BoneP.Location = BoneP.Location - (BoneP.InvMass * DeltaLambda) * -N;

		if (BoneA.IsPinned() == false)
			BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * N);
		if (BoneB.IsPinned() == false)
			BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * N);
		if (BoneC.IsPinned() == false)
			BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * N);
	}
	return true;
}

template <typename T>
void FLKAnimVerletConstraint_Self::CheckSphereTriangle(IN OUT FLKAnimVerletBone& BoneP, const T& CurTriangle, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
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
		CheckSphereSphere(IN OUT BoneP, IN OUT AVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		CheckSphereSphere(IN OUT BoneP, IN OUT BVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		CheckSphereSphere(IN OUT BoneP, IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		return;
	}

	CheckSphereTriangle(IN OUT BoneP, IN OUT AVerletBone, IN OUT BVerletBone, IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Self::CheckSphereTriangle(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);
		verify(BroadphaseTargetCache.Num() == Bones->Num());
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			FLKAnimVerletBone& CurBone = (*Bones)[i];

			if (bInitialUpdate)
			{
				verify(BroadphaseTargetCache[i].Num() == 0);

				const FLKAnimVerletBound MyBound = CurBone.MakeBound();
				BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurTriangle) {
					verify(CurTriangle.Type == ELKAnimVerletBpDataCategory::Triangle);

					const FLKAnimVerletBoneIndicatorTriangle& CurTriangleIndicator = (*BoneTriangles)[CurTriangle.ListIndex];
					if (i == CurTriangleIndicator.BoneA.AnimVerletBoneIndex || i == CurTriangleIndicator.BoneB.AnimVerletBoneIndex || i == CurTriangleIndicator.BoneC.AnimVerletBoneIndex)
						return true;

					BroadphaseTargetCache[i].Emplace(CurTriangle);
					CheckSphereTriangle(CurBone, CurTriangle, DeltaTime, bInitialUpdate, bFinalize, i * BoneTriangles->Num() + CurTriangle.ListIndex);
					return true;
				});
			}
			else
			{
				for (const FLKAnimVerletBpData& CurTriangle : BroadphaseTargetCache[i])
					CheckSphereTriangle(CurBone, CurTriangle, DeltaTime, bInitialUpdate, bFinalize, i * BoneTriangles->Num() + CurTriangle.ListIndex);
			}
		}
	}
	else
	{
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			FLKAnimVerletBone& CurBone = (*Bones)[i];
			for (int32 j = 0; j < BoneTriangles->Num(); ++j)
			{
				const FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*BoneTriangles)[j];
				if (i == CurTriangle.BoneA.AnimVerletBoneIndex || i == CurTriangle.BoneB.AnimVerletBoneIndex || i == CurTriangle.BoneC.AnimVerletBoneIndex)
					continue;

				CheckSphereTriangle(CurBone, CurTriangle, DeltaTime, bInitialUpdate, bFinalize, i * BoneTriangles->Num() + j);
			}
		}
	}
}

bool FLKAnimVerletConstraint_Self::CheckTriangleTriangle(IN OUT FLKAnimVerletBone& BoneA1, IN OUT FLKAnimVerletBone& BoneA2, IN OUT FLKAnimVerletBone& BoneA3,
														 IN OUT FLKAnimVerletBone& BoneB1, IN OUT FLKAnimVerletBone& BoneB2, IN OUT FLKAnimVerletBone& BoneB3,
														 float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	const FVector A1 = BoneA1.Location;
	const FVector A2 = BoneA2.Location;
	const FVector A3 = BoneA3.Location;

	const FVector B1 = BoneB1.Location;
	const FVector B2 = BoneB2.Location;
	const FVector B3 = BoneB3.Location;

	/// min distance candidates
	/// P = wa1*a1 + wa2*a2 + wa3*a3
	/// Q = wb1*b1 + wb2*b2 + wb3*b3
	float WA[3] = { 0,0,0 };
	float WB[3] = { 0,0,0 };
	FVector P = FVector::ZeroVector;
	FVector Q = FVector::ZeroVector;
	float MinDist2 = TNumericLimits<float>::Max();

	auto TrySetCandidate = [&](const float InWa[3], const float InWb[3], const FVector& InP, const FVector& InQ)
	{
		const float D2 = (InP - InQ).SizeSquared();
		if (D2 < MinDist2)
		{
			MinDist2 = D2;
			WA[0] = InWa[0]; WA[1] = InWa[1]; WA[2] = InWa[2];
			WB[0] = InWb[0]; WB[1] = InWb[1]; WB[2] = InWb[2];
			P = InP; 
			Q = InQ;
		}
	};

	/// 1) Vertex(A) - Triangle(B)
	{
		const FVector VA[3] = { A1, A2, A3 };
		for (int32 i = 0; i < 3; ++i)
		{
			float WB1 = 0.0f;
			float WB2 = 0.0f;
			float WB3 = 0.0f;
			const FVector Qp = LKAnimVerletUtil::ClosestPointOnTriangleWeights(OUT WB1, OUT WB2, OUT WB3, VA[i], B1, B2, B3);

			float CWa[3] = { 0.0f, 0.0f, 0.0f };
			float CWb[3] = { WB1, WB2, WB3 };
			CWa[i] = 1.0f;

			TrySetCandidate(CWa, CWb, VA[i], Qp);
		}
	}

	/// 2) Vertex(B) - Triangle(A)
	{
		const FVector VB[3] = { B1, B2, B3 };
		for (int32 i = 0; i < 3; ++i)
		{
			float WA1 = 0.0f;
			float WA2 = 0.0f;
			float WA3 = 0.0f;
			const FVector Pp = LKAnimVerletUtil::ClosestPointOnTriangleWeights(WA1, WA2, WA3, VB[i], A1, A2, A3);

			float CWa[3] = { WA1, WA2, WA3 };
			float CWb[3] = { 0.0f, 0.0f, 0.0f };
			CWb[i] = 1.0f;

			TrySetCandidate(CWa, CWb, Pp, VB[i]);
		}
	}

	/// 3) Edge(A) - Edge(B): 3x3 = 9
	{
		const FVector AE0[3] = { A1, A2, A3 };
		const FVector AE1[3] = { A2, A3, A1 };
		const FVector BE0[3] = { B1, B2, B3 };
		const FVector BE1[3] = { B2, B3, B1 };

		for (int32 ea = 0; ea < 3; ++ea)
		{
			for (int32 eb = 0; eb < 3; ++eb)
			{
				float S = 0.0f;
				float T = 0.0f;
				FVector C1 = FVector::ZeroVector;
				FVector C2 = FVector::ZeroVector;
				LKAnimVerletUtil::ClosestPointsSegmentSegment(OUT S, OUT T, OUT C1, OUT C2, AE0[ea], AE1[ea], BE0[eb], BE1[eb]);

				/// weights on A (start/end vertex of ea)
				float CWa[3] = { 0.0f, 0.0f, 0.0f };
				{
					const int Ia0 = ea;
					const int Ia1 = (ea + 1) % 3;
					CWa[Ia0] = 1.0f - S;
					CWa[Ia1] = S;
				}

				/// weights on B (start/end vertex of eb)
				float CWb[3] = { 0.0f, 0.0f, 0.0f };
				{
					const int Ib0 = eb;
					const int Ib1 = (eb + 1) % 3;
					CWb[Ib0] = 1.0f - T;
					CWb[Ib1] = T;
				}

				TrySetCandidate(CWa, CWb, C1, C2);
			}
		}
	}

	const float Dist = FMath::Sqrt(FMath::Max(MinDist2, 0.0f));

	const float ThickA = FMath::Max3(BoneA1.Thickness, BoneA2.Thickness, BoneA3.Thickness); ///WA[0] * BoneA1.Thickness + WA[1] * BoneA2.Thickness + WA[2] * BoneA3.Thickness;
	const float ThickB = FMath::Max3(BoneB1.Thickness, BoneB2.Thickness, BoneB3.Thickness); ///WB[0] * BoneB1.Thickness + WB[1] * BoneB2.Thickness + WB[2] * BoneB3.Thickness;
	const float Target = ThickA + ThickB + AdditionalMargin;
	const float C = Dist - Target;
	if (C >= 0.0f) 
		return false;

	FVector N = P - Q; /// B -> A
	if (N.SizeSquared() <= KINDA_SMALL_NUMBER)
	{
		/// Fallback
		const FVector NA = (A2 - A1).Cross(A3 - A1);
		const FVector NB = (B2 - B1).Cross(B3 - B1);
		N = (NA.SizeSquared() > NB.SizeSquared()) ? NA : NB;
		if (N.SizeSquared() <= KINDA_SMALL_NUMBER) 
			N = FVector(0.0f, 0.0f, 1.0f);
	}
	N.Normalize();

	if (bUseXPBDSolver && bFinalize == false)
	{
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return false;

		double& CurLambda = Lambdas[LambdaIndex];
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const float Denom = (BoneA1.InvMass * WA[0] * WA[0]) + (BoneA2.InvMass * WA[1] * WA[1]) + (BoneA3.InvMass * WA[2] * WA[2])
							+ (BoneB1.InvMass * WB[0] * WB[0]) + (BoneB2.InvMass * WB[1] * WB[1]) + (BoneB3.InvMass * WB[2] * WB[2])
							+ Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		if (BoneA1.IsPinned() == false)
			BoneA1.Location += (BoneA1.InvMass * WA[0] * DeltaLambda) * N;
		if (BoneA2.IsPinned() == false)
			BoneA2.Location += (BoneA2.InvMass * WA[1] * DeltaLambda) * N;
		if (BoneA3.IsPinned() == false)
			BoneA3.Location += (BoneA3.InvMass * WA[2] * DeltaLambda) * N;

		if (BoneB1.IsPinned() == false)
			BoneB1.Location -= (BoneB1.InvMass * WB[0] * DeltaLambda) * N;
		if (BoneB2.IsPinned() == false)
			BoneB2.Location -= (BoneB2.InvMass * WB[1] * DeltaLambda) * N;
		if (BoneB3.IsPinned() == false)
			BoneB3.Location -= (BoneB3.InvMass * WB[2] * DeltaLambda) * N;
	}
	else
	{
		const float Denom = (BoneA1.InvMass * WA[0] * WA[0]) + (BoneA2.InvMass * WA[1] * WA[1]) + (BoneA3.InvMass * WA[2] * WA[2])
							+ (BoneB1.InvMass * WB[0] * WB[0]) + (BoneB2.InvMass * WB[1] * WB[1]) + (BoneB3.InvMass * WB[2] * WB[2]);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const float DeltaLambda = -C / Denom;

		if (BoneA1.IsPinned() == false)
			BoneA1.Location += (BoneA1.InvMass * WA[0] * DeltaLambda) * N;
		if (BoneA2.IsPinned() == false)
			BoneA2.Location += (BoneA2.InvMass * WA[1] * DeltaLambda) * N;
		if (BoneA3.IsPinned() == false)
			BoneA3.Location += (BoneA3.InvMass * WA[2] * DeltaLambda) * N;

		if (BoneB1.IsPinned() == false)
			BoneB1.Location -= (BoneB1.InvMass * WB[0] * DeltaLambda) * N;
		if (BoneB2.IsPinned() == false)
			BoneB2.Location -= (BoneB2.InvMass * WB[1] * DeltaLambda) * N;
		if (BoneB3.IsPinned() == false)
			BoneB3.Location -= (BoneB3.InvMass * WB[2] * DeltaLambda) * N;
	}
	return true;
}

template <typename T, typename K>
void FLKAnimVerletConstraint_Self::CheckTriangleTriangle(const T& InTriangleA, const K& InTriangleB, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	verify(InTriangleA.BoneA.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(InTriangleA.BoneA.AnimVerletBoneIndex));
	verify(InTriangleA.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(InTriangleA.BoneB.AnimVerletBoneIndex));
	verify(InTriangleA.BoneC.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(InTriangleA.BoneC.AnimVerletBoneIndex));

	verify(InTriangleB.BoneA.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(InTriangleB.BoneA.AnimVerletBoneIndex));
	verify(InTriangleB.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(InTriangleB.BoneB.AnimVerletBoneIndex));
	verify(InTriangleB.BoneC.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(InTriangleB.BoneC.AnimVerletBoneIndex));

	FLKAnimVerletBone& BoneA1 = (*Bones)[InTriangleA.BoneA.AnimVerletBoneIndex];
	FLKAnimVerletBone& BoneA2 = (*Bones)[InTriangleA.BoneB.AnimVerletBoneIndex];
	FLKAnimVerletBone& BoneA3 = (*Bones)[InTriangleA.BoneC.AnimVerletBoneIndex];

	FLKAnimVerletBone& BoneB1 = (*Bones)[InTriangleB.BoneA.AnimVerletBoneIndex];
	FLKAnimVerletBone& BoneB2 = (*Bones)[InTriangleB.BoneB.AnimVerletBoneIndex];
	FLKAnimVerletBone& BoneB3 = (*Bones)[InTriangleB.BoneC.AnimVerletBoneIndex];
	CheckTriangleTriangle(IN OUT BoneA1, IN OUT BoneA2, IN OUT BoneA3, 
						  IN OUT BoneB1, IN OUT BoneB2, IN OUT BoneB3, 
						  DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Self::CheckTriangleTriangle(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);
		verify(BroadphaseTargetCache.Num() == BoneTriangles->Num());
		for (int32 i = 0; i < BoneTriangles->Num(); ++i)
		{	
			const FLKAnimVerletBoneIndicatorTriangle& TriangleA = (*BoneTriangles)[i];
			if (bInitialUpdate)
			{
				verify(BroadphaseTargetCache[i].Num() == 0);

				const FLKAnimVerletBound MyBound = TriangleA.MakeBound(*Bones);
				BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurTriangle) {
					verify(CurTriangle.Type == ELKAnimVerletBpDataCategory::Triangle);

					const FLKAnimVerletBoneIndicatorTriangle& TriangleB = (*BoneTriangles)[CurTriangle.ListIndex];
					if (TriangleA.BoneA.AnimVerletBoneIndex == TriangleB.BoneA.AnimVerletBoneIndex
						|| TriangleA.BoneA.AnimVerletBoneIndex == TriangleB.BoneB.AnimVerletBoneIndex
						|| TriangleA.BoneA.AnimVerletBoneIndex == TriangleB.BoneC.AnimVerletBoneIndex

						|| TriangleA.BoneB.AnimVerletBoneIndex == TriangleB.BoneA.AnimVerletBoneIndex
						|| TriangleA.BoneB.AnimVerletBoneIndex == TriangleB.BoneB.AnimVerletBoneIndex
						|| TriangleA.BoneB.AnimVerletBoneIndex == TriangleB.BoneC.AnimVerletBoneIndex

						|| TriangleA.BoneC.AnimVerletBoneIndex == TriangleB.BoneA.AnimVerletBoneIndex
						|| TriangleA.BoneC.AnimVerletBoneIndex == TriangleB.BoneB.AnimVerletBoneIndex
						|| TriangleA.BoneC.AnimVerletBoneIndex == TriangleB.BoneC.AnimVerletBoneIndex)
						return true;

					BroadphaseTargetCache[i].Emplace(CurTriangle);
					CheckTriangleTriangle(TriangleA, CurTriangle, DeltaTime, bInitialUpdate, bFinalize, i * BoneTriangles->Num() + CurTriangle.ListIndex);
					return true;
				});
			}
			else
			{
				for (const FLKAnimVerletBpData& CurTriangle : BroadphaseTargetCache[i])
					CheckTriangleTriangle(TriangleA, CurTriangle, DeltaTime, bInitialUpdate, bFinalize, i * BoneTriangles->Num() + CurTriangle.ListIndex);
			}
		}
	}
	else
	{
		for (int32 i = 0; i < BoneTriangles->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorTriangle& TriangleA = (*BoneTriangles)[i];
			for (int32 j = i + 1; j < BoneTriangles->Num(); ++j)
			{
				const FLKAnimVerletBoneIndicatorTriangle& TriangleB = (*BoneTriangles)[j];
				if (TriangleA.BoneA.AnimVerletBoneIndex == TriangleB.BoneA.AnimVerletBoneIndex
					|| TriangleA.BoneA.AnimVerletBoneIndex == TriangleB.BoneB.AnimVerletBoneIndex
					|| TriangleA.BoneA.AnimVerletBoneIndex == TriangleB.BoneC.AnimVerletBoneIndex
					
					|| TriangleA.BoneB.AnimVerletBoneIndex == TriangleB.BoneA.AnimVerletBoneIndex
					|| TriangleA.BoneB.AnimVerletBoneIndex == TriangleB.BoneB.AnimVerletBoneIndex
					|| TriangleA.BoneB.AnimVerletBoneIndex == TriangleB.BoneC.AnimVerletBoneIndex

					|| TriangleA.BoneC.AnimVerletBoneIndex == TriangleB.BoneA.AnimVerletBoneIndex
					|| TriangleA.BoneC.AnimVerletBoneIndex == TriangleB.BoneB.AnimVerletBoneIndex
					|| TriangleA.BoneC.AnimVerletBoneIndex == TriangleB.BoneC.AnimVerletBoneIndex)
					continue;

				CheckTriangleTriangle(TriangleA, TriangleB, DeltaTime, bInitialUpdate, bFinalize, i * BoneTriangles->Num() + j);
			}
		}
	}
}
///=========================================================================================================================================