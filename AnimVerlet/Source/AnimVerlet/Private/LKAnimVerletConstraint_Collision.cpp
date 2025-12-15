#include "LKAnimVerletConstraint_Collision.h"

#include "LKAnimVerletBone.h"
#include "LKAnimVerletBroadphaseContainer.h"
#include "LKAnimVerletConstraintUtil.h"


///=========================================================================================================================================
/// FLKAnimVerletConstraint_Sphere
///=========================================================================================================================================
FLKAnimVerletConstraint_Sphere::FLKAnimVerletConstraint_Sphere(const FVector& InLocation, float InRadius, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: Location(InLocation)
	, Radius(InRadius)
	, Bones(InCollisionInput.Bones)
	, ExcludeBones(InCollisionInput.ExcludeBones)
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

void FLKAnimVerletConstraint_Sphere::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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
			CheckSphereCapsule(DeltaTime, bInitialUpdate, bFinalize);
		else
			CheckSphereTriangle(DeltaTime, bInitialUpdate, bFinalize);
	}
	else
	{
		CheckSphereSphere(DeltaTime, bInitialUpdate, bFinalize);
	}
}

bool FLKAnimVerletConstraint_Sphere::CheckSphereSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	if (CurVerletBone.IsPinned())
		return false;

	const float ConstraintDistance = CurVerletBone.Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector SphereToBoneVec = (CurVerletBone.Location - Location);
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
			const double Denom = (CurVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda);
		}
		else
		{
			CurVerletBone.Location = Location + (SphereToBoneDir * ConstraintDistance);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Sphere::CheckSphereSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(LambdaIndex) && ExcludeBones[LambdaIndex])
		return;

	FLKAnimVerletBone& CurVerletBone = (*Bones)[LambdaIndex];
	CheckSphereSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Sphere::CheckSphereSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurUserData) {		
			verify(CurUserData.Type == ELKAnimVerletBpDataCategory::Bone);
			CheckSphereSphere(DeltaTime, bInitialUpdate, bFinalize, CurUserData.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			CheckSphereSphere(DeltaTime, bInitialUpdate, bFinalize, i);
		}
	}
}

bool FLKAnimVerletConstraint_Sphere::CheckSphereCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	const float ConstraintDistance = CurVerletBone.Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector ClosestOnBone = FMath::ClosestPointOnSegment(Location, ParentVerletBone.Location, CurVerletBone.Location);
	const FVector SphereToBoneVec = (ClosestOnBone - Location);
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

			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1 + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		else
		{
			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;	
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
			const double Denom = (W0 + W1);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const float DeltaLambda = PenetrationDepth / Denom;
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
void FLKAnimVerletConstraint_Sphere::CheckSphereCapsule(const T& CurPair, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
		return;

	verify(CurPair.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
	FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
	if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
	{
		CheckSphereSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		return;
	}

	verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
	FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
	CheckSphereCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Sphere::CheckSphereCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurPair) {
			verify(CurPair.Type == ELKAnimVerletBpDataCategory::Pair);
			CheckSphereCapsule(CurPair, DeltaTime, bInitialUpdate, bFinalize, CurPair.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < BonePairs->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
			CheckSphereCapsule(CurPair, DeltaTime, bInitialUpdate, bFinalize, i);
		}
	}
}

bool FLKAnimVerletConstraint_Sphere::CheckSphereTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	float WA = 0.0f;
	float WB = 0.0f;
	float WC = 0.0f;
	const FVector Q = LKAnimVerletUtil::ClosestPointOnTriangleWeights(OUT WA, OUT WB, OUT WC, Location, BoneA.Location, BoneB.Location, BoneC.Location);

	const FVector D = Location - Q;
	float Dist = D.Size();

	/// Consider triangle thickness
	const float TriThickness = FMath::Max3(BoneA.Thickness, BoneB.Thickness, BoneC.Thickness);
	const float Target = Radius + TriThickness;

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
		const float SumGrad = BoneA.InvMass * (WA * WA) + BoneB.InvMass * (WB * WB) + BoneC.InvMass * (WC * WC);
		const float Denom = SumGrad + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(Cval + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * N);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * N);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * N);
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
		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * N);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * N);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * N);
	}
	return true;
}

template <typename T>
void FLKAnimVerletConstraint_Sphere::CheckSphereTriangle(const T& CurTriangle, float DeltaTime, bool bInitialUpdate, bool bFinalize, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneA.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneA.AnimVerletBoneIndex])
		return;
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneB.AnimVerletBoneIndex])
		return;
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneC.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneC.AnimVerletBoneIndex])
		return;

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
		CheckSphereSphere(IN OUT AVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		CheckSphereSphere(IN OUT BVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		CheckSphereSphere(IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
		return;
	}

	CheckSphereTriangle(IN OUT AVerletBone, IN OUT BVerletBone, IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, LambdaIndex);
}

void FLKAnimVerletConstraint_Sphere::CheckSphereTriangle(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurTriangle) {
			verify(CurTriangle.Type == ELKAnimVerletBpDataCategory::Triangle);
			CheckSphereTriangle(CurTriangle, DeltaTime, bInitialUpdate, bFinalize, CurTriangle.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < BoneTriangles->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*BoneTriangles)[i];
			CheckSphereTriangle(CurTriangle, DeltaTime, bInitialUpdate, bFinalize, i);
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Capsule
///=========================================================================================================================================
FLKAnimVerletConstraint_Capsule::FLKAnimVerletConstraint_Capsule(const FVector& InLocation, const FQuat& InRot, float InRadius,
																 float InHalfHeight, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: Location(InLocation)
	, Rotation(InRot)
	, Radius(InRadius)
	, HalfHeight(InHalfHeight)
	, Bones(InCollisionInput.Bones)
	, ExcludeBones(InCollisionInput.ExcludeBones)
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

void FLKAnimVerletConstraint_Capsule::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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
			CheckCapsuleCapsule(DeltaTime, bInitialUpdate, bFinalize);
		else
			CheckCapsuleTriangle(DeltaTime, bInitialUpdate, bFinalize);
	}
	else
	{
		CheckCapsuleSphere(DeltaTime, bInitialUpdate, bFinalize);
	}
}

FLKAnimVerletBound FLKAnimVerletConstraint_Capsule::MakeBound() const
{
	const FVector UpVector = Rotation.GetUpVector();
	const FVector CapsuleStart = (Location - UpVector * HalfHeight);
	const FVector CapsuleEnd = (Location + UpVector * HalfHeight);
	
	const FVector AabbMin(FMath::Min(CapsuleStart.X, CapsuleEnd.X) - Radius, FMath::Min(CapsuleStart.Y, CapsuleEnd.Y) - Radius, FMath::Min(CapsuleStart.Z, CapsuleEnd.Z) - Radius);
	const FVector AabbMax(FMath::Max(CapsuleStart.X, CapsuleEnd.X) + Radius, FMath::Max(CapsuleStart.Y, CapsuleEnd.Y) + Radius, FMath::Max(CapsuleStart.Z, CapsuleEnd.Z) + Radius);
	return FLKAnimVerletBound::MakeBoundFromMinMax(AabbMin, AabbMax);
}

bool FLKAnimVerletConstraint_Capsule::CheckCapsuleSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
	if (CurVerletBone.IsPinned())
		return false;

	const float ConstraintDistance = CurVerletBone.Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector ClosestOnCapsule = FMath::ClosestPointOnSegment(CurVerletBone.Location, CapsuleStart, CapsuleEnd);
	const FVector CapsuleToBoneVec = (CurVerletBone.Location - ClosestOnCapsule);
	const float CapsuleToBoneSQ = CapsuleToBoneVec.SizeSquared();
	if (CapsuleToBoneSQ < ConstraintDistanceSQ)
	{
		const float CapsuleToBoneDist = FMath::Sqrt(CapsuleToBoneSQ);
		const FVector CapsuleToBoneDir = CapsuleToBoneDist > KINDA_SMALL_NUMBER ? (CapsuleToBoneVec / CapsuleToBoneDist) : FVector::ZeroVector;
		const float PenetrationDepth = ConstraintDistance - CapsuleToBoneDist;
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

			CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * DeltaLambda);
		}
		else
		{
			CurVerletBone.Location = ClosestOnCapsule + (CapsuleToBoneDir * ConstraintDistance);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Capsule::CheckCapsuleSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(LambdaIndex) && ExcludeBones[LambdaIndex])
		return;

	FLKAnimVerletBone& CurVerletBone = (*Bones)[LambdaIndex];
	CheckCapsuleSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
}

void FLKAnimVerletConstraint_Capsule::CheckCapsuleSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;

	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurUserData) {
			verify(CurUserData.Type == ELKAnimVerletBpDataCategory::Bone);
			CheckCapsuleSphere(DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, CurUserData.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			CheckCapsuleSphere(DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, i);
		}
	}
}

bool FLKAnimVerletConstraint_Capsule::CheckCapsuleCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
	const float ConstraintDistance = CurVerletBone.Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	FVector DirFromParent = FVector::ZeroVector;
	float DistFromParent = 0.0f;
	(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);

	FVector ClosestOnBone = FVector::ZeroVector;
	FVector ClosestOnCapsule = FVector::ZeroVector;
	FMath::SegmentDistToSegment(ParentVerletBone.Location, CurVerletBone.Location, CapsuleStart, CapsuleEnd, OUT ClosestOnBone, OUT ClosestOnCapsule);

	const float CapsuleToBoneSQ = (ClosestOnBone - ClosestOnCapsule).SizeSquared();
	if (CapsuleToBoneSQ < ConstraintDistanceSQ)
	{
		float CapsuleToBoneDist = 0.0f;
		FVector CapsuleToBoneDir = FVector::ZeroVector;
		(ClosestOnBone - ClosestOnCapsule).ToDirectionAndLength(OUT CapsuleToBoneDir, OUT CapsuleToBoneDist);

		const float PenetrationDepth = ConstraintDistance - CapsuleToBoneDist;
		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1 + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		else
		{
			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const float DeltaLambda = PenetrationDepth / Denom;
			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		return true;
	}
	return false;
}

template <typename T>
void FLKAnimVerletConstraint_Capsule::CheckCapsuleCapsule(const T& CurPair, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
		return;

	verify(CurPair.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
	FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
	if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
	{
		CheckCapsuleSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
		return;
	}

	verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
	FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
	CheckCapsuleCapsule(CurVerletBone, ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
}

void FLKAnimVerletConstraint_Capsule::CheckCapsuleCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;

	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurPair) {
			verify(CurPair.Type == ELKAnimVerletBpDataCategory::Pair);
			CheckCapsuleCapsule(CurPair, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, CurPair.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < BonePairs->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
			CheckCapsuleCapsule(CurPair, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, i);
		}
	}
}

bool FLKAnimVerletConstraint_Capsule::CheckCapsuleTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
	/// 1) Find closest points between capsule axis segment and triangle
	FVector Pc = FVector::ZeroVector;
	FVector Qt = FVector::ZeroVector;
	float WA = 0.0f;
	float WB = 0.0f;
	float WC = 0.0f;
	float DistSQ = 0.0f;
	LKAnimVerletUtil::ClosestPointsCapsuleSegTriangle(OUT Pc, OUT Qt, OUT WA, OUT WB, OUT WC, OUT DistSQ,
													  CapsuleStart, CapsuleEnd, BoneA.Location, BoneB.Location, BoneC.Location);

	const float Dist = FMath::Sqrt(FMath::Max(DistSQ, 0.0f));

	/// Consider triangle thickness
	const float TriThickness = FMath::Max3(BoneA.Thickness, BoneB.Thickness, BoneC.Thickness);
	const float Target = Radius + TriThickness;

	/// CollisionNormal
	FVector N = FVector::ZeroVector;
	if (Dist > KINDA_SMALL_NUMBER)
	{
		N = (Pc - Qt) / Dist;	///from triangle to capsule axis
	}
	else
	{
		// Fallback: triangle normal if possible
		const FVector TriN = (BoneB.Location - BoneA.Location).Cross(BoneC.Location - BoneA.Location);
		N = (TriN.SizeSquared() > KINDA_SMALL_NUMBER) ? TriN.GetSafeNormal() : FVector::UpVector;
	}

	/// if penetrate then C < 0
	const float Cval = Dist - Target;
	if (Cval >= 0.0f)
		return false;

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

		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * N);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * N);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * N);
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
		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * N);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * N);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * N);
	}
	return true;
}

template <typename T>
void FLKAnimVerletConstraint_Capsule::CheckCapsuleTriangle(const T& CurTriangle, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneA.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneA.AnimVerletBoneIndex])
		return;
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneB.AnimVerletBoneIndex])
		return;
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneC.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneC.AnimVerletBoneIndex])
		return;

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
		CheckCapsuleSphere(IN OUT AVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
		CheckCapsuleSphere(IN OUT BVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
		CheckCapsuleSphere(IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
		return;
	}

	CheckCapsuleTriangle(IN OUT AVerletBone, IN OUT BVerletBone, IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, LambdaIndex);
}

void FLKAnimVerletConstraint_Capsule::CheckCapsuleTriangle(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;

	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurTriangle) {
			verify(CurTriangle.Type == ELKAnimVerletBpDataCategory::Triangle);
			CheckCapsuleTriangle(CurTriangle, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, CurTriangle.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < BoneTriangles->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*BoneTriangles)[i];
			CheckCapsuleTriangle(CurTriangle, DeltaTime, bInitialUpdate, bFinalize, CapsuleStart, CapsuleEnd, i);
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Box
///=========================================================================================================================================
FLKAnimVerletConstraint_Box::FLKAnimVerletConstraint_Box(const FVector& InLocation, const FQuat& InRot, const FVector& InHalfExtents, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: Location(InLocation)
	, Rotation(InRot)
	, HalfExtents(InHalfExtents)
	, Bones(InCollisionInput.Bones)
	, ExcludeBones(InCollisionInput.ExcludeBones)
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

void FLKAnimVerletConstraint_Box::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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
			CheckBoxCapsule(DeltaTime, bInitialUpdate, bFinalize);
		else
			CheckBoxTriangle(DeltaTime, bInitialUpdate, bFinalize);
	}
	else
	{
		CheckBoxSphere(DeltaTime, bInitialUpdate, bFinalize);
	}
}

FLKAnimVerletBound FLKAnimVerletConstraint_Box::MakeBound() const
{
	const FVector AxisXAbs = Rotation.GetAxisX().GetAbs();
	const FVector AxisYAbs = Rotation.GetAxisY().GetAbs();
	const FVector AxisZAbs = Rotation.GetAxisZ().GetAbs();

	const FVector AabbMin(AxisXAbs * -HalfExtents.X + AxisYAbs * -HalfExtents.Y + AxisZAbs * -HalfExtents.Z + Location);
	const FVector AabbMax(AxisXAbs * HalfExtents.X + AxisYAbs * HalfExtents.Y + AxisZAbs * HalfExtents.Z + Location);
	return FLKAnimVerletBound::MakeBoundFromMinMax(AabbMin, AabbMax);
}

bool FLKAnimVerletConstraint_Box::IntersectOriginAabbSphere(OUT FVector& OutCollisionNormal, OUT float& OutPenetrationDepth, IN OUT FLKAnimVerletBone& CurVerletBone, const FVector& SphereLocation)
{
	if (FMath::Abs(SphereLocation.X) < HalfExtents.X + CurVerletBone.Thickness &&
		FMath::Abs(SphereLocation.Y) < HalfExtents.Y + CurVerletBone.Thickness &&
		FMath::Abs(SphereLocation.Z) < HalfExtents.Z + CurVerletBone.Thickness)
	{
		const FVector MaxDistToSurface = SphereLocation - HalfExtents;
		const FVector MinDistsToSurface = -HalfExtents - SphereLocation;

		/// Determine closest distance and normal to box surface(PenetrationDepth is negative in this overlap case)
		float ClosestPenetrationDepthToSurface = 0.0f;
		FVector NormalToSurface = FVector::ZeroVector;
		const FVector ComponentMax = MaxDistToSurface.ComponentMax(MinDistsToSurface);
		if (ComponentMax.X > ComponentMax.Y)
		{
			if (ComponentMax.X > ComponentMax.Z)
			{
				ClosestPenetrationDepthToSurface = ComponentMax.X;
				NormalToSurface.X = MaxDistToSurface.X > MinDistsToSurface.X ? 1.0f : -1.0f;
			}
			else
			{
				ClosestPenetrationDepthToSurface = ComponentMax.Z;
				NormalToSurface.Z = MaxDistToSurface.Z > MinDistsToSurface.Z ? 1.0f : -1.0f;
			}
		}
		else
		{
			if (ComponentMax.Y > ComponentMax.Z)
			{
				ClosestPenetrationDepthToSurface = ComponentMax.Y;
				NormalToSurface.Y = MaxDistToSurface.Y > MinDistsToSurface.Y ? 1.0f : -1.0f;
			}
			else
			{
				ClosestPenetrationDepthToSurface = ComponentMax.Z;
				NormalToSurface.Z = MaxDistToSurface.Z > MinDistsToSurface.Z ? 1.0f : -1.0f;
			}
		}
		verify(NormalToSurface.IsNormalized());

		OutCollisionNormal = NormalToSurface;
		OutPenetrationDepth = -(ClosestPenetrationDepthToSurface - CurVerletBone.Thickness);
		return true;
	}
	return false;
}

bool FLKAnimVerletConstraint_Box::IntersectObbSphere(OUT FVector& OutCollisionNormal, OUT float& OutPenetrationDepth, IN OUT FLKAnimVerletBone& CurVerletBone, const FVector& SphereLocation, const FQuat& InvRotation)
{
	if (CurVerletBone.IsPinned())
		return false;

	const FVector BoneLocationInBoxLocal = InvRotation.RotateVector(SphereLocation - Location);
	if (IntersectOriginAabbSphere(OUT OutCollisionNormal, OUT OutPenetrationDepth, IN OUT CurVerletBone, BoneLocationInBoxLocal))
	{
		OutCollisionNormal = Rotation.RotateVector(OutCollisionNormal);
		return true;
	}
	return false;
}

bool FLKAnimVerletConstraint_Box::CheckBoxSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FVector& SphereLocation, const FQuat& InvRotation, int32 LambdaIndex)
{
	float PenetrationDepth = 0.0f;
	FVector CollisionNormal = FVector::ZeroVector;
	if (IntersectObbSphere(OUT CollisionNormal, OUT PenetrationDepth, CurVerletBone, SphereLocation, InvRotation))
	{
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

			const FVector NewLocation = CurVerletBone.Location + CollisionNormal * DeltaLambda;
			CurVerletBone.Location = NewLocation;
		}
		else
		{
			const FVector NewLocation = CurVerletBone.Location + CollisionNormal * PenetrationDepth;
			CurVerletBone.Location = NewLocation;
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Box::CheckBoxSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(LambdaIndex) && ExcludeBones[LambdaIndex])
		return;

	FLKAnimVerletBone& CurVerletBone = (*Bones)[LambdaIndex];
	CheckBoxSphere(CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, CurVerletBone.Location, InvRotation, LambdaIndex);
}

void FLKAnimVerletConstraint_Box::CheckBoxSphere(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const FQuat InvRotation = Rotation.Inverse();

	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurUserData) {
			verify(CurUserData.Type == ELKAnimVerletBpDataCategory::Bone);
			CheckBoxSphere(DeltaTime, bInitialUpdate, bFinalize, InvRotation, CurUserData.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < Bones->Num(); ++i)
		{
			CheckBoxSphere(DeltaTime, bInitialUpdate, bFinalize, InvRotation, i);
		}
	}
}

bool FLKAnimVerletConstraint_Box::CheckBoxCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	const FVector ParentBoneLocationInBoxLocal = InvRotation.RotateVector(ParentVerletBone.Location - Location);
	const FVector CurBoneLocationInBoxLocal = InvRotation.RotateVector(CurVerletBone.Location - Location);

	float SegT = 0.0f;
	FVector SegPtL = FVector::ZeroVector;
	FVector	BoxPtL = FVector::ZeroVector;
	LKAnimVerletUtil::ClosestPtSegmentAABB(OUT SegT, OUT SegPtL, OUT BoxPtL, ParentBoneLocationInBoxLocal, CurBoneLocationInBoxLocal, HalfExtents);

	const FVector DeltaL = SegPtL - BoxPtL;
	const float DistSqr = DeltaL.SizeSquared();
	const float Dist = FMath::Sqrt(DistSqr);

	if (Dist > CurVerletBone.Thickness)
		return false;

	const float PenetrationDepth = CurVerletBone.Thickness - Dist;

	FVector NormalInLocal = FVector::ZeroVector;
	if (Dist > KINDA_SMALL_NUMBER)
	{
		NormalInLocal = DeltaL / Dist; /// box -> capsule
	}
	else
	{
		float DX = HalfExtents.X - FMath::Abs(SegPtL.X);
		float DY = HalfExtents.Y - FMath::Abs(SegPtL.Y);
		float DZ = HalfExtents.Z - FMath::Abs(SegPtL.Z);

		if (DX <= DY && DX <= DZ)
			NormalInLocal = FVector((SegPtL.X >= 0.f) ? 1.f : -1.f, 0, 0);
		else if (DY <= DX && DY <= DZ)
			NormalInLocal = FVector(0, (SegPtL.Y >= 0.f) ? 1.f : -1.f, 0);
		else
			NormalInLocal = FVector(0, 0, (SegPtL.Z >= 0.f) ? 1.f : -1.f);
	}

	const FVector ContactPointOnBoxL = BoxPtL;
	const FVector ContactPointOnCapsuleL = SegPtL - NormalInLocal * CurVerletBone.Thickness;
	const FVector CollisionNormal = Rotation.RotateVector(NormalInLocal);

	const FVector ClosestOnBox = Rotation.RotateVector(ContactPointOnBoxL) + Location;
	const FVector ClosestOnBone = Rotation.RotateVector(ContactPointOnCapsuleL) + Location;

	FVector DirFromParent = FVector::ZeroVector;
	float DistFromParent = 0.0f;
	(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);

	if (bUseXPBDSolver && bFinalize == false)
	{
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return false;

		///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
		///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
		///if (ParentVerletBone.IsPinned())
		///	ChildMoveAlpha = 1.0f;
		///if (CurVerletBone.IsPinned())
		///	ChildMoveAlpha = 0.0f;
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
		const double Denom = (W0 + W1 + Alpha);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		if (ParentVerletBone.IsPinned() == false)
			ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * B0 * ParentVerletBone.InvMass);
		if (CurVerletBone.IsPinned() == false)
			CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * B1 * CurVerletBone.InvMass);
	}
	else
	{
		///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
		///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
		///if (ParentVerletBone.IsPinned())
		///	ChildMoveAlpha = 1.0f;
		///if (CurVerletBone.IsPinned())
		///	ChildMoveAlpha = 0.0f;
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
		const double Denom = (W0 + W1);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const float DeltaLambda = PenetrationDepth / Denom;
		if (ParentVerletBone.IsPinned() == false)
			ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * B0 * ParentVerletBone.InvMass);
		if (CurVerletBone.IsPinned() == false)
			CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * B1 * CurVerletBone.InvMass);
	}
	return true;
}

bool FLKAnimVerletConstraint_Box::CheckBoxBox(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	/// Consider BoneLine to OBB
	FVector DirFromParent = FVector::ZeroVector;
	float DistFromParent = 0.0f;
	(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);
	const FVector BoneBoxLocation = (CurVerletBone.Location + ParentVerletBone.Location) * 0.5f;

	const FVector BoxAxisX = Rotation.GetAxisX();
	const FVector BoxAxisY = Rotation.GetAxisY();
	const FVector BoxAxisZ = Rotation.GetAxisZ();
	const FVector BoxAxes[3]{ BoxAxisX, BoxAxisY, BoxAxisZ };

	float MinCos = TNumericLimits<float>::Max();
	FVector MaxAngleAxis = FVector::ZeroVector;
	for (int32 i = 0; i < 3; ++i)
	{
		const float CurCos = FMath::Abs(DirFromParent.Dot(BoxAxes[i]));
		if (CurCos < MinCos)
		{
			MinCos = CurCos;
			MaxAngleAxis = BoxAxes[i];
		}
	}

	const FQuat BoneBoxQuat = FRotationMatrix::MakeFromZX(DirFromParent, MaxAngleAxis).ToQuat();	///Maintain z axis
	const FVector BoneBoxAxisX = BoneBoxQuat.GetAxisX();
	const FVector BoneBoxAxisY = BoneBoxQuat.GetAxisY();
	const FVector BoneBoxAxisZ = BoneBoxQuat.GetAxisZ();
	const FVector BoneBoxAxes[3]{ BoneBoxAxisX, BoneBoxAxisY, BoneBoxAxisZ };
	const FVector BoneBoxHalfExtents(CurVerletBone.Thickness, CurVerletBone.Thickness, (DistFromParent * 0.5f + CurVerletBone.Thickness));

	const FVector SATAxes[15] = { BoxAxisX, BoxAxisY, BoxAxisZ, BoneBoxAxisX, BoneBoxAxisY, BoneBoxAxisZ,
		BoxAxisX.Cross(BoneBoxAxisX), BoxAxisX.Cross(BoneBoxAxisY), BoxAxisX.Cross(BoneBoxAxisZ),
		BoxAxisY.Cross(BoneBoxAxisX), BoxAxisY.Cross(BoneBoxAxisY), BoxAxisY.Cross(BoneBoxAxisZ),
		BoxAxisZ.Cross(BoneBoxAxisX), BoxAxisZ.Cross(BoneBoxAxisY), BoxAxisZ.Cross(BoneBoxAxisZ) };

	bool bNoCollision = false;
	float PenetrationDepth = TNumericLimits<float>::Max();
	FVector CollisionNormal = FVector::ZeroVector;
	for (const FVector& CurAxis : SATAxes)
	{
		if (FMath::IsNearlyZero(CurAxis.SizeSquared(), KINDA_SMALL_NUMBER))
			continue;

		const FVector NormalizedAxis = CurAxis.GetSafeNormal();

		float ProjA = 0.0f;
		for (int32 CurAxisI = 0; CurAxisI < 3; ++CurAxisI)
		{
			ProjA += FMath::Abs((BoxAxes[CurAxisI] * HalfExtents[CurAxisI]).Dot(NormalizedAxis));
		}

		float ProjB = 0.0f;
		for (int32 CurAxisI = 0; CurAxisI < 3; ++CurAxisI)
		{
			ProjB += FMath::Abs((BoneBoxAxes[CurAxisI] * BoneBoxHalfExtents[CurAxisI]).Dot(NormalizedAxis));
		}

		const float Distance = (BoneBoxLocation - Location).Dot(NormalizedAxis);
		const float Overlap = ProjA + ProjB - FMath::Abs(Distance);

		if (Overlap <= 0.0f)
		{
			bNoCollision = true;
			break;
		}

		if (Overlap < PenetrationDepth)
		{
			PenetrationDepth = Overlap;
			CollisionNormal = Distance < 0.0f ? -NormalizedAxis : NormalizedAxis;
		}
	}

	if (bNoCollision)
		return false;

	///const FVector ContactPointBox = Location + CollisionNormal * (HalfExtents.X + HalfExtents.Y + HalfExtents.Z);
	const FVector ContactPointBone = BoneBoxLocation - CollisionNormal * (BoneBoxHalfExtents.X + BoneBoxHalfExtents.Y + BoneBoxHalfExtents.Z);
	const FVector ClosestOnBone = FMath::ClosestPointOnSegment(ContactPointBone, CurVerletBone.Location, ParentVerletBone.Location);
	{
		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1 + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		else
		{
			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const float DeltaLambda = PenetrationDepth / Denom;
			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
	}
	return true;
}

template <typename T>
void FLKAnimVerletConstraint_Box::CheckBoxCapsule(const T& CurPair, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
		return;

	verify(CurPair.BoneB.IsValidBoneIndicator());
	verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
	FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
	if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
	{
		CheckBoxSphere(IN OUT CurVerletBone, DeltaTime, bInitialUpdate, bFinalize, CurVerletBone.Location, InvRotation, LambdaIndex);
		return;
	}

	verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
	FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

	///CheckBoxCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, InvRotation, LambdaIndex);

	/// OBB - OBB version
	/// Consider BoneLine to OBB
	CheckBoxBox(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bInitialUpdate, bFinalize, InvRotation, LambdaIndex);
}

void FLKAnimVerletConstraint_Box::CheckBoxCapsule(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	/// OBB - Capsule version
	const FQuat InvRotation = Rotation.Inverse();

	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurPair) {
			verify(CurPair.Type == ELKAnimVerletBpDataCategory::Pair);
			CheckBoxCapsule(CurPair, DeltaTime, bInitialUpdate, bFinalize, InvRotation, CurPair.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < BonePairs->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
			CheckBoxCapsule(CurPair, DeltaTime, bInitialUpdate, bFinalize, InvRotation, i);
		}
	}
}

bool FLKAnimVerletConstraint_Box::CheckBoxTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	/// Transform triangle vertices into box-local (AABB space)
	const FVector AInLocal = InvRotation.RotateVector(BoneA.Location - Location);
	const FVector BInLocal = InvRotation.RotateVector(BoneB.Location - Location);
	const FVector CInLocal = InvRotation.RotateVector(BoneC.Location - Location);

	/// Consider triangle thickness
	const float TriThickness = FMath::Max3(BoneA.Thickness, BoneB.Thickness, BoneC.Thickness);

	/// Expanded box (triangle thickness as margin)
	const FVector He = HalfExtents + FVector(TriThickness);

	/// Choose a single contact point on triangle: closest to box center (origin) in box-local (This gives us WA/WB/WC for gradient distribution)
	float WA = 0.0f;
	float WB = 0.0f;
	float WC = 0.0f;
	const FVector QtL = LKAnimVerletUtil::ClosestPointOnTriangleWeights(OUT WA, OUT WB, OUT WC, FVector::ZeroVector, AInLocal, BInLocal, CInLocal);

	/// Signed distance from QtL to expanded AABB
	float SignedDist = 0.0f;;
	FVector Nlocal = FVector::ZeroVector;
	FVector ClosestL = FVector::ZeroVector;
	LKAnimVerletUtil::SignedDistancePointAABB(OUT SignedDist, OUT Nlocal, OUT ClosestL, QtL, He);

	/// Constraint: C = SignedDist (outside => +, inside => -)
	if (SignedDist >= 0.0f)
		return false;

	const float Cval = -SignedDist;

	/// Normal in world space (box local -> world)
	const FQuat BoxRotation = InvRotation.Inverse();
	const FVector Nworld = BoxRotation.RotateVector(Nlocal).GetSafeNormal();
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

		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * Nworld);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * Nworld);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * Nworld);
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
		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (-WA * Nworld);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (-WB * Nworld);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (-WC * Nworld);
	}
	return true;
}

template <typename T>
void FLKAnimVerletConstraint_Box::CheckBoxTriangle(const T& CurTriangle, float DeltaTime, bool bInitialUpdate, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneA.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneA.AnimVerletBoneIndex])
		return;
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneB.AnimVerletBoneIndex])
		return;
	if (ExcludeBones.IsValidIndex(CurTriangle.BoneC.AnimVerletBoneIndex) && ExcludeBones[CurTriangle.BoneC.AnimVerletBoneIndex])
		return;

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
		CheckBoxSphere(IN OUT AVerletBone, DeltaTime, bInitialUpdate, bFinalize, AVerletBone.Location, InvRotation, LambdaIndex);
		CheckBoxSphere(IN OUT BVerletBone, DeltaTime, bInitialUpdate, bFinalize, BVerletBone.Location, InvRotation, LambdaIndex);
		CheckBoxSphere(IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, CVerletBone.Location, InvRotation, LambdaIndex);
		return;
	}

	CheckBoxTriangle(IN OUT AVerletBone, IN OUT BVerletBone, IN OUT CVerletBone, DeltaTime, bInitialUpdate, bFinalize, InvRotation, LambdaIndex);
}

void FLKAnimVerletConstraint_Box::CheckBoxTriangle(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	const FQuat InvRotation = Rotation.Inverse();

	if (bUseBroadphase)
	{
		verify(BroadphaseContainer != nullptr);

		const FLKAnimVerletBound MyBound = MakeBound();
		BroadphaseContainer->QueryAABB(MyBound, [&](const LKAnimVerletBVH<>::LKBvhID CurID, const FLKAnimVerletBpData& CurTriangle) {
			verify(CurTriangle.Type == ELKAnimVerletBpDataCategory::Triangle);
			CheckBoxTriangle(CurTriangle, DeltaTime, bInitialUpdate, bFinalize, InvRotation, CurTriangle.ListIndex);
			return true;
		});
	}
	else
	{
		for (int32 i = 0; i < BoneTriangles->Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorTriangle& CurTriangle = (*BoneTriangles)[i];
			CheckBoxTriangle(CurTriangle, DeltaTime, bInitialUpdate, bFinalize, InvRotation, i);
		}
	}
}
///=========================================================================================================================================

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

			CurVerletBone.Location += (PlaneNormal * DeltaLambda);
		}
		else
		{
			CurVerletBone.Location += (PlaneNormal * PenetrationDepth);
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
		if (bUseXPBDSolver && bFinalize == false)
		{
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return false;

			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1 + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (PlaneNormal * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (PlaneNormal * DeltaLambda * B1 * CurVerletBone.InvMass);
		}
		else
		{
			///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			///if (ParentVerletBone.IsPinned())
			///	ChildMoveAlpha = 1.0f;
			///if (CurVerletBone.IsPinned())
			///	ChildMoveAlpha = 0.0f;
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
			const double Denom = (W0 + W1);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const float DeltaLambda = PenetrationDepth / Denom;
			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (PlaneNormal * DeltaLambda * B0 * ParentVerletBone.InvMass);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (PlaneNormal * DeltaLambda * B1 * CurVerletBone.InvMass);
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

		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (WA * N);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (WB * N);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (WC * N);
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
		BoneA.Location = BoneA.Location + (BoneA.InvMass * DeltaLambda) * (WA * N);
		BoneB.Location = BoneB.Location + (BoneB.InvMass * DeltaLambda) * (WB * N);
		BoneC.Location = BoneC.Location + (BoneC.InvMass * DeltaLambda) * (WC * N);
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
		///float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (HitResult.Location - ParentVerletBone.Location).Size() / DistFromParent;
		///ChildMoveAlpha = FMath::Clamp(ChildMoveAlpha, 0.0f, 1.0f);
		///ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
		///if (ParentVerletBone.IsPinned() || CurVerletBone.IsPinned())
		///	ChildMoveAlpha = 1.0f;
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