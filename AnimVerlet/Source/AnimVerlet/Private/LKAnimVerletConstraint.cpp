#include "LKAnimVerletConstraint.h"

#include "LKAnimVerletBone.h"

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Pin
///=========================================================================================================================================
void FLKAnimVerletConstraint_Pin::Update(float DeltaTime, bool bFinalize)
{
	verify(Bone != nullptr);

	if (FMath::IsNearlyZero(PinMargin))
	{
		Bone->Location = Bone->PoseLocation;
	}
	else
	{
		/// Calculate the distance
		FVector Direction = FVector::ZeroVector;
		float Distance = 0.0f;
		(Bone->Location - Bone->PoseLocation).ToDirectionAndLength(OUT Direction, OUT Distance);

		/// Adjust distance constraint
		if (Distance > PinMargin)
		{
			Bone->Location = Bone->PoseLocation + Direction * PinMargin;
		}
	}
}

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Distance
///=========================================================================================================================================
FLKAnimVerletConstraint_Distance::FLKAnimVerletConstraint_Distance(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, bool bInUseXPBDSolver, double InStiffness, bool bInStretchEachBone, float InStretchStrength)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, bStretchEachBone(bInStretchEachBone)
	, StretchStrength(InStretchStrength)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	Length = (BoneB->PoseLocation - BoneA->PoseLocation).Size();
	Lambda = 0.0f;

	bUseXPBDSolver = bInUseXPBDSolver;
	if (bUseXPBDSolver)
		Compliance = InStiffness;
	else
		Stiffness = static_cast<float>(InStiffness);
}

void FLKAnimVerletConstraint_Distance::Update(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	/// Update length
	FVector PoseDirection = FVector::ZeroVector;
	(BoneB->PoseLocation - BoneA->PoseLocation).ToDirectionAndLength(OUT PoseDirection, OUT Length);

	/// Calculate the distance
	FVector Direction = FVector::ZeroVector;
	float Distance = 0.0f;
	(BoneB->Location - BoneA->Location).ToDirectionAndLength(OUT Direction, OUT Distance);

	/// XPBD
	if (bUseXPBDSolver)
	{
		const float C = Distance - Length;
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double DeltaLambda = -(C + Alpha * Lambda) / (BoneA->InvMass + BoneB->InvMass + Alpha);
		Lambda += DeltaLambda;

		if (bStretchEachBone)
			Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

		/// Adjust distance constraint
		const FVector DiffDir = (Direction * DeltaLambda);
		BoneA->Location -= (DiffDir * BoneA->InvMass);
		BoneB->Location += (DiffDir * BoneB->InvMass);
	}
	/// PBD
	else
	{
		/// Calculate the resting distance
		const float Diff = ((Length - Distance) / Distance) * Stiffness;
		
		if (bStretchEachBone)
			Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();
		
		/// Adjust distance constraint
		const FVector DiffDir = Direction * Diff * 0.5f;
		BoneA->Location -= (DiffDir * BoneA->InvMass);
		BoneB->Location += (DiffDir * BoneB->InvMass);
	}
}

void FLKAnimVerletConstraint_Distance::PostUpdate(float DeltaTime)
{
	Lambda = 0.0f;
}

void FLKAnimVerletConstraint_Distance::ResetSimulation()
{
	Lambda = 0.0f;
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_IsometricBending
///=========================================================================================================================================
FLKAnimVerletConstraint_IsometricBending::FLKAnimVerletConstraint_IsometricBending(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, FLKAnimVerletBone* InBoneD, bool bInUseXPBDSolver, float InStiffness)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, BoneC(InBoneC)
	, BoneD(InBoneD)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);
	verify(BoneD != nullptr);

	bUseXPBDSolver = bInUseXPBDSolver;
	if (bUseXPBDSolver)
		Compliance = InStiffness;
	else
		Stiffness = InStiffness;

	CalculateQMatrix(Q, BoneA, BoneB, BoneC, BoneD);
	RestAngle = CalculateRestAngle(BoneA, BoneB, BoneC, BoneD);
}

void FLKAnimVerletConstraint_IsometricBending::CalculateQMatrix(float InQ[4][4], FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, FLKAnimVerletBone* InBoneD)
{
	verify(InBoneA != nullptr);
	verify(InBoneB != nullptr);
	verify(InBoneC != nullptr);
	verify(InBoneD != nullptr);

	const FVector A = InBoneB->Location - InBoneA->Location;
	const FVector B = InBoneC->Location - InBoneA->Location;
	const FVector C = InBoneD->Location - InBoneA->Location;

	const float Area = A.Cross(B).Length() / 2.0f;
	for (int32 i = 0; i < 4; ++i)
	{
		for (int32 j = 0; j < 4; ++j)
		{
			if (i == j)
			{
				InQ[i][j] = 2.0f / Area;
			}
			else
			{
				const FVector V1 = (i == 0) ? A : (i == 1) ? B : C;
				const FVector V2 = (j == 0) ? A : (j == 1) ? B : C;
				InQ[i][j] = -2.0f * V1.Dot(V2) / (4.0f * Area);
			}
		}
	}

	/*for (int32 i = 0; i < 4; ++i)
	{
		for (int32 j = 0; j < 4; ++j)
		{
			if (i == j)
			{
				InQ[i][j] = 1.0f;
			}
			else
			{
				InQ[i][j] = -1.0f/3.0f;
			}
		}
	}*/
}

float FLKAnimVerletConstraint_IsometricBending::CalculateRestAngle(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, FLKAnimVerletBone* InBoneD)
{
	verify(InBoneA != nullptr);
	verify(InBoneB != nullptr);
	verify(InBoneC != nullptr);
	verify(InBoneD != nullptr);

	const FVector Vec1 = InBoneB->Location - InBoneA->Location;
	const FVector Vec2 = InBoneC->Location - InBoneA->Location;
	const FVector Vec3 = InBoneD->Location - InBoneA->Location;

	const float Angle1 = FMath::Atan2(Vec2.Y, Vec2.X) - FMath::Atan2(Vec1.Y, Vec1.X);
	const float Angle2 = FMath::Atan2(Vec3.Y, Vec3.X) - FMath::Atan2(Vec1.Y, Vec1.X);
	return Angle2 - Angle1;
}

void FLKAnimVerletConstraint_IsometricBending::Update(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);
	verify(BoneD != nullptr);

	FLKAnimVerletBone* Bones[4] = { BoneA, BoneB, BoneC, BoneD };
	float C = 0.0f;
	for (int32 i = 0; i < 4; ++i)
	{
		for (int32 j = 0; j < 4; ++j)
		{
			C += Bones[i]->Location.Dot(Bones[j]->Location) * Q[i][j];
		}
	}
	C -= RestAngle;

	FVector Grad[4] = {};
	for (int32 i = 0; i < 4; ++i)
	{
		Grad[i] = FVector::ZeroVector;
		for (int32 j = 0; j < 4; ++j)
		{
			Grad[i] = Grad[i] + Bones[j]->Location * (2.0f * Q[i][j]);
		}
	}

	float Sum = 0.0f;
	for (int32 i = 0; i < 4; ++i)
	{
		Sum += Bones[i]->InvMass * Grad[i].Dot(Grad[i]);
	}

	/// XPBD
	if (bUseXPBDSolver)
	{
		const float Alpha = Compliance / (DeltaTime * DeltaTime);
		const float DeltaLambda = -(C + Alpha * Lambda) / (Sum + Alpha);
		Lambda += DeltaLambda;

		for (int32 i = 0; i < 4; ++i)
		{
			Bones[i]->Location += Grad[i] * (DeltaLambda * Bones[i]->InvMass);
		}
	}
	/// PBD
	else
	{
		const float DeltaLambda = C * Stiffness;
		for (int32 i = 0; i < 4; ++i)
		{
			Bones[i]->Location += Grad[i] * (DeltaLambda * Bones[i]->InvMass);
		}
	}
}

void FLKAnimVerletConstraint_IsometricBending::PostUpdate(float DeltaTime)
{
	Lambda = 0.0f;
}

void FLKAnimVerletConstraint_IsometricBending::ResetSimulation()
{
	Lambda = 0.0f;
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Straighten
///=========================================================================================================================================
FLKAnimVerletConstraint_Straighten::FLKAnimVerletConstraint_Straighten(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, float InStraightenStrength, bool bInStraightenCenterBone)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, BoneC(InBoneC)
	, StraightenStrength(InStraightenStrength)
	, bStraightenCenterBone(bInStraightenCenterBone)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);
}

void FLKAnimVerletConstraint_Straighten::Update(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);

	float BToALength = 0.0f;
	FVector BToADir = FVector::ZeroVector;
	(BoneA->Location - BoneB->Location).ToDirectionAndLength(OUT BToADir, OUT BToALength);

	float BToCLength = 0.0f;
	FVector BToCDir = FVector::ZeroVector;
	(BoneC->Location - BoneB->Location).ToDirectionAndLength(OUT BToCDir, OUT BToCLength);

	const FVector AToBDir = -BToADir;
	if (bStraightenCenterBone)
	{
		/*const FVector StraightenedVec = (BToADir * BToALength) + (BToCDir * BToCLength) * 0.5f;

		float StraightenLength = 0.0f;
		FVector StraightenedDir = FVector::ZeroVector;
		StraightenedVec.ToDirectionAndLength(OUT StraightenedDir, OUT StraightenLength);

		const float StraightenDist = FMath::Lerp(0.0f, StraightenLength, FMath::Clamp(StraightenStrength * DeltaTime, 0.0f, 1.0f));
		BoneB->Location += StraightenedDir * StraightenDist;

		float NewBToALength = 0.0f;
		FVector NewBToADir = FVector::ZeroVector;
		(BoneA->Location - BoneB->Location).ToDirectionAndLength(OUT NewBToADir, OUT NewBToALength);

		float NewBToCLength = 0.0f;
		FVector NewBToCDir = FVector::ZeroVector;
		(BoneC->Location - BoneB->Location).ToDirectionAndLength(OUT NewBToCDir, OUT NewBToCLength);

		BoneA->Location = BoneB->Location * NewBToADir * BToALength;
		BoneC->Location = BoneB->Location * NewBToCDir * BToCLength;*/

		const FVector StraightenedDirC = FMath::Lerp(BToCDir, AToBDir, StraightenStrength * DeltaTime);
		BoneC->Location = BoneB->Location + StraightenedDirC * BToCLength;

		float NewCToBLength = 0.0f;
		FVector NewCToBDir = FVector::ZeroVector;
		(BoneB->Location - BoneC->Location).ToDirectionAndLength(OUT NewCToBDir, OUT NewCToBLength);

		const FVector AStraightenedDir = FMath::Lerp(BToADir, NewCToBDir, StraightenStrength * DeltaTime);
		BoneA->Location = BoneB->Location + AStraightenedDir * BToALength;
	}
	else
	{
		const FVector StraightenedDir = FMath::Lerp(BToCDir, AToBDir, StraightenStrength * DeltaTime);
		BoneC->Location = BoneB->Location + StraightenedDir * BToCLength;
	}
}
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletConstraint_FixedDistance
///=========================================================================================================================================
FLKAnimVerletConstraint_FixedDistance::FLKAnimVerletConstraint_FixedDistance(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, bool bInStretchEachBone, float InStretchStrength, bool bInAwayFromEachOther, float InLengthMargin)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, bStretchEachBone(bInStretchEachBone)
	, bAwayFromEachOther(bInAwayFromEachOther)
	, StretchStrength(InStretchStrength)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(InLengthMargin >= 0.0f);

	Length = (BoneB->PoseLocation - BoneA->PoseLocation).Size();
	LengthMargin = InLengthMargin;
}

void FLKAnimVerletConstraint_FixedDistance::Update(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	/// Update length
	FVector PoseDirection = FVector::ZeroVector;
	(BoneB->PoseLocation - BoneA->PoseLocation).ToDirectionAndLength(OUT PoseDirection, OUT Length);

	/// Calculate the distance
	FVector Direction = FVector::ZeroVector;
	float Distance = 0.0f;
	(BoneB->Location - BoneA->Location).ToDirectionAndLength(OUT Direction, OUT Distance);

	///if (bStretchEachBone)
	///	Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

	/// Adjust distance constraint
	if (Distance > Length + LengthMargin)
	{
		const float LengthWithMargin = Length + LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneA->Location + Direction * Distance * 0.5f;
			BoneB->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneA->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneB->Location = BoneA->Location + Direction * LengthWithMargin;
		}
	}
	else if (Distance < Length - LengthMargin)
	{
		const float LengthWithMargin = Length - LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneA->Location + Direction * Distance * 0.5f;
			BoneB->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneA->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneB->Location = BoneA->Location + Direction * LengthWithMargin;
		}
	}
}

void FLKAnimVerletConstraint_FixedDistance::BackwardUpdate(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	/// Update length
	FVector PoseDirection = FVector::ZeroVector;
	(BoneA->PoseLocation - BoneB->PoseLocation).ToDirectionAndLength(OUT PoseDirection, OUT Length);


	/// Calculate the distance
	FVector Direction = FVector::ZeroVector;
	float Distance = 0.0f;
	(BoneA->Location - BoneB->Location).ToDirectionAndLength(OUT Direction, OUT Distance);

	///if (bStretchEachBone)
	///	Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

	/// Adjust distance constraint
	if (Distance > Length + LengthMargin)
	{
		const float LengthWithMargin = Length + LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneB->Location + Direction * Distance * 0.5f;
			BoneA->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneB->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneA->Location = BoneB->Location + Direction * LengthWithMargin;
		}
	}
	else if (Distance < Length - LengthMargin)
	{
		const float LengthWithMargin = Length - LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneB->Location + Direction * Distance * 0.5f;
			BoneA->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneB->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneA->Location = BoneB->Location + Direction * LengthWithMargin;
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_BallSocket
///=========================================================================================================================================
FLKAnimVerletConstraint_BallSocket::FLKAnimVerletConstraint_BallSocket(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InGrandParentNullable, FLKAnimVerletBone* InParentNullable
																	   , float InAngleDegrees, bool bInUseXPBDSolver, double InCompliance)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, GrandParentBoneNullable(InGrandParentNullable)
	, ParentBoneNullable(InParentNullable)
	, AngleDegrees(InAngleDegrees)
	, bUseXPBDSolver(bInUseXPBDSolver)
	, Compliance(InCompliance)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
}

void FLKAnimVerletConstraint_BallSocket::Update(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	FVector BoneAToBoneB = FVector::ZeroVector;
	float BoneAToBoneBSize = 0.0f;
	(BoneB->Location - BoneA->Location).ToDirectionAndLength(OUT BoneAToBoneB, OUT BoneAToBoneBSize);

	FVector ConstraintDirection = FVector::ZeroVector;
	if (GrandParentBoneNullable != nullptr && ParentBoneNullable != nullptr)
	{
		const FVector GrandParentToParent = (ParentBoneNullable->Location - GrandParentBoneNullable->Location).GetSafeNormal();
		ConstraintDirection = GrandParentToParent;
	}
	else
	{
		const FVector PoseAToPoseB = (BoneB->PoseLocation - BoneA->PoseLocation).GetSafeNormal();
		ConstraintDirection = PoseAToPoseB;
	}

	const FVector RotationAxis = FVector::CrossProduct(ConstraintDirection, BoneAToBoneB);
	const float RotationAngle = FMath::Acos(FVector::DotProduct(ConstraintDirection, BoneAToBoneB));
	const float AngleDiff = FMath::RadiansToDegrees(RotationAngle) - AngleDegrees;
	if (AngleDiff > 0.0f)
	{
		if (bUseXPBDSolver && bFinalize == false)
		{
			const float C = -AngleDiff;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double DeltaLambda = -(C + Alpha * Lambda) / (BoneA->InvMass + BoneB->InvMass + Alpha);
			Lambda += DeltaLambda;

			const FVector ConstraintDir = BoneAToBoneB.RotateAngleAxis(-DeltaLambda, RotationAxis);
			BoneB->Location = BoneA->Location + (ConstraintDir * BoneAToBoneBSize);
		}
		else
		{
			const FVector ConstraintDir = BoneAToBoneB.RotateAngleAxis(-AngleDiff, RotationAxis);
			BoneB->Location = BoneA->Location + (ConstraintDir * BoneAToBoneBSize);
		}
	}
}
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletConstraint_Sphere
///=========================================================================================================================================
FLKAnimVerletConstraint_Sphere::FLKAnimVerletConstraint_Sphere(const FVector& InLocation, float InRadius, const FLKAnimVerletCollisionConstraintInput& InCollisionInput)
	: Location(InLocation)
	, Radius(InRadius)
	, Bones(InCollisionInput.Bones)
	, ExcludeBones(InCollisionInput.ExcludeBones)
	, bUseCapsuleCollisionForChain(InCollisionInput.bUseCapsuleCollisionForChain)
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, bUseXPBDSolver(InCollisionInput.bUseXPBDSolver)
	, Compliance(InCollisionInput.Compliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			Lambdas.Reserve(BonePairs->Num());
		}
		else
		{
			Lambdas.Reserve(Bones->Num());
		}
	}
}

void FLKAnimVerletConstraint_Sphere::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			if (Lambdas.Num() != BonePairs->Num())
				Lambdas.SetNum(BonePairs->Num(), false);
		}
		else
		{
			if (Lambdas.Num() != Bones->Num())
				Lambdas.SetNum(Bones->Num(), false);
		}
	}

	if (bUseCapsuleCollisionForChain)
		CheckSphereCapsule(DeltaTime, bFinalize);
	else
		CheckSphereSphere(DeltaTime, bFinalize);
}

bool FLKAnimVerletConstraint_Sphere::CheckSphereSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, int32 LambdaIndex)
{
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
			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
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

void FLKAnimVerletConstraint_Sphere::CheckSphereSphere(float DeltaTime, bool bFinalize)
{
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		CheckSphereSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, i);
	}
}

void FLKAnimVerletConstraint_Sphere::CheckSphereCapsule(float DeltaTime, bool bFinalize)
{
	for (int32 i = 0; i < BonePairs->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
		if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];		verify(CurPair.BoneB.IsValidBoneIndicator());
		if (CurPair.BoneA.IsValidBoneIndicator() == false)
		{
			CheckSphereSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, i);
			continue;
		}

		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
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
				double& CurLambda = Lambdas[i];
				const float C = -PenetrationDepth;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * DeltaLambda * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * ChildMoveAlpha);
			}
			else
			{
				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * PenetrationDepth * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * PenetrationDepth * ChildMoveAlpha);
			}
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
	, bUseCapsuleCollisionForChain(InCollisionInput.bUseCapsuleCollisionForChain)
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, bUseXPBDSolver(InCollisionInput.bUseXPBDSolver)
	, Compliance(InCollisionInput.Compliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			Lambdas.Reserve(BonePairs->Num());
		}
		else
		{
			Lambdas.Reserve(Bones->Num());
		}
	}
}

void FLKAnimVerletConstraint_Capsule::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			if (Lambdas.Num() != BonePairs->Num())
				Lambdas.SetNum(BonePairs->Num(), false);
		}
		else
		{
			if (Lambdas.Num() != Bones->Num())
				Lambdas.SetNum(Bones->Num(), false);
		}
	}

	if (bUseCapsuleCollisionForChain)
		CheckCapsuleCapsule(DeltaTime, bFinalize);
	else
		CheckCapsuleSphere(DeltaTime, bFinalize);
}

bool FLKAnimVerletConstraint_Capsule::CheckCapsuleSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
{
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
			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
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

void FLKAnimVerletConstraint_Capsule::CheckCapsuleSphere(float DeltaTime, bool bFinalize)
{
	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		CheckCapsuleSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, CapsuleStart, CapsuleEnd, i);
	}
}

void FLKAnimVerletConstraint_Capsule::CheckCapsuleCapsule(float DeltaTime, bool bFinalize)
{
	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;

	for (int32 i = 0; i < BonePairs->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
		if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];		verify(CurPair.BoneB.IsValidBoneIndicator());
		if (CurPair.BoneA.IsValidBoneIndicator() == false)
		{
			CheckCapsuleSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, CapsuleStart, CapsuleEnd, i);
			continue;
		}

		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
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
				double& CurLambda = Lambdas[i];
				const float C = -PenetrationDepth;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * ChildMoveAlpha);
			}
			else
			{
				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (CapsuleToBoneDir * PenetrationDepth * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * PenetrationDepth * ChildMoveAlpha);
			}
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
	, bUseCapsuleCollisionForChain(InCollisionInput.bUseCapsuleCollisionForChain)
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, bUseXPBDSolver(InCollisionInput.bUseXPBDSolver)
	, Compliance(InCollisionInput.Compliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			Lambdas.Reserve(BonePairs->Num());
		}
		else
		{
			Lambdas.Reserve(Bones->Num());
		}
	}
}

void FLKAnimVerletConstraint_Box::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			if (Lambdas.Num() != BonePairs->Num())
				Lambdas.SetNum(BonePairs->Num(), false);
		}
		else
		{
			if (Lambdas.Num() != Bones->Num())
				Lambdas.SetNum(Bones->Num(), false);
		}
	}

	if (bUseCapsuleCollisionForChain)
		CheckBoxCapsule(DeltaTime, bFinalize);
	else
		CheckBoxSphere(DeltaTime, bFinalize);
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
	const FVector BoneLocationInBoxLocal = InvRotation.RotateVector(SphereLocation - Location);
	if (IntersectOriginAabbSphere(OUT OutCollisionNormal, OUT OutPenetrationDepth, IN OUT CurVerletBone, BoneLocationInBoxLocal))
	{
		OutCollisionNormal = Rotation.RotateVector(OutCollisionNormal);
		return true;
	}
	return false;
}

bool FLKAnimVerletConstraint_Box::CheckBoxSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const FVector& SphereLocation, const FQuat& InvRotation, int32 LambdaIndex)
{
	float PenetrationDepth = 0.0f;
	FVector CollisionNormal = FVector::ZeroVector;
	if (IntersectObbSphere(OUT CollisionNormal, OUT PenetrationDepth, CurVerletBone, SphereLocation, InvRotation))
	{
		if (bUseXPBDSolver && bFinalize == false)
		{
			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
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

void FLKAnimVerletConstraint_Box::CheckBoxSphere(float DeltaTime, bool bFinalize)
{
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		CheckBoxSphere(CurVerletBone, DeltaTime, bFinalize, CurVerletBone.Location, InvRotation, i);
	}
}

void FLKAnimVerletConstraint_Box::CheckBoxCapsule(float DeltaTime, bool bFinalize)
{
	/// OBB - Capsule version
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < BonePairs->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
		if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];		verify(CurPair.BoneB.IsValidBoneIndicator());
		if (CurPair.BoneA.IsValidBoneIndicator() == false)
		{
			CheckBoxSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, CurVerletBone.Location, InvRotation, i);
			continue;
		}

		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		const FVector ParentBoneLocationInBoxLocal = InvRotation.RotateVector(ParentVerletBone.Location - Location);
		const FVector CurBoneLocationInBoxLocal = InvRotation.RotateVector(CurVerletBone.Location - Location);

		FVector ClosestDummyOnBone = FVector::ZeroVector;
		FVector ClosestX = FVector::ZeroVector;
		FMath::SegmentDistToSegment(-FVector::XAxisVector * HalfExtents.X, FVector::XAxisVector * HalfExtents.X, CurBoneLocationInBoxLocal, ParentBoneLocationInBoxLocal, OUT ClosestX, OUT ClosestDummyOnBone);

		FVector ClosestY = FVector::ZeroVector;
		FMath::SegmentDistToSegment(-FVector::YAxisVector * HalfExtents.Y, FVector::YAxisVector * HalfExtents.Y, CurBoneLocationInBoxLocal, ParentBoneLocationInBoxLocal, OUT ClosestY, OUT ClosestDummyOnBone);

		FVector ClosestZ = FVector::ZeroVector;
		FMath::SegmentDistToSegment(-FVector::ZAxisVector * HalfExtents.Z, FVector::ZAxisVector * HalfExtents.Z, CurBoneLocationInBoxLocal, ParentBoneLocationInBoxLocal, OUT ClosestZ, OUT ClosestDummyOnBone);

		const FVector ClosestOnBoxLocal(ClosestX.X, ClosestY.Y, ClosestZ.Z);
		const FVector ClosestOnBoneLocal = FMath::ClosestPointOnSegment(ClosestOnBoxLocal, CurBoneLocationInBoxLocal, ParentBoneLocationInBoxLocal);

		float PenetrationDepth = 0.0f;
		FVector CollisionNormal = FVector::ZeroVector;
		if (IntersectOriginAabbSphere(OUT CollisionNormal, OUT PenetrationDepth, CurVerletBone, ClosestOnBoneLocal))
		{
			CollisionNormal = Rotation.RotateVector(CollisionNormal);
			const FVector ClosestOnBone = Rotation.RotateVector(ClosestOnBoneLocal) + Location;

			FVector DirFromParent = FVector::ZeroVector;
			float DistFromParent = 0.0f;
			(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);

			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -PenetrationDepth;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * ChildMoveAlpha);
			}
			else
			{
				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * PenetrationDepth * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * PenetrationDepth * ChildMoveAlpha);
			}
		}
	}

	/// OBB - OBB version
	/*const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < BonePairs->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
		if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];		verify(CurPair.BoneB.IsValidBoneIndicator());
		if (CurPair.BoneA.IsValidBoneIndicator() == false)
		{
			CheckBoxSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, CurVerletBone.Location, InvRotation, i);
			continue;
		}

		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		/// Consider BoneLine to OBB
		FVector DirFromParent = FVector::ZeroVector;
		float DistFromParent = 0.0f;
		(CurVerletBone.Location - ParentVerletBone.Location).ToDirectionAndLength(OUT DirFromParent, OUT DistFromParent);
		const FVector BoneBoxLocation = (CurVerletBone.Location + ParentVerletBone.Location) * 0.5f;

		const FVector BoxAxisX = Rotation.GetAxisX();
		const FVector BoxAxisY = Rotation.GetAxisY();
		const FVector BoxAxisZ = Rotation.GetAxisZ();
		const FVector BoxAxes[3]{ BoxAxisX, BoxAxisY, BoxAxisZ };

		const FQuat BoneBoxQuat = FRotationMatrix::MakeFromZ(DirFromParent).ToQuat();
		const FVector BoneBoxAxisX = BoneBoxQuat.GetAxisX();
		const FVector BoneBoxAxisY = BoneBoxQuat.GetAxisY();
		const FVector BoneBoxAxisZ = BoneBoxQuat.GetAxisZ();
		const FVector BoneBoxAxes[3]{ BoneBoxAxisX, BoneBoxAxisY, BoneBoxAxisZ };
		const FVector BoneBoxHalfExtents(CurVerletBone.Thickness, CurVerletBone.Thickness, (DistFromParent + 2.0f * CurVerletBone.Thickness));

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
			continue;

		///const FVector ContactPointBox = Location + CollisionNormal * (HalfExtents.X + HalfExtents.Y + HalfExtents.Z);
		const FVector ContactPointBone = BoneBoxLocation - CollisionNormal * (BoneBoxHalfExtents.X + BoneBoxHalfExtents.Y + BoneBoxHalfExtents.Z);
		const FVector ClosestOnBone = FMath::ClosestPointOnSegment(ContactPointBone, CurVerletBone.Location, ParentVerletBone.Location);
		{
			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -PenetrationDepth;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * ChildMoveAlpha);
			}
			else
			{
				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * PenetrationDepth * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * PenetrationDepth * ChildMoveAlpha);
			}
		}
	}*/
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
	, BonePairs(InCollisionInput.SimulateBonePairIndicators)
	, bUseXPBDSolver(InCollisionInput.bUseXPBDSolver)
	, Compliance(InCollisionInput.Compliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			Lambdas.Reserve(BonePairs->Num());
		}
		else
		{
			Lambdas.Reserve(Bones->Num());
		}
	}
}

void FLKAnimVerletConstraint_Plane::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
	{
		if (bUseCapsuleCollisionForChain)
		{
			verify(BonePairs != nullptr);
			if (Lambdas.Num() != BonePairs->Num())
				Lambdas.SetNum(BonePairs->Num(), false);
		}
		else
		{
			if (Lambdas.Num() != Bones->Num())
				Lambdas.SetNum(Bones->Num(), false);
		}
	}

	if (bUseCapsuleCollisionForChain)
		CheckPlaneCapsule(DeltaTime, bFinalize);
	else
		CheckPlaneSphere(DeltaTime, bFinalize);
}

bool FLKAnimVerletConstraint_Plane::CheckPlaneSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex)
{
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
			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
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

void FLKAnimVerletConstraint_Plane::CheckPlaneSphere(float DeltaTime, bool bFinalize)
{
	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		CheckPlaneSphere(CurVerletBone, DeltaTime, bFinalize, bFinitePlane, InvRotation, i);
	}
}

void FLKAnimVerletConstraint_Plane::CheckPlaneCapsule(float DeltaTime, bool bFinalize)
{
	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < BonePairs->Num(); ++i)
	{
		const FLKAnimVerletBoneIndicatorPair& CurPair = (*BonePairs)[i];
		if (ExcludeBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) && ExcludeBones[CurPair.BoneB.AnimVerletBoneIndex])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];		verify(CurPair.BoneB.IsValidBoneIndicator());
		if (CurPair.BoneA.IsValidBoneIndicator() == false)
		{
			CheckPlaneSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, bFinitePlane, InvRotation, i);
			continue;
		}

		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

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
					continue;
			}

			const float PenetrationDepth = (CurVerletBone.Thickness - DistToPlane);
			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -PenetrationDepth;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;
			
				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)
			
				ParentVerletBone.Location = ParentVerletBone.Location + (PlaneNormal * DeltaLambda * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (PlaneNormal * DeltaLambda * ChildMoveAlpha);
			}
			else
			{
				float ChildMoveAlpha = (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
				ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

				ParentVerletBone.Location = ParentVerletBone.Location + (PlaneNormal * PenetrationDepth * (1.0f - ChildMoveAlpha));
				CurVerletBone.Location = CurVerletBone.Location + (PlaneNormal * PenetrationDepth * ChildMoveAlpha);
			}
		}
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

void FLKAnimVerletConstraint_World::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (WorldPtr.IsValid() == false || SelfComponentPtr.IsValid() == false)
		return;

	if (bUseCapsuleCollisionForChain)
		CheckWorldCapsule(DeltaTime, bFinalize);
	else
		CheckWorldSphere(DeltaTime, bFinalize);
}

bool FLKAnimVerletConstraint_World::CheckWorldSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const UWorld* World, 
													 const FCollisionQueryParams& CollisionQueryParams, const FTransform& ComponentTransform, int32 LambdaIndex)
{
	verify(World != nullptr);

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

void FLKAnimVerletConstraint_World::CheckWorldSphere(float DeltaTime, bool bFinalize)
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
		CheckWorldSphere(CurVerletBone, DeltaTime, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
	}
}

void FLKAnimVerletConstraint_World::CheckWorldCapsule(float DeltaTime, bool bFinalize)
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

		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];		verify(CurPair.BoneB.IsValidBoneIndicator());
		if (CurPair.BoneA.IsValidBoneIndicator() == false)
		{
			CheckWorldSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
			continue;
		}

		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

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
		}
	}
}
///=========================================================================================================================================