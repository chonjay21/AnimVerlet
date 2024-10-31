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
FLKAnimVerletConstraint_Sphere::FLKAnimVerletConstraint_Sphere(const FVector& InLocation, float InRadius, float InThickness, TArray<FLKAnimVerletBone>* InBones, 
															   const TExcludeBoneBits& InExcludeBones, bool bInUseXPBDSolver, double InCompliance)
	: Location(InLocation)
	, Radius(InRadius)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
	, bUseXPBDSolver(bInUseXPBDSolver)
	, Compliance(InCompliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
		Lambdas.Reserve(Bones->Num());
}

void FLKAnimVerletConstraint_Sphere::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver && Lambdas.Num() != Bones->Num())
		Lambdas.SetNum(Bones->Num(), false);

	const float ConstraintDistance = Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector SphereToBone = (CurVerletBone.Location - Location);
		const float SphereToBoneSQ = SphereToBone.SizeSquared();
		if (SphereToBoneSQ < ConstraintDistanceSQ)
		{
			const float SphereToBoneSize = FMath::Sqrt(SphereToBoneSQ);
			const FVector SphereToBoneDir = SphereToBoneSize > KINDA_SMALL_NUMBER ? (SphereToBone / SphereToBoneSize) : FVector::ZeroVector;

			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -ConstraintDistance;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				CurVerletBone.Location = Location + (SphereToBoneDir * DeltaLambda);
			}
			else
			{
				CurVerletBone.Location = Location + (SphereToBoneDir * ConstraintDistance);
			}
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Capsule
///=========================================================================================================================================
FLKAnimVerletConstraint_Capsule::FLKAnimVerletConstraint_Capsule(const FVector& InLocation, const FQuat& InRot, float InRadius, float InHalfHeight, float InThickness, 
																 TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones, bool bInUseXPBDSolver, double InCompliance)
	: Location(InLocation)
	, Rotation(InRot)
	, Radius(InRadius)
	, HalfHeight(InHalfHeight)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
	, bUseXPBDSolver(bInUseXPBDSolver)
	, Compliance(InCompliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
		Lambdas.Reserve(Bones->Num());
}

void FLKAnimVerletConstraint_Capsule::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver && Lambdas.Num() != Bones->Num())
		Lambdas.SetNum(Bones->Num(), false);

	const float ConstraintDistance = Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector ClosestOnCapsule = FMath::ClosestPointOnSegment(CurVerletBone.Location, CapsuleStart, CapsuleEnd);
		const float CapsuleToBoneSQ = (CurVerletBone.Location - ClosestOnCapsule).SizeSquared();
		if (CapsuleToBoneSQ < ConstraintDistanceSQ)
		{
			const FVector CapsuleToBoneDir = (CurVerletBone.Location - ClosestOnCapsule).GetSafeNormal();
			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -ConstraintDistance;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				CurVerletBone.Location = ClosestOnCapsule + (CapsuleToBoneDir * DeltaLambda);
			}
			else
			{
				CurVerletBone.Location = ClosestOnCapsule + (CapsuleToBoneDir * ConstraintDistance);
			}
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Box
///=========================================================================================================================================
FLKAnimVerletConstraint_Box::FLKAnimVerletConstraint_Box(const FVector& InLocation, const FQuat& InRot, const FVector& InHalfExtents, float InThickness, 
														 TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones, bool bInUseXPBDSolver, double InCompliance)
	: Location(InLocation)
	, Rotation(InRot)
	, HalfExtents(InHalfExtents)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
	, bUseXPBDSolver(bInUseXPBDSolver)
	, Compliance(InCompliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
		Lambdas.Reserve(Bones->Num());
}

void FLKAnimVerletConstraint_Box::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver && Lambdas.Num() != Bones->Num())
		Lambdas.SetNum(Bones->Num(), false);

	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector BoneLocationInBoxLocal = InvRotation.RotateVector(CurVerletBone.Location - Location);
		if (FMath::Abs(BoneLocationInBoxLocal.X) < HalfExtents.X + Thickness &&
			FMath::Abs(BoneLocationInBoxLocal.Y) < HalfExtents.Y + Thickness &&
			FMath::Abs(BoneLocationInBoxLocal.Z) < HalfExtents.Z + Thickness)
		{
			const FVector MaxDistToSurface = BoneLocationInBoxLocal - HalfExtents;
			const FVector MinDistsToSurface = -HalfExtents - BoneLocationInBoxLocal;

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

			const float CurDiff = (ClosestPenetrationDepthToSurface - Thickness);
			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -CurDiff;
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				const FVector NewLocation = CurVerletBone.Location - Rotation.RotateVector(NormalToSurface) * DeltaLambda;
				CurVerletBone.Location = NewLocation;
			}
			else
			{
				const FVector NewLocation = CurVerletBone.Location - Rotation.RotateVector(NormalToSurface) * CurDiff;
				CurVerletBone.Location = NewLocation;
			}
		}
	}
}

void FLKAnimVerletConstraint_Box::PostUpdate(float DeltaTime)
{
	Lambdas.Reset();
}

void FLKAnimVerletConstraint_Box::ResetSimulation() 
{ 
	Lambdas.Reset(); 
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Plane
///=========================================================================================================================================
FLKAnimVerletConstraint_Plane::FLKAnimVerletConstraint_Plane(const FVector& InPlaneBase, const FVector& InPlaneNormal, const FQuat& InRotation, const FVector2D& InPlaneHalfExtents, float InThickness, 
															 TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones, bool bInUseXPBDSolver, double InCompliance)
	: PlaneBase(InPlaneBase)
	, PlaneNormal(InPlaneNormal)
	, Rotation(InRotation)
	, PlaneHalfExtents(InPlaneHalfExtents)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
	, bUseXPBDSolver(bInUseXPBDSolver)
	, Compliance(InCompliance)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver)
		Lambdas.Reserve(Bones->Num());
}

void FLKAnimVerletConstraint_Plane::Update(float DeltaTime, bool bFinalize)
{
	verify(Bones != nullptr);

	if (bUseXPBDSolver && Lambdas.Num() != Bones->Num())
		Lambdas.SetNum(Bones->Num(), false);

	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const float DistToPlane = FVector::PointPlaneDist(CurVerletBone.Location, PlaneBase, PlaneNormal);
		if (DistToPlane < Thickness)
		{
			if (bFinitePlane)
			{
				const FVector ProjectedLocationOnPlane = CurVerletBone.Location - (DistToPlane * PlaneNormal);
				const FVector BoneLocationInPlaneLocal = InvRotation.RotateVector(ProjectedLocationOnPlane - PlaneBase);
				if (BoneLocationInPlaneLocal.X > PlaneHalfExtents.X + Thickness || BoneLocationInPlaneLocal.X < -PlaneHalfExtents.X + Thickness ||
					BoneLocationInPlaneLocal.Y > PlaneHalfExtents.Y + Thickness || BoneLocationInPlaneLocal.Y < -PlaneHalfExtents.Y + Thickness)
					continue;
			}

			if (bUseXPBDSolver && bFinalize == false)
			{
				double& CurLambda = Lambdas[i];
				const float C = -(Thickness - DistToPlane);
				const double Alpha = Compliance / (DeltaTime * DeltaTime);
				const double DeltaLambda = -(C + Alpha * CurLambda) / (CurVerletBone.InvMass + Alpha);
				CurLambda += DeltaLambda;

				CurVerletBone.Location += (PlaneNormal * DeltaLambda);
			}
			else
			{
				CurVerletBone.Location += (PlaneNormal * (Thickness - DistToPlane));
			}
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_World
///=========================================================================================================================================
FLKAnimVerletConstraint_World::FLKAnimVerletConstraint_World(const UWorld* InWorld, UPrimitiveComponent* InSelfComponent, const FName& InCollisionProfileName,
															 float InThickness, TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones)
	: WorldPtr(InWorld)
	, SelfComponentPtr(InSelfComponent)
	, WorldCollisionProfileName(InCollisionProfileName)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
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
		const FVector PrevWorldLoc = ComponentTransform.TransformPosition(CurVerletBone.PrevLocation);
		const FVector CurWorldLoc = ComponentTransform.TransformPosition(CurVerletBone.Location);

		FHitResult HitResult;
		const bool bHit = World->SweepSingleByProfile(OUT HitResult, PrevWorldLoc, CurWorldLoc, FQuat::Identity, WorldCollisionProfileName, FCollisionShape::MakeSphere(Thickness), CollisionQueryParams);
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