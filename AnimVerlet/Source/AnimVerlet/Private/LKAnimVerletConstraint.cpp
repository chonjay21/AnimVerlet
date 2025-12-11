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
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return;

		const float C = Distance - Length;
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double Denom = (BoneA->InvMass + BoneB->InvMass + Alpha);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const double DeltaLambda = -(C + Alpha * Lambda) / Denom;
		Lambda += DeltaLambda;

		if (bStretchEachBone)
			Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

		/// Adjust distance constraint
		const FVector DiffDir = (Direction * DeltaLambda);
		if (BoneA->IsPinned() == false)
			BoneA->Location -= (DiffDir * BoneA->InvMass);
		if (BoneB->IsPinned() == false)
			BoneB->Location += (DiffDir * BoneB->InvMass);
	}
	/// PBD
	else
	{
		if (FMath::IsNearlyZero(Distance, KINDA_SMALL_NUMBER))
			return;

		/// Calculate the resting distance
		const float Diff = ((Length - Distance) / Distance) * Stiffness;
		
		if (bStretchEachBone)
			Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();
		
		/// Adjust distance constraint
		const FVector DiffDir = Direction * Diff * 0.5f;
		if (BoneA->IsPinned() == false)
			BoneA->Location -= (DiffDir * BoneA->InvMass);
		if (BoneB->IsPinned() == false)
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
FLKAnimVerletConstraint_IsometricBending::FLKAnimVerletConstraint_IsometricBending(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, 
																				   FLKAnimVerletBone* InBoneD, bool bInUseXPBDSolver, double InStiffness)
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
		Stiffness = static_cast<float>(InStiffness);

	CalculateQMatrix(Q, BoneA, BoneB, BoneC, BoneD);
	RestAngle = CalculateRestAngle(BoneA, BoneB, BoneC, BoneD);
}

/// cot(angle at P) in triangle (P, Q, R)
inline float CotangentAtVertex(const FVector& P, const FVector& Q, const FVector& R)
{
	const FVector U = Q - P;
	const FVector V = R - P;

	const float CrossLen = U.Cross(V).Length();
	if (CrossLen < UE_SMALL_NUMBER)
		return 0.0f;

	const float DotUV = U.Dot(V);
	return DotUV / CrossLen;
}

inline float TriangleArea(const FVector& P, const FVector& Q, const FVector& R)
{
	return 0.5f * (Q - P).Cross(R - P).Length();
}

void FLKAnimVerletConstraint_IsometricBending::CalculateQMatrix(float InQ[4][4], FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, FLKAnimVerletBone* InBoneD)
{
	verify(InBoneA != nullptr);
	verify(InBoneB != nullptr);
	verify(InBoneC != nullptr);
	verify(InBoneD != nullptr);

	const FVector& A = InBoneA->Location;
	const FVector& B = InBoneB->Location;
	const FVector& C = InBoneC->Location;
	const FVector& D = InBoneD->Location;

	const float A0 = TriangleArea(A, B, C);
	const float A1 = TriangleArea(D, C, B);

	const float TotalArea = A0 + A1;
	if (TotalArea < UE_SMALL_NUMBER)
	{
		for (int32 i = 0; i < 4; ++i)
			for (int32 j = 0; j < 4; ++j)
				InQ[i][j] = 0.0f;
		return;
	}

	const float CotB0 = CotangentAtVertex(B, A, C); /// angle at B in triangle ABC
	const float CotC0 = CotangentAtVertex(C, A, B); /// angle at C in triangle ABC

	const float CotB1 = CotangentAtVertex(B, D, C); /// angle at B in triangle DBC
	const float CotC1 = CotangentAtVertex(C, D, B); /// angle at C in triangle DBC

	/// K vector in vertex order [A, B, C, D]
	/// Shared-edge vertices (B,C): positive sums across both triangles
	/// Opposite vertices (A,D): negative sums per triangle
	float K[4];
	K[0] = -(CotB0 + CotC0);		/// A
	K[1] = (CotB0 + CotB1);			/// B
	K[2] = (CotC0 + CotC1);			/// C
	K[3] = -(CotB1 + CotC1);		/// D

	const float Scale = 1.0f / (2.0f * TotalArea);

	/// Q = scale * K * K^T
	for (int32 i = 0; i < 4; ++i)
	{
		for (int32 j = 0; j < 4; ++j)
		{
			InQ[i][j] = Scale * K[i] * K[j];
		}
	}
}

float FLKAnimVerletConstraint_IsometricBending::CalculateRestAngle(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, FLKAnimVerletBone* InBoneD)
{
	verify(InBoneA != nullptr);
	verify(InBoneB != nullptr);
	verify(InBoneC != nullptr);
	verify(InBoneD != nullptr);

	FLKAnimVerletBone* X[4] = { InBoneA, InBoneB, InBoneC, InBoneD };
	float ResultAngle = 0.0f;
	for (int32 i = 0; i < 4; ++i)
	{
		for (int32 j = 0; j < 4; ++j)
		{
			ResultAngle += X[i]->Location.Dot(X[j]->Location) * Q[i][j];
		}
	}
	return ResultAngle;


	/*const FVector& A = InBoneA->Location;
	const FVector& B = InBoneB->Location;
	const FVector& C = InBoneC->Location;
	const FVector& D = InBoneD->Location;

	/// Shared edge direction (B -> C)
	FVector E = C - B;
	const float ELen = E.Length();
	if (ELen < UE_SMALL_NUMBER) 
		return 0.0f;

	E /= ELen;

	/// Triangle normals (make sure they are consistent with hinge BC)
	FVector N0 = (B - A).Cross(C - A);	/// normal of ABC
	FVector N1 = (C - D).Cross(B - D);	/// normal of DCB (note order!)

	const float N0Len = N0.Length();
	const float N1Len = N1.Length();
	if (N0Len < UE_SMALL_NUMBER || N1Len < UE_SMALL_NUMBER) 
		return 0.0f;

	N0 /= N0Len;
	N1 /= N1Len;

	/// Signed dihedral angle around edge e:
	/// angle = atan2( dot(e, n0 x n1), dot(n0, n1) )
	const float SinTerm = E.Dot(N0.Cross(N1));
	const float CosTerm = N0.Dot(N1);

	return FMath::Atan2(SinTerm, CosTerm);*/
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

	FVector Grad[4];
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
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return;

		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double Denom = Sum + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const double DeltaLambda = -(C + Alpha * Lambda) / Denom;
		Lambda += DeltaLambda;

		for (int32 i = 0; i < 4; ++i)
		{
			if (Bones[i]->IsPinned() == false)
				Bones[i]->Location += Grad[i] * (DeltaLambda * Bones[i]->InvMass);
		}
	}
	/// PBD
	else
	{
		const float Denom = Sum;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const float DeltaLambda = (-C / Denom) * Stiffness;
		for (int32 i = 0; i < 4; ++i)
		{
			if (Bones[i]->IsPinned() == false)
				Bones[i]->Location += Grad[i] * (DeltaLambda * Bones[i]->InvMass);
		}
	}
	


	/*const FVector& A = BoneA->Location;
	const FVector& B = BoneB->Location;
	const FVector& C = BoneC->Location;
	const FVector& D = BoneD->Location;

	FVector E = C - B;
	float ELen = E.Length();
	if (ELen < UE_SMALL_NUMBER) 
		return;

	FVector EHat = E / ELen;
	FVector N0 = (B - A).Cross(C - A);   /// ABC
	FVector N1 = (C - D).Cross(B - D);   /// DCB (order matters)

	float N0Len = N0.Length();
	float N1Len = N1.Length();
	if (N0Len < UE_SMALL_NUMBER || N1Len < UE_SMALL_NUMBER)
		return;

	FVector N0Hat = N0 / N0Len;
	FVector N1Hat = N1 / N1Len;

	/// Current signed dihedral angle around BC
	float SinTerm = EHat.Dot(N0Hat.Cross(N1Hat));
	float CosTerm = N0Hat.Dot(N1Hat);
	float Theta = FMath::Atan2(SinTerm, CosTerm);

	/// Constraint value: C = theta - RestAngle
	float Cval = Theta - RestAngle;

	const FVector GradA = (ELen / N0Len) * N0Hat;
	const FVector GradD = (ELen / N1Len) * N1Hat;

	const float InvELen = 1.0f / ELen;

	float TB0 = (C - A).Dot(E) * (InvELen / N0Len);
	float TB1 = (C - D).Dot(E) * (InvELen / N1Len);
	const FVector GradB = -(TB0 * N0Hat + TB1 * N1Hat);

	float TC0 = (B - A).Dot(E) * (InvELen / N0Len);
	float TC1 = (B - D).Dot(E) * (InvELen / N1Len);
	const FVector GradC = -(TC0 * N0Hat + TC1 * N1Hat);

	const float InvMass[4] = { BoneA->InvMass, BoneB->InvMass, BoneC->InvMass, BoneD->InvMass };
	const FVector Grad[4] = { GradA, GradB, GradC, GradD };

	/// Sum w_i * |grad_i|^2
	float Sum = 0.0f;
	for (int i = 0; i < 4; ++i)
		Sum += InvMass[i] * Grad[i].Dot(Grad[i]);

	/// XPBD
	if (bUseXPBDSolver)
	{
		if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
			return;

		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double Denom = Sum + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const double DLambda = -(Cval + Alpha * Lambda) / Denom;
		Lambda += DLambda;

		if (BoneA->IsPinned() == false)
			BoneA->Location += GradA * (DLambda * BoneA->InvMass);
		if (BoneB->IsPinned() == false)
			BoneB->Location += GradB * (DLambda * BoneB->InvMass);
		if (BoneC->IsPinned() == false)
			BoneC->Location += GradC * (DLambda * BoneC->InvMass);
		if (BoneD->IsPinned() == false)
			BoneD->Location += GradD * (DLambda * BoneD->InvMass);
	}
	/// PBD
	else
	{
		const float Denom = Sum;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const float DLambda = (-Cval / Denom) * Stiffness;
		if (BoneA->IsPinned() == false)
			BoneA->Location += GradA * (DLambda * BoneA->InvMass);
		if (BoneB->IsPinned() == false)
			BoneB->Location += GradB * (DLambda * BoneB->InvMass);
		if (BoneC->IsPinned() == false)
			BoneC->Location += GradC * (DLambda * BoneC->InvMass);
		if (BoneD->IsPinned() == false)
			BoneD->Location += GradD * (DLambda * BoneD->InvMass);
	}*/
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
/// FLKAnimVerletConstraint_FlatBending
///=========================================================================================================================================
FLKAnimVerletConstraint_FlatBending::FLKAnimVerletConstraint_FlatBending(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, 
																		 FLKAnimVerletBone* InBoneD, bool bInUseXPBDSolver, double InStiffness, float InFlatAlpha)
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

	FlatAlpha = InFlatAlpha;
	TargetAngle = ComputeDihedralAngle_BC(InBoneA->Location, InBoneB->Location, InBoneC->Location, InBoneD->Location);
	///TargetAngle = 0.0f;
}

float FLKAnimVerletConstraint_FlatBending::ComputeDihedralAngle_BC(const FVector& A, const FVector& B, const FVector& C, const FVector& D)
{
	const FVector E = C - B;
	const float ELen = E.Size();
	if (ELen < KINDA_SMALL_NUMBER) 
		return 0.0f;

	const FVector EN = E / ELen;

	const FVector N0 = (C - B).Cross(A - B).GetSafeNormal();
	const FVector N1 = (B - C).Cross(D - C).GetSafeNormal();

	const float CosT = FMath::Clamp(N0.Dot(N1), -1.0f, 1.0f);
	float Theta = FMath::Acos(CosT);

	const float S = N0.Cross(N1).Dot(EN);
	if (S < 0.0f) 
		Theta = -Theta;

	return Theta;
}

void FLKAnimVerletConstraint_FlatBending::ComputeBendingGradients(OUT FVector& GradientsA, OUT FVector& GradientsB, OUT FVector& GradientsC, OUT FVector& GradientsD,
																  const FVector& A, const FVector& B, const FVector& C, const FVector& D)
{
	const FVector E = C - B;
	const float ELen = E.Size();
	if (ELen < KINDA_SMALL_NUMBER)
	{
		GradientsA = FVector::ZeroVector;
		GradientsB = FVector::ZeroVector;
		GradientsC = FVector::ZeroVector;
		GradientsD = FVector::ZeroVector;
		return;
	}

	const FVector N0 = (C - B).Cross(A - B);
	const FVector N1 = (B - C).Cross(D - C);

	const float N0Len2 = N0.SizeSquared();
	const float N1Len2 = N1.SizeSquared();
	if (N0Len2 < KINDA_SMALL_NUMBER || N1Len2 < KINDA_SMALL_NUMBER)
	{
		GradientsA = FVector::ZeroVector;
		GradientsB = FVector::ZeroVector;
		GradientsC = FVector::ZeroVector;
		GradientsD = FVector::ZeroVector;
		return;
	}

	const FVector QA = (ELen / N0Len2) * N0;
	const FVector QD = (ELen / N1Len2) * N1;

	const float InvELen2 = 1.0f / (ELen * ELen);

	const float WB0 = (C - A).Dot(E) * InvELen2;
	const float WC0 = (A - B).Dot(E) * InvELen2;

	const float WB1 = (C - D).Dot(E) * InvELen2;
	const float WC1 = (D - B).Dot(E) * InvELen2;

	GradientsA = QA;
	GradientsD = QD;
	GradientsB = -WB0 * QA - WB1 * QD;
	GradientsC = -WC0 * QA - WC1 * QD;
}

void FLKAnimVerletConstraint_FlatBending::Update(float DeltaTime, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);
	verify(BoneD != nullptr);

	/// Dynamic Rest -> InitRestAngle
	const float K = FMath::Max(0.0f, FlatAlpha);   /// Rest speed per sec
	const float DT = FMath::Max(DeltaTime, 0.0f);

	const float Decay = FMath::Exp(-K * DT);
	RestAngle = TargetAngle + (RestAngle - TargetAngle) * Decay;

	const FVector& A = BoneA->Location;
	const FVector& B = BoneB->Location;
	const FVector& C = BoneC->Location;
	const FVector& D = BoneD->Location;
	const float Theta = ComputeDihedralAngle_BC(A, B, C, D);

	/// Constraint value: C = theta - RestAngle
	const float Cval = Theta - RestAngle;

	FVector GradientsA = FVector::ZeroVector;
	FVector GradientsB = FVector::ZeroVector;
	FVector GradientsC = FVector::ZeroVector;
	FVector GradientsD = FVector::ZeroVector;
	ComputeBendingGradients(OUT GradientsA, OUT GradientsB, OUT GradientsC, OUT GradientsD, A, B, C, D);

	/// XPBD
	if (bUseXPBDSolver)
	{
		if (FMath::IsNearlyZero(DT, KINDA_SMALL_NUMBER))
			return;

		const double Alpha = Compliance / (DT * DT);
		const double Denom = BoneA->InvMass * GradientsA.SizeSquared() + BoneB->InvMass * GradientsB.SizeSquared() + BoneC->InvMass * GradientsC.SizeSquared() + BoneD->InvMass * GradientsD.SizeSquared() + Alpha;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const double DeltaLambda = -(Cval + Alpha * Lambda) / Denom;
		Lambda += DeltaLambda;

		if (BoneA->IsPinned() == false) 
			BoneA->Location += BoneA->InvMass * DeltaLambda * GradientsA;
		if (BoneB->IsPinned() == false)
			BoneB->Location += BoneB->InvMass * DeltaLambda * GradientsB;
		if (BoneC->IsPinned() == false)
			BoneC->Location += BoneC->InvMass * DeltaLambda * GradientsC;
		if (BoneD->IsPinned() == false)
			BoneD->Location += BoneD->InvMass * DeltaLambda * GradientsD;
	}
	/// PBD
	else
	{
		const float Denom = BoneA->InvMass * GradientsA.SizeSquared() + BoneB->InvMass * GradientsB.SizeSquared() + BoneC->InvMass * GradientsC.SizeSquared() + BoneD->InvMass * GradientsD.SizeSquared();
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const float DeltaLambda = (-Cval / Denom) * Stiffness;

		if (BoneA->IsPinned() == false)
			BoneA->Location += BoneA->InvMass * DeltaLambda * GradientsA;
		if (BoneB->IsPinned() == false)
			BoneB->Location += BoneB->InvMass * DeltaLambda * GradientsB;
		if (BoneC->IsPinned() == false)
			BoneC->Location += BoneC->InvMass * DeltaLambda * GradientsC;
		if (BoneD->IsPinned() == false)
			BoneD->Location += BoneD->InvMass * DeltaLambda * GradientsD;
	}
}

void FLKAnimVerletConstraint_FlatBending::PostUpdate(float DeltaTime)
{
	Lambda = 0.0f;
}

void FLKAnimVerletConstraint_FlatBending::ResetSimulation()
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
		if (BoneB->IsPinned() == false)
			BoneB->Location += StraightenedDir * StraightenDist;

		float NewBToALength = 0.0f;
		FVector NewBToADir = FVector::ZeroVector;
		(BoneA->Location - BoneB->Location).ToDirectionAndLength(OUT NewBToADir, OUT NewBToALength);

		float NewBToCLength = 0.0f;
		FVector NewBToCDir = FVector::ZeroVector;
		(BoneC->Location - BoneB->Location).ToDirectionAndLength(OUT NewBToCDir, OUT NewBToCLength);

		if (BoneA->IsPinned() == false)
			BoneA->Location = BoneB->Location * NewBToADir * BToALength;
		if (BoneC->IsPinned() == false)
			BoneC->Location = BoneB->Location * NewBToCDir * BToCLength;*/

		const FVector StraightenedDirC = FMath::Lerp(BToCDir, AToBDir, StraightenStrength * DeltaTime);
		if (BoneC->IsPinned() == false)
			BoneC->Location = BoneB->Location + StraightenedDirC * BToCLength;

		float NewCToBLength = 0.0f;
		FVector NewCToBDir = FVector::ZeroVector;
		(BoneB->Location - BoneC->Location).ToDirectionAndLength(OUT NewCToBDir, OUT NewCToBLength);

		const FVector AStraightenedDir = FMath::Lerp(BToADir, NewCToBDir, StraightenStrength * DeltaTime);
		if (BoneA->IsPinned() == false)
			BoneA->Location = BoneB->Location + AStraightenedDir * BToALength;
	}
	else
	{
		if (BoneC->IsPinned() == false)
		{
			const FVector StraightenedDir = FMath::Lerp(BToCDir, AToBDir, StraightenStrength * DeltaTime);
			BoneC->Location = BoneB->Location + StraightenedDir * BToCLength;
		}
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
			if (BoneB->IsPinned() == false)
				BoneB->Location = Center + Direction * LengthWithMargin * 0.5f;
			if (BoneA->IsPinned() == false)
				BoneA->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			if (BoneB->IsPinned() == false)
				BoneB->Location = BoneA->Location + Direction * LengthWithMargin;
		}
	}
	else if (Distance < Length - LengthMargin)
	{
		const float LengthWithMargin = Length - LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneA->Location + Direction * Distance * 0.5f;
			if (BoneB->IsPinned() == false)
				BoneB->Location = Center + Direction * LengthWithMargin * 0.5f;
			if (BoneA->IsPinned() == false)
				BoneA->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			if (BoneB->IsPinned() == false)
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
			if (BoneA->IsPinned() == false)
				BoneA->Location = Center + Direction * LengthWithMargin * 0.5f;
			if (BoneB->IsPinned() == false)
				BoneB->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			if (BoneA->IsPinned() == false)
				BoneA->Location = BoneB->Location + Direction * LengthWithMargin;
		}
	}
	else if (Distance < Length - LengthMargin)
	{
		const float LengthWithMargin = Length - LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneB->Location + Direction * Distance * 0.5f;
			if (BoneA->IsPinned() == false)
				BoneA->Location = Center + Direction * LengthWithMargin * 0.5f;
			if (BoneB->IsPinned() == false)
				BoneB->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			if (BoneA->IsPinned() == false)
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
			if (FMath::IsNearlyZero(DeltaTime, KINDA_SMALL_NUMBER))
				return;

			const float C = -AngleDiff;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (BoneA->InvMass + BoneB->InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return;

			const double DeltaLambda = -(C + Alpha * Lambda) / Denom;
			Lambda += DeltaLambda;

			if (BoneB->IsPinned() == false)
			{
				const FVector ConstraintDir = BoneAToBoneB.RotateAngleAxis(-DeltaLambda, RotationAxis);
				BoneB->Location = BoneA->Location + (ConstraintDir * BoneAToBoneBSize);
			}
		}
		else
		{
			if (BoneB->IsPinned() == false)
			{
				const FVector ConstraintDir = BoneAToBoneB.RotateAngleAxis(-AngleDiff, RotationAxis);
				BoneB->Location = BoneA->Location + (ConstraintDir * BoneAToBoneBSize);
			}
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
		CheckSphereCapsule(DeltaTime, bFinalize);
	else
		CheckSphereSphere(DeltaTime, bFinalize);
}

bool FLKAnimVerletConstraint_Sphere::CheckSphereSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, int32 LambdaIndex)
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

bool FLKAnimVerletConstraint_Sphere::CheckSphereCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, int32 LambdaIndex)
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

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * DeltaLambda * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * DeltaLambda * ChildMoveAlpha);
		}
		else
		{
			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (SphereToBoneDir * PenetrationDepth * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (SphereToBoneDir * PenetrationDepth * ChildMoveAlpha);
		}
		return true;
	}
	return false;
}

void FLKAnimVerletConstraint_Sphere::CheckSphereCapsule(float DeltaTime, bool bFinalize)
{
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
			CheckSphereSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		CheckSphereCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bFinalize, i);
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
		CheckCapsuleCapsule(DeltaTime, bFinalize);
	else
		CheckCapsuleSphere(DeltaTime, bFinalize);
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

bool FLKAnimVerletConstraint_Capsule::CheckCapsuleSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
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

bool FLKAnimVerletConstraint_Capsule::CheckCapsuleCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex)
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

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * DeltaLambda * ChildMoveAlpha);
		}
		else
		{
			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CapsuleToBoneDir * PenetrationDepth * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CapsuleToBoneDir * PenetrationDepth * ChildMoveAlpha);
		}
		return true;
	}
	return false;
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

		verify(CurPair.BoneB.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
		if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
		{
			CheckCapsuleSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, CapsuleStart, CapsuleEnd, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];
		
		CheckCapsuleCapsule(CurVerletBone, ParentVerletBone, DeltaTime, bFinalize, CapsuleStart, CapsuleEnd, i);
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
		CheckBoxCapsule(DeltaTime, bFinalize);
	else
		CheckBoxSphere(DeltaTime, bFinalize);
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

bool FLKAnimVerletConstraint_Box::CheckBoxSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const FVector& SphereLocation, const FQuat& InvRotation, int32 LambdaIndex)
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

inline FVector ClampPointToAABB(const FVector& P, const FVector& E)
{
	return FVector(FMath::Clamp(P.X, -E.X, E.X), FMath::Clamp(P.Y, -E.Y, E.Y), FMath::Clamp(P.Z, -E.Z, E.Z));
}

inline bool Slab(IN OUT float& TMin, IN OUT float& TMax, float A, float D, float E)
{
	if (FMath::Abs(D) < KINDA_SMALL_NUMBER)
	{
		return (A >= -E && A <= E);
	}

	const float OOD = 1.0f / D;
	float T1 = (-E - A) * OOD;
	float T2 = (E - A) * OOD;
	if (T1 > T2)
		Swap(T1, T2);

	TMin = FMath::Max(TMin, T1);
	TMax = FMath::Min(TMax, T2);
	return (TMin <= TMax);
};

inline bool SegmentIntersectsAABB(OUT float& OutTMin, OUT float& OutTMax, const FVector& A, const FVector& B, const FVector& E)
{
	const FVector D = B - A;

	float TMin = 0.0f;
	float TMax = 1.0f;
	if (Slab(IN OUT TMin, IN OUT TMax, A.X, D.X, E.X) == false)
		return false;
	if (Slab(IN OUT TMin, IN OUT TMax, A.Y, D.Y, E.Y) == false)
		return false;
	if (Slab(IN OUT TMin, IN OUT TMax, A.Z, D.Z, E.Z) == false)
		return false;

	OutTMin = TMin;
	OutTMax = TMax;
	return true;
}

inline void ClosestPtSegmentAABB(OUT float& OutSegT, OUT FVector& OutSegPt, OUT FVector& OutBoxPt, const FVector& A, const FVector& B, const FVector& E)
{
	const FVector D = B - A;
	const float DD = D.Dot(D);

	float TMin = 0.0f;
	float TMax = 0.0f;
	if (SegmentIntersectsAABB(OUT TMin, OUT TMax, A, B, E))
	{
		OutSegT = FMath::Clamp((TMin + TMax) * 0.5f, 0.0f, 1.0f);
		OutSegPt = A + D * OutSegT;
		OutBoxPt = OutSegPt;
		return;
	}

	FVector BoxPt = ClampPointToAABB(A, E);
	float T = 0.0f;
	if (DD > KINDA_SMALL_NUMBER)
	{
		T = (BoxPt - A).Dot(D) / DD;
		T = FMath::Clamp(T, 0.0f, 1.0f);
	}

	for (int32 i = 0; i < 3; ++i)
	{
		const FVector SegPt = A + D * T;
		BoxPt = ClampPointToAABB(SegPt, E);

		if (DD > KINDA_SMALL_NUMBER)
		{
			T = (BoxPt - A).Dot(D) / DD;
			T = FMath::Clamp(T, 0.0f, 1.0f);
		}
		else
		{
			T = 0.0f;
		}
	}

	OutSegT = T;
	OutSegPt = A + D * T;
	OutBoxPt = ClampPointToAABB(OutSegPt, E);
}

bool FLKAnimVerletConstraint_Box::CheckBoxCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
{
	const FVector ParentBoneLocationInBoxLocal = InvRotation.RotateVector(ParentVerletBone.Location - Location);
	const FVector CurBoneLocationInBoxLocal = InvRotation.RotateVector(CurVerletBone.Location - Location);

	float SegT = 0.0f;
	FVector SegPtL = FVector::ZeroVector;
	FVector	BoxPtL = FVector::ZeroVector;
	ClosestPtSegmentAABB(OUT SegT, OUT SegPtL, OUT BoxPtL, ParentBoneLocationInBoxLocal, CurBoneLocationInBoxLocal, HalfExtents);

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

		double& CurLambda = Lambdas[LambdaIndex];
		const float C = -PenetrationDepth;
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double Denom = (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return false;

		const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
		CurLambda += DeltaLambda;

		float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
		ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

		if (ParentVerletBone.IsPinned())
			ChildMoveAlpha = 1.0f;
		if (CurVerletBone.IsPinned())
			ChildMoveAlpha = 0.0f;

		if (ParentVerletBone.IsPinned() == false)
			ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * (1.0f - ChildMoveAlpha));
		if (CurVerletBone.IsPinned() == false)
			CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * ChildMoveAlpha);
	}
	else
	{
		float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
		ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

		if (ParentVerletBone.IsPinned())
			ChildMoveAlpha = 1.0f;
		if (CurVerletBone.IsPinned())
			ChildMoveAlpha = 0.0f;

		if (ParentVerletBone.IsPinned() == false)
			ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * PenetrationDepth * (1.0f - ChildMoveAlpha));
		if (CurVerletBone.IsPinned() == false)
			CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * PenetrationDepth * ChildMoveAlpha);
	}
	return true;
}

bool FLKAnimVerletConstraint_Box::CheckBoxBox(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex)
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

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * DeltaLambda * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * DeltaLambda * ChildMoveAlpha);
		}
		else
		{
			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (CollisionNormal * PenetrationDepth * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (CollisionNormal * PenetrationDepth * ChildMoveAlpha);
		}
	}
	return true;
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

		verify(CurPair.BoneB.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
		if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
		{
			CheckBoxSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, CurVerletBone.Location, InvRotation, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		///CheckBoxCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bFinalize, InvRotation, i);

		/// OBB - OBB version
		/// Consider BoneLine to OBB
		CheckBoxBox(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bFinalize, InvRotation, i);
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
		CheckPlaneCapsule(DeltaTime, bFinalize);
	else
		CheckPlaneSphere(DeltaTime, bFinalize);
}

bool FLKAnimVerletConstraint_Plane::CheckPlaneSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex)
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

bool FLKAnimVerletConstraint_Plane::CheckPlaneCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex)
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

			double& CurLambda = Lambdas[LambdaIndex];
			const float C = -PenetrationDepth;
			const double Alpha = Compliance / (DeltaTime * DeltaTime);
			const double Denom = (CurVerletBone.InvMass + ParentVerletBone.InvMass + Alpha);
			if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
				return false;

			const double DeltaLambda = -(C + Alpha * CurLambda) / Denom;
			CurLambda += DeltaLambda;

			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (PlaneNormal * DeltaLambda * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (PlaneNormal * DeltaLambda * ChildMoveAlpha);
		}
		else
		{
			float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (ClosestOnBone - ParentVerletBone.Location).Size() / DistFromParent;
			ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

			if (ParentVerletBone.IsPinned())
				ChildMoveAlpha = 1.0f;
			if (CurVerletBone.IsPinned())
				ChildMoveAlpha = 0.0f;

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (PlaneNormal * PenetrationDepth * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (PlaneNormal * PenetrationDepth * ChildMoveAlpha);
		}
		return true;
	}
	return false;
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

		verify(CurPair.BoneB.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
		if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
		{
			CheckPlaneSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, bFinitePlane, InvRotation, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		CheckPlaneCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bFinalize, bFinitePlane, InvRotation, i);
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

bool FLKAnimVerletConstraint_World::CheckWorldCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, 
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
		float ChildMoveAlpha = FMath::IsNearlyZero(DistFromParent, KINDA_SMALL_NUMBER) ? 0.0f : (HitResult.Location - ParentVerletBone.Location).Size() / DistFromParent;
		ChildMoveAlpha = FMath::Clamp(ChildMoveAlpha, 0.0f, 1.0f);
		ChildMoveAlpha = ChildMoveAlpha * (2.0f - ChildMoveAlpha);	///EaseOutQuad for better movement(heuristic center of mass)

		if (ParentVerletBone.IsPinned() || CurVerletBone.IsPinned())
			ChildMoveAlpha = 1.0f;

		if (HitResult.bStartPenetrating && HitResult.PenetrationDepth > 0.0f)
		{
			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (HitResult.Normal * HitResult.PenetrationDepth * (1.0f - ChildMoveAlpha));
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (HitResult.Normal * HitResult.PenetrationDepth * ChildMoveAlpha);
		}
		else
		{
			const FVector ResolvedLocationInWorld = HitResult.Location;
			const FVector NewLocation = ComponentTransform.InverseTransformPosition(ResolvedLocationInWorld);

			if (ParentVerletBone.IsPinned() == false)
				ParentVerletBone.Location = ParentVerletBone.Location + (NewLocation - VerletBoneCenter) * (1.0f - ChildMoveAlpha);
			if (CurVerletBone.IsPinned() == false)
				CurVerletBone.Location = CurVerletBone.Location + (NewLocation - VerletBoneCenter) * ChildMoveAlpha;
		}
		return true;
	}
	return false;
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

		verify(CurPair.BoneB.IsValidBoneIndicator());
		verify(Bones->IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
		FLKAnimVerletBone& CurVerletBone = (*Bones)[CurPair.BoneB.AnimVerletBoneIndex];
		if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
		{
			CheckWorldSphere(IN OUT CurVerletBone, DeltaTime, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
			continue;
		}

		verify(Bones->IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
		FLKAnimVerletBone& ParentVerletBone = (*Bones)[CurPair.BoneA.AnimVerletBoneIndex];

		CheckWorldCapsule(IN OUT CurVerletBone, IN OUT ParentVerletBone, DeltaTime, bFinalize, World, CollisionQueryParams, ComponentTransform, i);
	}
}
///=========================================================================================================================================