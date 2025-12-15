#include "LKAnimVerletConstraint.h"

#include "LKAnimVerletBone.h"
#include "LKAnimVerletConstraintUtil.h"


///=========================================================================================================================================
/// FLKAnimVerletConstraint_Pin
///=========================================================================================================================================
void FLKAnimVerletConstraint_Pin::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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

void FLKAnimVerletConstraint_Distance::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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

	const float A0 = LKAnimVerletUtil::TriangleArea(A, B, C);
	const float A1 = LKAnimVerletUtil::TriangleArea(D, C, B);

	const float TotalArea = A0 + A1;
	if (TotalArea < UE_SMALL_NUMBER)
	{
		for (int32 i = 0; i < 4; ++i)
			for (int32 j = 0; j < 4; ++j)
				InQ[i][j] = 0.0f;
		return;
	}

	const float CotB0 = LKAnimVerletUtil::CotangentAtVertex(B, A, C); /// angle at B in triangle ABC
	const float CotC0 = LKAnimVerletUtil::CotangentAtVertex(C, A, B); /// angle at C in triangle ABC

	const float CotB1 = LKAnimVerletUtil::CotangentAtVertex(B, D, C); /// angle at B in triangle DBC
	const float CotC1 = LKAnimVerletUtil::CotangentAtVertex(C, D, B); /// angle at C in triangle DBC

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

void FLKAnimVerletConstraint_IsometricBending::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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
/// FLKAnimVerletConstraint_Bending_1D
///=========================================================================================================================================
FLKAnimVerletConstraint_Bending_1D::FLKAnimVerletConstraint_Bending_1D(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC, bool bInUseXPBDSolver, double InStiffness)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, BoneC(InBoneC)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);

	bUseXPBDSolver = bInUseXPBDSolver;
	if (bUseXPBDSolver)
		Compliance = InStiffness;
	else
		Stiffness = static_cast<float>(InStiffness);

	RestAngle = CalculateRestAngle(BoneA, BoneB, BoneC);
}

float FLKAnimVerletConstraint_Bending_1D::CalculateRestAngle(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, FLKAnimVerletBone* InBoneC)
{
	verify(InBoneA != nullptr);
	verify(InBoneB != nullptr);
	verify(InBoneC != nullptr);

	FVector E1 = InBoneA->Location - InBoneB->Location;
	FVector E2 = InBoneC->Location - InBoneB->Location;

	const float Len1 = E1.Size();
	const float Len2 = E2.Size();
	if (Len1 < KINDA_SMALL_NUMBER || Len2 < KINDA_SMALL_NUMBER)
		return 0.0f;

	E1 /= Len1;
	E2 /= Len2;

	float CosTheta = E1.Dot(E2);
	CosTheta = FMath::Clamp(CosTheta, -1.0f, 1.0f);
	return CosTheta;
}

void FLKAnimVerletConstraint_Bending_1D::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(BoneC != nullptr);

	/// Edges
	const FVector E1 = BoneA->Location - BoneB->Location;
	const FVector E2 = BoneC->Location - BoneB->Location;

	const float Len1 = E1.Size();
	const float Len2 = E2.Size();

	const float Eps = 1e-6f;
	if (Len1 < KINDA_SMALL_NUMBER || Len2 < KINDA_SMALL_NUMBER)
		return;

	const FVector N1 = E1 / Len1;
	const FVector N2 = E2 / Len2;

	float CosTheta = N1.Dot(N2);
	CosTheta = FMath::Clamp(CosTheta, -1.0f, 1.0f);

	/// Constraint: C(x) = cos(theta) - cos(restAngle) = 0
	const float C = CosTheta - RestAngle;

	/// Gradient of C wrt positions
	/// dC/dA = (n2 - cosTheta * n1) / |e1|
	const FVector GradientsA = (N2 - CosTheta * N1) / Len1;
	/// dC/dC = (n1 - cosTheta * n2) / |e2|
	const FVector GradientsC = (N1 - CosTheta * N2) / Len2;
	/// dC/dB = -dC/dA - dC/dC
	const FVector GradientsB = -GradientsA - GradientsC;

	const float Sum = BoneA->InvMass * GradientsA.SizeSquared() + BoneB->InvMass * GradientsB.SizeSquared() + BoneC->InvMass * GradientsC.SizeSquared();

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

		if (BoneA->IsPinned() == false)
		{
			BoneA->Location += (DeltaLambda * BoneA->InvMass) * GradientsA;
		}
		if (BoneB->IsPinned() == false)
		{
			BoneB->Location += (DeltaLambda * BoneB->InvMass) * GradientsB;
		}
		if (BoneC->IsPinned() == false)
		{
			BoneC->Location += (DeltaLambda * BoneC->InvMass) * GradientsC;
		}
	}
	/// PBD
	else
	{
		const float Denom = Sum;
		if (FMath::IsNearlyZero(Denom, KINDA_SMALL_NUMBER))
			return;

		const float DeltaLambda = (-C / Denom) * Stiffness;
		if (BoneA->IsPinned() == false)
		{
			BoneA->Location += (DeltaLambda * BoneA->InvMass) * GradientsA;
		}
		if (BoneB->IsPinned() == false)
		{
			BoneB->Location += (DeltaLambda * BoneB->InvMass) * GradientsB;
		}
		if (BoneC->IsPinned() == false)
		{
			BoneC->Location += (DeltaLambda * BoneC->InvMass) * GradientsC;
		}
	}
}

void FLKAnimVerletConstraint_Bending_1D::PostUpdate(float DeltaTime)
{
	Lambda = 0.0f;
}

void FLKAnimVerletConstraint_Bending_1D::ResetSimulation()
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

void FLKAnimVerletConstraint_FlatBending::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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

void FLKAnimVerletConstraint_Straighten::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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

void FLKAnimVerletConstraint_FixedDistance::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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

void FLKAnimVerletConstraint_FixedDistance::BackwardUpdate(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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

void FLKAnimVerletConstraint_BallSocket::Update(float DeltaTime, bool bInitialUpdate, bool bFinalize)
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