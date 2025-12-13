#pragma once
#include <CoreMinimal.h>

namespace LKAnimVerletUtil
{
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

	/// ClosestPoint on Triangle(A,B,C) from P + "weights for Q"
	inline FVector ClosestPointOnTriangleWeights(OUT float& OutWA, OUT float& OutWB, OUT float& OutWC, const FVector& P, 
												 const FVector& A, const FVector& B, const FVector& C)
	{
		const FVector AB = B - A;
		const FVector AC = C - A;
		const FVector AP = P - A;

		const float D1 = AB.Dot(AP);
		const float D2 = AC.Dot(AP);
		if (D1 <= 0.0f && D2 <= 0.0f)
		{
			OutWA = 1.0f;
			OutWB = 0.0f;
			OutWC = 0.0f;
			return A;
		}

		const FVector BP = P - B;
		const float D3 = AB.Dot(BP);
		const float D4 = AC.Dot(BP);
		if (D3 >= 0.0f && D4 <= D3)
		{
			OutWA = 0.0f;
			OutWB = 1.0f;
			OutWC = 0.0f;
			return B;
		}

		const float VC = D1 * D4 - D3 * D2;
		if (VC <= 0.0f && D1 >= 0.0f && D3 <= 0.0f)
		{
			const float V = D1 / (D1 - D3); /// on AB
			OutWA = 1.0f - V;
			OutWB = V;
			OutWC = 0.0f;
			return A + V * AB;
		}

		const FVector CP = P - C;
		const float D5 = AB.Dot(CP);
		const float D6 = AC.Dot(CP);
		if (D6 >= 0.0f && D5 <= D6)
		{
			OutWA = 0.0f;
			OutWB = 0.0f;
			OutWC = 1.0f;
			return C;
		}

		const float VB = D5 * D2 - D1 * D6;
		if (VB <= 0.0f && D2 >= 0.0f && D6 <= 0.0f)
		{
			const float W = D2 / (D2 - D6); /// on AC
			OutWA = 1.0f - W;
			OutWB = 0.0f;
			OutWC = W;
			return A + W * AC;
		}

		const float VA = D3 * D6 - D5 * D4;
		if (VA <= 0.0f && (D4 - D3) >= 0.0f && (D5 - D6) >= 0.0f)
		{
			const FVector BC = C - B;
			const float W = (D4 - D3) / ((D4 - D3) + (D5 - D6)); /// on BC
			OutWA = 0.0f;
			OutWB = 1.0f - W;
			OutWC = W;
			return B + W * BC;
		}

		/// Inside face region
		const float Denom = 1.0f / (VA + VB + VC);
		const float V = VB * Denom;
		const float W = VC * Denom;
		const float U = 1.0f - V - W;
		OutWA = U;
		OutWB = V;
		OutWC = W;
		return U * A + V * B + W * C;
	}

	inline void ClosestPointsSegmentSegment(OUT float& OutS, OUT float& OutT, OUT FVector& OutCP, OUT FVector& OutCQ,
											const FVector& P0, const FVector& P1, const FVector& Q0, const FVector& Q1)
	{
		OutS = 0.0f;
		OutT = 0.0f;

		const FVector D1 = P1 - P0; /// direction of segment P
		const FVector D2 = Q1 - Q0; /// direction of segment Q
		const FVector R = P0 - Q0;
		const float A = D1.Dot(D1);
		const float E = D2.Dot(D2);
		const float F = D2.Dot(R);

		/// Both segments degenerate
		if (A <= KINDA_SMALL_NUMBER && E <= KINDA_SMALL_NUMBER)
		{
			OutCP = P0;
			OutCQ = Q0;
			return;
		}

		/// P degenerate
		if (A <= KINDA_SMALL_NUMBER)
		{
			OutS = 0.0f;
			OutT = FMath::Clamp((F / E), 0.0f, 1.0f);
			OutCP = P0;
			OutCQ = Q0 + OutT * D2;
			return;
		}

		const float C = D1.Dot(R);

		/// Q degenerate
		if (E <= KINDA_SMALL_NUMBER)
		{
			OutT = 0.0f;
			OutS = FMath::Clamp((-C / A), 0.0f, 1.0f);
			OutCP = P0 + OutS * D1;
			OutCQ = Q0;
			return;
		}

		const float B = D1.Dot(D2);
		const float Denom = A * E - B * B;

		/// If not parallel, compute closest point on infinite lines then clamp
		if (Denom != 0.0f)
		{
			OutS = FMath::Clamp(((B * F - C * E) / Denom), 0.0f, 1.0f);
		}
		else
		{
			OutS = 0.0f;
		}
		OutT = (B * OutS + F) / E;

		/// Clamp t, recompute s if needed
		if (OutT < 0.0f)
		{
			OutT = 0.0f;
			OutS = FMath::Clamp((-C / A), 0.0f, 1.0f);
		}
		else if (OutT > 1.0f)
		{
			OutT = 1.0f;
			OutS = FMath::Clamp(((B - C) / A), 0.0f, 1.0f);
		}

		OutCP = P0 + OutS * D1;
		OutCQ = Q0 + OutT * D2;
	}

	/// Barycentric weights for point on triangle plane (not clamped) - Assumes triangle not degenerate.
	inline void BarycentricOnTriangle(OUT float& OutWA, OUT float& OutWB, OUT float& OutWC, const FVector& P,
									  const FVector& A, const FVector& B, const FVector& C)
	{
		const FVector V0 = B - A;
		const FVector V1 = C - A;
		const FVector V2 = P - A;

		const float D00 = V0.Dot(V0);
		const float D01 = V0.Dot(V1);
		const float D11 = V1.Dot(V1);
		const float D20 = V2.Dot(V0);
		const float D21 = V2.Dot(V1);

		const float Denom = D00 * D11 - D01 * D01;
		if (FMath::Abs(Denom) <= KINDA_SMALL_NUMBER)
		{
			/// Degenerate fallback: put all weight on A
			OutWA = 1.0f;
			OutWB = 0.0f;
			OutWC = 0.0f;
			return;
		}

		const float InvDenom = 1.0f / Denom;
		const float V = (D11 * D20 - D01 * D21) * InvDenom;		/// weight for B
		const float W = (D00 * D21 - D01 * D20) * InvDenom;		/// weight for C
		const float U = 1.0f - V - W;							/// weight for A

		OutWA = U;
		OutWB = V;
		OutWC = W;
	}

	inline bool IsPointInTriangleBarycentric(float WA, float WB, float WC, float Eps = KINDA_SMALL_NUMBER)
	{
		return (WA >= -Eps && WB >= -Eps && WC >= -Eps);
	}

	/// Capsule axis segment vs triangle: find minimal distance pair and weights on triangle
	inline void ClosestPointsCapsuleSegTriangle(OUT FVector& OutPc, OUT FVector& OutQt,
												OUT float& OutWA, OUT float& OutWB, OUT float& OutWC,
												OUT float& OutDistSQ,
												const FVector& Seg0, const FVector& Seg1,
												const FVector& A, const FVector& B, const FVector& C)
	{
		OutDistSQ = TNumericLimits<float>::Max();
		OutWA = 1.0f;
		OutWB = 0.0f;
		OutWC = 0.0f;
		OutPc = Seg0;
		OutQt = A;

		/// Candidate 1: segment vs triangle face interior
		{
			const FVector AB = B - A;
			const FVector AC = C - A;
			const FVector N = AB.Cross(AC);
			const float NSq = N.SizeSquared();

			if (NSq > KINDA_SMALL_NUMBER)
			{
				const FVector UnitNormal = N / FMath::Sqrt(NSq);
				const FVector D = Seg1 - Seg0;

				const float DN = D.Dot(UnitNormal);
				float T = 0.0f;

				if (FMath::Abs(DN) > KINDA_SMALL_NUMBER)
				{
					/// minimize signed distance to plane: (Seg0 + tD - A)¡¤n = 0
					T = -FVector::DotProduct(Seg0 - A, UnitNormal) / DN;
					T = FMath::Clamp(T, 0.0f, 1.0f);
				}
				else
				{
					/// segment nearly parallel: take endpoint closer to plane
					const float S0 = (Seg0 - A).Dot(UnitNormal);
					const float S1 = (Seg1 - A).Dot(UnitNormal);
					T = (FMath::Abs(S1) < FMath::Abs(S0)) ? 1.0f : 0.0f;
				}

				const FVector Pc = Seg0 + T * D;
				/// project Pc onto plane
				const float SignedDist = (Pc - A).Dot(UnitNormal);
				const FVector Qt = Pc - SignedDist * UnitNormal;

				float WA = 0.0f;
				float WB = 0.0f;
				float WC = 0.0f;
				BarycentricOnTriangle(OUT WA, OUT WB, OUT WC, Qt, A, B, C);

				if (IsPointInTriangleBarycentric(WA, WB, WC))
				{
					const float D2 = (Pc - Qt).SizeSquared();
					if (D2 < OutDistSQ)
					{
						OutDistSQ = D2;
						OutPc = Pc;
						OutQt = Qt;
						OutWA = WA;
						OutWB = WB;
						OutWC = WC;
					}
				}
			}
		}

		/// Candidate 2: segment vs each triangle edge (AB, BC, CA)
		auto ConsiderEdge = [&](const FVector& E0, const FVector& E1, float wA0, float wB0, float wC0, float wA1, float wB1, float wC1)
		{
			float s, t;
			FVector Pc, Qe;
			ClosestPointsSegmentSegment(OUT s, OUT t, OUT Pc, OUT Qe, Seg0, Seg1, E0, E1);
			const float D2 = (Pc - Qe).SizeSquared();
			if (D2 < OutDistSQ)
			{
				OutDistSQ = D2;
				OutPc = Pc;
				OutQt = Qe;

				/// Edge point weights by interpolation along edge parameter t
				const float K0 = 1.0f - t;
				const float K1 = t;
				OutWA = K0 * wA0 + K1 * wA1;
				OutWB = K0 * wB0 + K1 * wB1;
				OutWC = K0 * wC0 + K1 * wC1;
			}
		};

		/// AB: weights at A=(1,0,0), B=(0,1,0)
		ConsiderEdge(A, B, 1, 0, 0, 0, 1, 0);
		/// BC: weights at B=(0,1,0), C=(0,0,1)
		ConsiderEdge(B, C, 0, 1, 0, 0, 0, 1);
		/// CA: weights at C=(0,0,1), A=(1,0,0)
		ConsiderEdge(C, A, 0, 0, 1, 1, 0, 0);

		/// Candidate 3 (safety net): closest point from segment endpoints to triangle (covers some degenerate cases)
		{
			float WA = 0.0f;
			float WB = 0.0f;
			float WC = 0.0f;
			const FVector Qt0 = ClosestPointOnTriangleWeights(OUT WA, OUT WB, OUT WC, Seg0, A, B, C);
			const float D20 = (Seg0 - Qt0).SizeSquared();
			if (D20 < OutDistSQ)
			{
				OutDistSQ = D20;
				OutPc = Seg0;
				OutQt = Qt0;
				OutWA = WA;
				OutWB = WB;
				OutWC = WC;
			}

			const FVector Qt1 = ClosestPointOnTriangleWeights(OUT WA, OUT WB, OUT WC, Seg1, A, B, C);
			const float D21 = (Seg1 - Qt1).SizeSquared();
			if (D21 < OutDistSQ)
			{
				OutDistSQ = D21;
				OutPc = Seg1;
				OutQt = Qt1;
				OutWA = WA;
				OutWB = WB;
				OutWC = WC;
			}
		}
	}

	/// ------------------------------
	/// Signed distance from point to (expanded) AABB at origin.
	/// Returns:
	///  - OutSignedDist: >0 outside, <0 inside (penetration)
	///  - OutNormal: outward normal (pointing from box to point) in box-local
	///  - OutClosest: closest point on box surface if outside, equals point if inside
	/// ------------------------------
	inline void SignedDistancePointAABB(OUT float& OutSignedDist, OUT FVector& OutNormal, OUT FVector& OutClosest, const FVector& P, const FVector& HalfExtents)
	{
		const FVector Clamped(FMath::Clamp(P.X, -HalfExtents.X, HalfExtents.X), FMath::Clamp(P.Y, -HalfExtents.Y, HalfExtents.Y), FMath::Clamp(P.Z, -HalfExtents.Z, HalfExtents.Z));
		const bool bInside = (FMath::Abs(P.X) <= HalfExtents.X) && (FMath::Abs(P.Y) <= HalfExtents.Y) && (FMath::Abs(P.Z) <= HalfExtents.Z);

		if (!bInside)
		{
			OutClosest = Clamped;
			const FVector D = P - Clamped;
			const float Dist = D.Size();
			OutSignedDist = Dist;
			OutNormal = (Dist > KINDA_SMALL_NUMBER) ? (D / Dist) : FVector::UpVector;
			return;
		}

		/// Inside: penetration depth = min distance to any face
		const float DX = HalfExtents.X - FMath::Abs(P.X);
		const float DY = HalfExtents.Y - FMath::Abs(P.Y);
		const float DZ = HalfExtents.Z - FMath::Abs(P.Z);

		/// Choose smallest escape distance (most constraining)
		if (DX <= DY && DX <= DZ)
		{
			OutSignedDist = -DX;
			OutNormal = FVector((P.X >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f);
		}
		else if (DY <= DX && DY <= DZ)
		{
			OutSignedDist = -DY;
			OutNormal = FVector(0.0f, (P.Y >= 0.0f) ? 1.0f : -1.0f, 0.0f);
		}
		else
		{
			OutSignedDist = -DZ;
			OutNormal = FVector(0.0f, 0.0f, (P.Z >= 0.0f) ? 1.0f : -1.0f);
		}
		OutClosest = P; /// inside: "closest" is itself for our constraint formulation
	}
};