#pragma once
#include <CoreMinimal.h>
#include <UObject/WeakObjectPtrTemplates.h>
#include "LKAnimVerletBound.h"
#include "LKAnimVerletConstraintType.h"


///=========================================================================================================================================
/// FLKAnimVerletConstraint
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint
{
public:
	virtual ~FLKAnimVerletConstraint() {}
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) = 0;
	virtual void BackwardUpdate(float DeltaTime, bool bInitialUpdate, bool bFinalize) { Update(DeltaTime, bInitialUpdate, bFinalize); }
	virtual void PostUpdate(float DeltaTime) = 0;
	virtual void ResetSimulation() = 0;
};

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Pin
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Pin : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* Bone = nullptr;
	float PinMargin = 0.0f;

public:
	FLKAnimVerletConstraint_Pin(struct FLKAnimVerletBone* InBone, float InPinMargin = 0.0f) 
		: Bone(InBone), PinMargin(InPinMargin) 
	{ 
		verify(Bone != nullptr); 
	}
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Distance
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Distance : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;

	bool bUseXPBDSolver = false;
	bool bStretchEachBone = false;
	float StretchStrength = 0.0f;
	float Stiffness = 0.0f;
	float Length = 0.0f;
	double Lambda = 0.0;		///for XPBD
	double Compliance = 0.0;	///for XPBD

public:
	FLKAnimVerletConstraint_Distance(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, bool bInUseXPBDSolver, 
									 double InStiffness, bool bInStretchEachBone, float InStretchStrength);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override;
	virtual void ResetSimulation() override;
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_IsometricBending
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_IsometricBending : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;
	struct FLKAnimVerletBone* BoneC = nullptr;
	struct FLKAnimVerletBone* BoneD = nullptr;

	bool bUseXPBDSolver = false;
	float Q[4][4] = {};
	float RestAngle = 0.0f;
	float Stiffness = 0.0f;
	double Lambda = 0.0f;		///for XPBD
	double Compliance = 0.0;	///for XPBD

public:
	/// [Edge B-C is shared]
	///	A---B
	///	\  | \
	///	 \ |  \
	///   \|   \
	///	   C----D
	FLKAnimVerletConstraint_IsometricBending(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, 
											 struct FLKAnimVerletBone* InBoneD, bool bInUseXPBDSolver, double InStiffness);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override;
	virtual void ResetSimulation() override;

private:
	void CalculateQMatrix(float Q[4][4], struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, struct FLKAnimVerletBone* InBoneD);
	float CalculateRestAngle(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, struct FLKAnimVerletBone* InBoneD);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Bending_1D
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Bending_1D : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;
	struct FLKAnimVerletBone* BoneC = nullptr;

	bool bUseXPBDSolver = false;
	float RestAngle = 0.0f;
	float Stiffness = 0.0f;
	double Lambda = 0.0f;		///for XPBD
	double Compliance = 0.0;	///for XPBD

public:
	FLKAnimVerletConstraint_Bending_1D(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, bool bInUseXPBDSolver, double InStiffness);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override;
	virtual void ResetSimulation() override;

private:
	float CalculateRestAngle(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_FlatBending
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_FlatBending : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;
	struct FLKAnimVerletBone* BoneC = nullptr;
	struct FLKAnimVerletBone* BoneD = nullptr;

	bool bUseXPBDSolver = false;
	float Stiffness = 0.0f;
	double Lambda = 0.0f;			///for XPBD
	double Compliance = 0.0;		///for XPBD

	float RestAngle = 0.0f;
	float TargetAngle = 0.0f;
	float FlatAlpha = 0.0f;

public:
	/// [Edge B-C is shared]
	///	A---B
	///	\  | \
	///	 \ |  \
	///   \|   \
	///	   C----D
	FLKAnimVerletConstraint_FlatBending(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, 
										struct FLKAnimVerletBone* InBoneD, bool bInUseXPBDSolver, double InStiffness, float InFlatAlpha);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override;
	virtual void ResetSimulation() override;

private:
	float ComputeDihedralAngle_BC(const FVector& A, const FVector& B, const FVector& C, const FVector& D);	///Dihedral angle around shared edge BC (tri0=B,C,A / tri1=C,B,D)
	void ComputeBendingGradients(OUT FVector& GradientsA, OUT FVector& GradientsB, OUT FVector& GradientsC, OUT FVector& GradientsD,
								 const FVector& A, const FVector& B, const FVector& C, const FVector& D);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Straighten
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Straighten : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;
	struct FLKAnimVerletBone* BoneC = nullptr;

	float StraightenStrength = 0.0;
	bool bStraightenCenterBone = false;

public:
	FLKAnimVerletConstraint_Straighten(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, 
									   float InStraightenStrength, bool bInStraightenCenterBone);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_FixedDistance
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_FixedDistance : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;

	bool bStretchEachBone = false;
	bool bAwayFromEachOther = false;
	float StretchStrength = 0.0f;
	float Length = 0.0f;
	float LengthMargin = 0.0f;

public:
	FLKAnimVerletConstraint_FixedDistance(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, bool bInStretchEachBone, 
										  float InStretchStrength, bool bInAwayFromEachOther, float InLengthMargin = 0.0f);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void BackwardUpdate(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_BallSocket
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_BallSocket : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;
	struct FLKAnimVerletBone* GrandParentBoneNullable = nullptr;
	struct FLKAnimVerletBone* ParentBoneNullable = nullptr;


	float AngleDegrees = 0.0f;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;		///for XPBD
	double Lambda = 0.0;			///for XPBD

public:
	FLKAnimVerletConstraint_BallSocket(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InGrandParentNullable, struct FLKAnimVerletBone* InParentNullable,
									   float InAngleDegrees, bool bInUseXPBDSolver, double InCompliance);
	virtual void Update(float DeltaTime, bool bInitialUpdate, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override { Lambda = 0.0; }
	virtual void ResetSimulation() override { Lambda = 0.0; }
};
///=========================================================================================================================================