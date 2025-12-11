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
	virtual void Update(float DeltaTime, bool bFinalize) = 0;
	virtual void BackwardUpdate(float DeltaTime, bool bFinalize) { Update(DeltaTime, bFinalize); }
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override;
	virtual void ResetSimulation() override;

private:
	void CalculateQMatrix(float Q[4][4], struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, struct FLKAnimVerletBone* InBoneD);
	float CalculateRestAngle(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, struct FLKAnimVerletBone* InBoneC, struct FLKAnimVerletBone* InBoneD);
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void BackwardUpdate(float DeltaTime, bool bFinalize) override {}
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void BackwardUpdate(float DeltaTime, bool bFinalize) override;
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
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override { Lambda = 0.0; }
	virtual void ResetSimulation() override { Lambda = 0.0; }
};
///=========================================================================================================================================


///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Sphere
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Sphere : public FLKAnimVerletConstraint
{
public:
	FVector Location = FVector::ZeroVector;
	float Radius = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;								///for XPBD
	TArray<double, TInlineAllocator<64>> Lambdas;			///for XPBD

public:
	FLKAnimVerletConstraint_Sphere(const FVector& InLocation, float InRadius, const FLKAnimVerletCollisionConstraintInput& InCollisionInput);
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override { Lambdas.Reset(); }
	virtual void ResetSimulation() override { Lambdas.Reset(); }

	inline FLKAnimVerletBound MakeBound() const { return FLKAnimVerletBound::MakeBoundFromCenterHalfExtents(Location, FVector(Radius, Radius, Radius)); }

private:
	bool CheckSphereSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, int32 LambdaIndex);
	void CheckSphereSphere(float DeltaTime, bool bFinalize);
	bool CheckSphereCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, int32 LambdaIndex);
	void CheckSphereCapsule(float DeltaTime, bool bFinalize);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Capsule
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Capsule : public FLKAnimVerletConstraint
{
public:
	FVector Location = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	float Radius = 0.0f;
	float HalfHeight = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;								///for XPBD
	TArray<double, TInlineAllocator<64>> Lambdas;			///for XPBD

public:
	FLKAnimVerletConstraint_Capsule(const FVector& InLocation, const FQuat& InRot, float InRadius, 
									float InHalfHeight, const FLKAnimVerletCollisionConstraintInput& InCollisionInput);
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override { Lambdas.Reset(); }
	virtual void ResetSimulation() override { Lambdas.Reset(); }

	FLKAnimVerletBound MakeBound() const;

private:
	bool CheckCapsuleSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex);
	void CheckCapsuleSphere(float DeltaTime, bool bFinalize);
	bool CheckCapsuleCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex);
	void CheckCapsuleCapsule(float DeltaTime, bool bFinalize);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Box
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Box : public FLKAnimVerletConstraint
{
public:
	FVector Location = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	FVector HalfExtents = FVector::ZeroVector;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;								///for XPBD
	TArray<double, TInlineAllocator<64>> Lambdas;			///for XPBD

public:
	FLKAnimVerletConstraint_Box(const FVector& InLocation, const FQuat& InRot, const FVector& InHalfExtents, const FLKAnimVerletCollisionConstraintInput& InCollisionInput);
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override { Lambdas.Reset(); }
	virtual void ResetSimulation() override { Lambdas.Reset(); }

	FLKAnimVerletBound MakeBound() const;

private:
	bool IntersectOriginAabbSphere(OUT FVector& OutCollisionNormal, OUT float& OutPenetrationDepth, IN OUT FLKAnimVerletBone& CurVerletBone, const FVector& SphereLocation);
	bool IntersectObbSphere(OUT FVector& OutCollisionNormal, OUT float& OutPenetrationDepth, IN OUT FLKAnimVerletBone& CurVerletBone, const FVector& SphereLocation, const FQuat& InvRotation);
	bool CheckBoxSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const FVector& SphereLocation, const FQuat& InvRotation, int32 LambdaIndex);
	void CheckBoxSphere(float DeltaTime, bool bFinalize);
	bool CheckBoxCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex);
	void CheckBoxCapsule(float DeltaTime, bool bFinalize);

	bool CheckBoxBox(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Plane
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_Plane : public FLKAnimVerletConstraint
{
public:
	FVector PlaneBase = FVector::ZeroVector;	/// Point on the plane
	FVector PlaneNormal = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	FVector2D PlaneHalfExtents = FVector2D::ZeroVector;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;

	bool bUseXPBDSolver = false;
	double Compliance = 0.0;								///for XPBD
	TArray<double, TInlineAllocator<64>> Lambdas;			///for XPBD

public:
	FLKAnimVerletConstraint_Plane(const FVector& InPlaneBase, const FVector& InPlaneNormal, const FQuat& InRotation, 
								  const FVector2D& InPlaneHalfExtents, const FLKAnimVerletCollisionConstraintInput& InCollisionInput);
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override { Lambdas.Reset(); }
	virtual void ResetSimulation() override { Lambdas.Reset(); }

private:
	bool CheckPlaneSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex);
	void CheckPlaneSphere(float DeltaTime, bool bFinalize);
	bool CheckPlaneCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex);
	void CheckPlaneCapsule(float DeltaTime, bool bFinalize);
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_World
///=========================================================================================================================================
struct ANIMVERLET_API FLKAnimVerletConstraint_World : public FLKAnimVerletConstraint
{
public:
	TWeakObjectPtr<const class UWorld> WorldPtr = nullptr;
	TWeakObjectPtr<class UPrimitiveComponent> SelfComponentPtr = nullptr;
	FName WorldCollisionProfileName = NAME_None;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

	bool bUseCapsuleCollisionForChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;

public:
	FLKAnimVerletConstraint_World(const class UWorld* InWorld, class UPrimitiveComponent* InSelfComponent, const FName& InCollisionProfileName, const FLKAnimVerletCollisionConstraintInput& InCollisionInput);
	virtual void Update(float DeltaTime, bool bFinalize) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}

private:
	bool CheckWorldSphere(IN OUT FLKAnimVerletBone& CurVerletBone, float DeltaTime, bool bFinalize, const UWorld* World,
						  const struct FCollisionQueryParams& CollisionQueryParams, const FTransform& ComponentTransform, int32 LambdaIndex);
	void CheckWorldSphere(float DeltaTime, bool bFinalize);
	bool CheckWorldCapsule(IN OUT FLKAnimVerletBone& CurVerletBone, IN OUT FLKAnimVerletBone& ParentVerletBone, float DeltaTime, bool bFinalize, 
						   const UWorld* World, const struct FCollisionQueryParams& CollisionQueryParams, const FTransform& ComponentTransform, int32 LambdaIndex);
	void CheckWorldCapsule(float DeltaTime, bool bFinalize);
};
///=========================================================================================================================================