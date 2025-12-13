#pragma once
#include <CoreMinimal.h>
#include <UObject/WeakObjectPtrTemplates.h>
#include "LKAnimVerletBound.h"
#include "LKAnimVerletConstraint.h"
#include "LKAnimVerletConstraintType.h"


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
	bool bSingleChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;
	TArray<FLKAnimVerletBoneIndicatorTriangle>* BoneTriangles = nullptr;

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
	bool CheckSphereTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bFinalize, int32 LambdaIndex);
	void CheckSphereTriangle(float DeltaTime, bool bFinalize);
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
	bool bSingleChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;
	TArray<FLKAnimVerletBoneIndicatorTriangle>* BoneTriangles = nullptr;

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
	bool CheckCapsuleTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bFinalize, const FVector& CapsuleStart, const FVector& CapsuleEnd, int32 LambdaIndex);
	void CheckCapsuleTriangle(float DeltaTime, bool bFinalize);
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
	bool bSingleChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;
	TArray<FLKAnimVerletBoneIndicatorTriangle>* BoneTriangles = nullptr;

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

	bool CheckBoxTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bFinalize, const FQuat& InvRotation, int32 LambdaIndex);
	void CheckBoxTriangle(float DeltaTime, bool bFinalize);
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
	bool bSingleChain = false;
	TArray<FLKAnimVerletBoneIndicatorPair>* BonePairs = nullptr;
	TArray<FLKAnimVerletBoneIndicatorTriangle>* BoneTriangles = nullptr;

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
	bool CheckPlaneTriangle(IN OUT FLKAnimVerletBone& BoneA, IN OUT FLKAnimVerletBone& BoneB, IN OUT FLKAnimVerletBone& BoneC, float DeltaTime, bool bFinalize, bool bFinitePlane, const FQuat& InvRotation, int32 LambdaIndex);
	void CheckPlaneTriangle(float DeltaTime, bool bFinalize);
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