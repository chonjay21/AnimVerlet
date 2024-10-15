#pragma once
#include <CoreMinimal.h>
#include <UObject/WeakObjectPtrTemplates.h>

using TExcludeBoneBits = TBitArray<TInlineAllocator<64>>;

///=========================================================================================================================================
/// FLKAnimVerletConstraint
///=========================================================================================================================================
struct FLKAnimVerletConstraint
{
public:
	virtual ~FLKAnimVerletConstraint() {}
	virtual void Update(float DeltaTime) = 0;
	virtual void BackwardUpdate(float DeltaTime) { Update(DeltaTime); }
	virtual void PostUpdate(float DeltaTime) = 0;
	virtual void ResetSimulation() = 0;
};

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Pin
///=========================================================================================================================================
struct FLKAnimVerletConstraint_Pin : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* Bone = nullptr;
	float PinMargin = 0.0f;

public:
	FLKAnimVerletConstraint_Pin(struct FLKAnimVerletBone* InBone, float InPinMargin = 0.0f) : Bone(InBone), PinMargin(InPinMargin) { verify(Bone != nullptr); }
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Distance
///=========================================================================================================================================
struct FLKAnimVerletConstraint_Distance : public FLKAnimVerletConstraint
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
	FLKAnimVerletConstraint_Distance(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, bool bInUseXPBDSolver, double InStiffness, bool bInStretchEachBone, float InStretchStrength);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override;
	virtual void ResetSimulation() override;
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_FixedDistance
///=========================================================================================================================================
struct FLKAnimVerletConstraint_FixedDistance : public FLKAnimVerletConstraint
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
	FLKAnimVerletConstraint_FixedDistance(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, bool bInStretchEachBone, float InStretchStrength, bool bInAwayFromEachOther, float InLengthMargin = 0.0f);
	virtual void Update(float DeltaTime) override;
	virtual void BackwardUpdate(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_BallSocket
///=========================================================================================================================================
struct FLKAnimVerletConstraint_BallSocket : public FLKAnimVerletConstraint
{
public:
	struct FLKAnimVerletBone* BoneA = nullptr;
	struct FLKAnimVerletBone* BoneB = nullptr;

	float AngleDegrees = 0.0f;

public:
	FLKAnimVerletConstraint_BallSocket(struct FLKAnimVerletBone* InBoneA, struct FLKAnimVerletBone* InBoneB, float InAngleDegrees);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================


///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Sphere
///=========================================================================================================================================
struct FLKAnimVerletConstraint_Sphere : public FLKAnimVerletConstraint
{
public:
	FVector Location = FVector::ZeroVector;
	float Radius = 0.0f;
	float Thickness = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

public:
	FLKAnimVerletConstraint_Sphere(const FVector& InLocation, float InRadius, float InThickness,
								   TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Capsule
///=========================================================================================================================================
struct FLKAnimVerletConstraint_Capsule : public FLKAnimVerletConstraint
{
public:
	FVector Location = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	float Radius = 0.0f;
	float HalfHeight = 0.0f;
	float Thickness = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

public:
	FLKAnimVerletConstraint_Capsule(const FVector& InLocation, const FQuat& InRot, float InRadius, float InHalfHeight, float InThickness, 
									TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Box
///=========================================================================================================================================
struct FLKAnimVerletConstraint_Box : public FLKAnimVerletConstraint
{
public:
	FVector Location = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	FVector HalfExtents = FVector::ZeroVector;
	float Thickness = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

public:
	FLKAnimVerletConstraint_Box(const FVector& InLocation, const FQuat& InRot, const FVector& InHalfExtents, float InThickness, 
								TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_Plane
///=========================================================================================================================================
struct FLKAnimVerletConstraint_Plane : public FLKAnimVerletConstraint
{
public:
	FVector PlaneBase = FVector::ZeroVector;	/// Point on the plane
	FVector PlaneNormal = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	FVector2D PlaneHalfExtents = FVector2D::ZeroVector;
	float Thickness = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

public:
	FLKAnimVerletConstraint_Plane(const FVector& InPlaneBase, const FVector& InPlaneNormal, const FQuat& InRotation, const FVector2D& InPlaneHalfExtents, 
								  float InThickness, TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================

///=========================================================================================================================================
/// CollisionConstraint
/// FLKAnimVerletConstraint_World
///=========================================================================================================================================
struct FLKAnimVerletConstraint_World : public FLKAnimVerletConstraint
{
public:
	TWeakObjectPtr<const class UWorld> WorldPtr = nullptr;
	TWeakObjectPtr<class UPrimitiveComponent> SelfComponentPtr = nullptr;
	FName WorldCollisionProfileName = NAME_None;
	float Thickness = 0.0f;
	TArray<FLKAnimVerletBone>* Bones = nullptr;
	TExcludeBoneBits ExcludeBones;

public:
	FLKAnimVerletConstraint_World(const class UWorld* InWorld, class UPrimitiveComponent* InSelfComponent, const FName& InCollisionProfileName,
								  float InThickness, TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones);
	virtual void Update(float DeltaTime) override;
	virtual void PostUpdate(float DeltaTime) override {}
	virtual void ResetSimulation() override {}
};
///=========================================================================================================================================