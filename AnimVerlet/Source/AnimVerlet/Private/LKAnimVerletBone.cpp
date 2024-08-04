#include "LKAnimVerletBone.h"

#include "LKAnimVerletSetting.h"

///=========================================================================================================================================
/// FLKAnimVerletBone
///=========================================================================================================================================
void FLKAnimVerletBone::InitializeTransform(const FTransform& InitialT)
{
	PoseLocation = InitialT.GetLocation();
	PrevPoseLocation = PoseLocation;
	Location = PoseLocation;
	PrevLocation = PoseLocation;

	PoseRotation = InitialT.GetRotation();
	PrevPoseRotation = PoseRotation;
	Rotation = PoseRotation;
	PrevRotation = PoseRotation;

	PoseScale = InitialT.GetScale3D();
}

void FLKAnimVerletBone::SetFakeBoneOffset(const FVector& InLocationOffset)
{
	FakeBoneLocationOffset = InLocationOffset;
}

FVector FLKAnimVerletBone::MakeFakeBonePoseLocation(const FTransform& PoseT) const
{
	return MakeFakeBonePoseTransform(PoseT).GetLocation();
}

FTransform FLKAnimVerletBone::MakeFakeBonePoseTransform(const FTransform& PoseT) const
{
	const FTransform LocationOffsetT(FQuat::Identity, FakeBoneLocationOffset);
	return (PoseT * LocationOffsetT);
}

void FLKAnimVerletBone::PrepareSimulation(const FTransform& PoseT, const FVector& InPoseDirFromParent)
{
	MoveDelta = (Location - PrevLocation);

	PrevPoseLocation = PoseLocation;
	PrevLocation = Location;
	PoseLocation = PoseT.GetLocation();

	PrevPoseRotation = PoseRotation;
	PrevRotation = Rotation;
	PoseRotation = PoseT.GetRotation();

	PoseDirFromParent = InPoseDirFromParent;
	PoseScale = PoseT.GetScale3D();
}

void FLKAnimVerletBone::Update(float DeltaTime, const FLKAnimVerletUpdateParam& InParam)
{
	/// VerletIntegration and Damping
	Location += MoveDelta * InParam.Damping;

	const float CurDeltaTime = InParam.bUseSquaredDeltaTime ? DeltaTime * DeltaTime : DeltaTime;

	/// Component movement
	Location += InParam.ComponentMoveDiff * CurDeltaTime;
	Location += (InParam.ComponentRotDiff.RotateVector(PrevLocation) - PrevLocation) * CurDeltaTime;

	/// Gravity
	Location += InParam.Gravity * CurDeltaTime;

	/// StretchForce
	Location += (PoseDirFromParent * InParam.StretchForce) * CurDeltaTime;

	/// ExternalForce
	Location += InParam.ExternalForce * CurDeltaTime;

	/// RandomWind
	if (InParam.RandomWindDir.IsNearlyZero(KINDA_SMALL_NUMBER) == false)
		Location += InParam.RandomWindDir * FMath::RandRange(InParam.RandomWindSizeMin, InParam.RandomWindSizeMax) * CurDeltaTime;

	/// ShapeMemoryForce
	Location += ((PoseLocation - Location).GetSafeNormal() * InParam.ShapeMemoryForce) * CurDeltaTime;
}

void FLKAnimVerletBone::AdjustPoseTransform(float DeltaTime, const FVector& ParentLocation, const FVector& ParentPoseLocation,
											float AnimationPoseInertia, float AnimationPoseDeltaInertia, bool bClampAnimationPoseDeltaInertia, float AnimationPoseDeltaInertiaClampMax)
{
	/// To Pose
	const FVector CurPoseVecFromParent = PoseLocation - ParentPoseLocation;
	Location += (ParentLocation + CurPoseVecFromParent - Location) * AnimationPoseInertia;

	/// Pose delta from last frame
	const FVector PoseDiff = PoseLocation - PrevPoseLocation;
	if (bClampAnimationPoseDeltaInertia)
	{
		FVector PoseDiffDir = FVector::ZeroVector;
		float PoseDiffSize = 0.0f;
		PoseDiff.ToDirectionAndLength(OUT PoseDiffDir, OUT PoseDiffSize);
		Location += PoseDiffDir * FMath::Min(PoseDiffSize * AnimationPoseDeltaInertia, AnimationPoseDeltaInertiaClampMax);
	}
	else
	{
		Location += PoseDiff * AnimationPoseDeltaInertia;
	}
}

void FLKAnimVerletBone::ResetSimulation()
{
	MoveDelta = FVector::ZeroVector;

	Location = PoseLocation;
	PrevLocation = Location;

	Rotation = PoseRotation;
	PrevRotation = Rotation;
}