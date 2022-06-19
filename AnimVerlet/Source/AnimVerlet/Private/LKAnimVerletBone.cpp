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

void FLKAnimVerletBone::PrepareSimulation(const FTransform& PoseT)
{
	MoveDelta = (Location - PrevLocation);

	PrevPoseLocation = PoseLocation;
	PrevLocation = Location;
	PoseLocation = PoseT.GetLocation();

	PrevPoseRotation = PoseRotation;
	PrevRotation = Rotation;
	PoseRotation = PoseT.GetRotation();

	PoseScale = PoseT.GetScale3D();
}

void FLKAnimVerletBone::Update(float DeltaTime, const FLKAnimVerletUpdateParam& InParam)
{
	/// VerletIntegration and Damping
	Location += MoveDelta * InParam.Damping;

	/// Component movement
	Location += InParam.ComponentMoveDiff * DeltaTime;
	Location += (InParam.ComponentRotDiff.RotateVector(PrevLocation) - PrevLocation) * DeltaTime;

	/// Gravity
	Location += InParam.Gravity * DeltaTime;

	/// ExternalForce
	Location += InParam.ExternalForce * DeltaTime;

	/// RandomWind
	if (InParam.RandomWindDir.IsNearlyZero(KINDA_SMALL_NUMBER) == false)
		Location += InParam.RandomWindDir * DeltaTime * FMath::RandRange(InParam.RandomWindSizeMin, InParam.RandomWindSizeMax);
}

void FLKAnimVerletBone::AdjustPoseTransform(float DeltaTime, const FVector& ParentLocation, const FVector& ParentPoseLocation,
											float ReferencePoseInertia, float ReferencePoseDeltaInertia, bool bClampReferencePoseDeltaInertia, float ReferencePoseDeltaInertiaClampMax)
{
	/// To Pose
	const FVector PoseDirFromParent = PoseLocation - ParentPoseLocation;
	Location += (ParentLocation + PoseDirFromParent - Location) * ReferencePoseInertia;

	/// Pose delta from last frame
	const FVector PoseDiff = PoseLocation - PrevPoseLocation;
	if (bClampReferencePoseDeltaInertia)
	{
		FVector PoseDiffDir = FVector::ZeroVector;
		float PoseDiffSize = 0.0f;
		PoseDiff.ToDirectionAndLength(OUT PoseDiffDir, OUT PoseDiffSize);
		Location += PoseDiffDir * FMath::Min(PoseDiffSize * ReferencePoseDeltaInertia, ReferencePoseDeltaInertiaClampMax);
	}
	else
	{
		Location += PoseDiff * ReferencePoseDeltaInertia;
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