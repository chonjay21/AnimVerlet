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

	Velocity = FVector::ZeroVector;
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
	/// XPBD
	/*if (bUseXPBDSolver)
	{
		const float CurDeltaTime = DeltaTime;
		
		/// VerletIntegration and Damping
		Velocity += MoveDelta * InParam.Damping * CurDeltaTime;

		/// Component movement
		Velocity += InParam.ComponentMoveDiff * CurDeltaTime;
		Velocity += (InParam.ComponentRotDiff.RotateVector(PrevLocation) - PrevLocation) * CurDeltaTime;

		/// Gravity
		Velocity += InParam.Gravity * CurDeltaTime;

		/// StretchForce
		Velocity += (PoseDirFromParent * InParam.StretchForce) * CurDeltaTime;

		/// SideStraightenForce
		Velocity += (Rotation.RotateVector(SideStraightenDirInLocal) * InParam.SideStraightenForce) * CurDeltaTime;

		/// ExternalForce
		Velocity += InParam.ExternalForce * CurDeltaTime;

		/// RandomWind
		if (InParam.RandomWindDir.IsNearlyZero(KINDA_SMALL_NUMBER) == false)
			Velocity += InParam.RandomWindDir * FMath::RandRange(InParam.RandomWindSizeMin, InParam.RandomWindSizeMax) * CurDeltaTime;

		/// ShapeMemoryForce
		Velocity += ((PoseLocation - Location).GetSafeNormal() * InParam.ShapeMemoryForce) * CurDeltaTime;

		Location += Velocity * DeltaTime;
	}
	/// PBD
	else*/
	{
		const float CurDeltaTime = InParam.bUseSquaredDeltaTime ? DeltaTime * DeltaTime : DeltaTime;
		
		/// VerletIntegration and Damping
		Location += MoveDelta * InParam.Damping;

		/// Component movement
		Location += InParam.ComponentMoveDiff * CurDeltaTime;
		Location += (InParam.ComponentRotDiff.RotateVector(PrevLocation) - PrevLocation) * CurDeltaTime;

		/// Gravity
		Location += InParam.Gravity * CurDeltaTime;

		/// StretchForce
		Location += (PoseDirFromParent * InParam.StretchForce) * CurDeltaTime;

		/// SideStraightenForce
		Location += (Rotation.RotateVector(SideStraightenDirInLocal) * InParam.SideStraightenForce) * CurDeltaTime;

		/// ExternalForce
		Location += InParam.ExternalForce * CurDeltaTime;

		/// RandomWind
		if (InParam.RandomWindDir.IsNearlyZero(KINDA_SMALL_NUMBER) == false)
			Location += InParam.RandomWindDir * FMath::RandRange(InParam.RandomWindSizeMin, InParam.RandomWindSizeMax) * CurDeltaTime;

		/// ShapeMemoryForce
		Location += ((PoseLocation - Location).GetSafeNormal() * InParam.ShapeMemoryForce) * CurDeltaTime;
	}
}

void FLKAnimVerletBone::PostUpdate(float DeltaTime)
{
	Velocity = FMath::IsNearlyZero(DeltaTime) ? FVector::ZeroVector : (Location - PrevLocation) / DeltaTime;
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

void FLKAnimVerletBone::Sleep()
{
	bSleep = true;
	SleepTriggerElapsedTime = 0.0f;

	Location = PrevLocation;
	Rotation = PrevRotation;
}

void FLKAnimVerletBone::WakeUp()
{
	bSleep = false;
	SleepTriggerElapsedTime = 0.0f;
}

void FLKAnimVerletBone::ResetSimulation()
{
	MoveDelta = FVector::ZeroVector;

	Location = PoseLocation;
	PrevLocation = Location;

	Rotation = PoseRotation;
	PrevRotation = Rotation;

	Velocity = FVector::ZeroVector;

	bSleep = false;
	SleepTriggerElapsedTime = 0.0f;
}


///=========================================================================================================================================
/// FLKAnimVerletExcludedBone
///=========================================================================================================================================
void FLKAnimVerletExcludedBone::PrepareSimulation(const FTransform& PoseT)
{
	PoseLocation = PoseT.GetLocation();
	Location = PoseLocation;
	PoseRotation = PoseT.GetRotation();
	Rotation = PoseRotation;

	PoseScale = PoseT.GetScale3D();
}