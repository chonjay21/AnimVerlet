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

FLKAnimVerletBound FLKAnimVerletBone::MakePairBound(const FLKAnimVerletBone& BoneA, const FLKAnimVerletBone& BoneB)
{
	const float Thickness = FMath::Max(BoneA.Thickness, BoneB.Thickness);
	const FVector AabbMin(FMath::Min(BoneA.Location.X, BoneB.Location.X) - Thickness, FMath::Min(BoneA.Location.Y, BoneB.Location.Y) - Thickness, FMath::Min(BoneA.Location.Z, BoneB.Location.Z) - Thickness);
	const FVector AabbMax(FMath::Max(BoneA.Location.X, BoneB.Location.X) + Thickness, FMath::Max(BoneA.Location.Y, BoneB.Location.Y) + Thickness, FMath::Max(BoneA.Location.Z, BoneB.Location.Z) + Thickness);
	return FLKAnimVerletBound::MakeBoundFromMinMax(AabbMin, AabbMax);
}

FLKAnimVerletBound FLKAnimVerletBone::MakeTriangleBound(const FLKAnimVerletBone& BoneA, const FLKAnimVerletBone& BoneB, const FLKAnimVerletBone& BoneC)
{
	const float Thickness = FMath::Max3(BoneA.Thickness, BoneB.Thickness, BoneC.Thickness);
	const FVector AabbMin(FMath::Min3(BoneA.Location.X, BoneB.Location.X, BoneC.Location.X) - Thickness, FMath::Min3(BoneA.Location.Y, BoneB.Location.Y, BoneC.Location.Y) - Thickness, FMath::Min3(BoneA.Location.Z, BoneB.Location.Z, BoneC.Location.Z) - Thickness);
	const FVector AabbMax(FMath::Max3(BoneA.Location.X, BoneB.Location.X, BoneC.Location.X) + Thickness, FMath::Max3(BoneA.Location.Y, BoneB.Location.Y, BoneC.Location.Y) + Thickness, FMath::Max3(BoneA.Location.Z, BoneB.Location.Z, BoneC.Location.Z) + Thickness);
	return FLKAnimVerletBound::MakeBoundFromMinMax(AabbMin, AabbMax);
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
	const float CurDeltaTime = InParam.bUseSquaredDeltaTime ? DeltaTime * DeltaTime : DeltaTime;
	const float ForceMassScale = InvMass;
		
	/// VerletIntegration and Damping
	Location += MoveDelta * InParam.Damping;

	/// Rebase the complete Verlet state from the previous component frame into the current component frame.
	if (InParam.ComponentMoveDiff.IsNearlyZero(KINDA_SMALL_NUMBER) == false || InParam.ComponentRotDiff.Equals(FQuat::Identity, KINDA_SMALL_NUMBER) == false)
	{
		Location = InParam.ComponentRotDiff.RotateVector(Location) + InParam.ComponentMoveDiff;
		PrevLocation = InParam.ComponentRotDiff.RotateVector(PrevLocation) + InParam.ComponentMoveDiff;
		Rotation = (InParam.ComponentRotDiff * Rotation).GetNormalized();
		PrevRotation = (InParam.ComponentRotDiff * PrevRotation).GetNormalized();
	}

	/// Gravity
	Location += InParam.Gravity * CurDeltaTime;

	/// StretchForce
	Location += (PoseDirFromParent * InParam.StretchForce) * (CurDeltaTime * ForceMassScale);

	/// SideStraightenForce
	Location += (Rotation.RotateVector(SideStraightenDirInLocal) * InParam.SideStraightenForce) * (CurDeltaTime * ForceMassScale);

	/// ExternalForce
	Location += InParam.ExternalForce * (CurDeltaTime * ForceMassScale);

	/// RandomWind
	{
		if (InParam.RandomWind.RandomForceDirection.IsNearlyZero(KINDA_SMALL_NUMBER) == false)
			Location += InParam.RandomWind.RandomForceDirection * FMath::RandRange(InParam.RandomWind.RandomForceSizeMin, InParam.RandomWind.RandomForceSizeMax) * (CurDeltaTime * ForceMassScale);

		for (const FLKAnimVerletRandomForceSetting& CurWind : InParam.AdditionalRandomWinds)
		{
			if (CurWind.RandomForceDirection.IsNearlyZero(KINDA_SMALL_NUMBER) == false)
				Location += CurWind.RandomForceDirection * FMath::RandRange(CurWind.RandomForceSizeMin, CurWind.RandomForceSizeMax) * (CurDeltaTime * ForceMassScale);
		}
	}

	/// ShapeMemoryForce
	Location += ((PoseLocation - Location).GetSafeNormal() * InParam.ShapeMemoryForce) * (CurDeltaTime * ForceMassScale);
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


///=========================================================================================================================================
/// FLKAnimVerletBoneIndicatorPair
///=========================================================================================================================================
FLKAnimVerletBound FLKAnimVerletBoneIndicatorPair::MakeBound(const TArray<FLKAnimVerletBone>& Bones) const
{ 
	const bool bValidA = Bones.IsValidIndex(BoneA.AnimVerletBoneIndex);
	const bool bValidB = Bones.IsValidIndex(BoneB.AnimVerletBoneIndex);
	if (bValidA && bValidB)
		return FLKAnimVerletBone::MakePairBound(Bones[BoneA.AnimVerletBoneIndex], Bones[BoneB.AnimVerletBoneIndex]);
	else if (bValidA)
		return Bones[BoneA.AnimVerletBoneIndex].MakeBound();
	else if (bValidB)
		return Bones[BoneB.AnimVerletBoneIndex].MakeBound();

	return FLKAnimVerletBound();
}


///=========================================================================================================================================
/// FLKAnimVerletBoneIndicatorTriangle
///=========================================================================================================================================
FLKAnimVerletBound FLKAnimVerletBoneIndicatorTriangle::MakeBound(const TArray<FLKAnimVerletBone>& Bones) const
{ 
	const bool bValidA = Bones.IsValidIndex(BoneA.AnimVerletBoneIndex);
	const bool bValidB = Bones.IsValidIndex(BoneB.AnimVerletBoneIndex);
	const bool bValidC = Bones.IsValidIndex(BoneC.AnimVerletBoneIndex);
	if (bValidA && bValidB && bValidC)
		return FLKAnimVerletBone::MakeTriangleBound(Bones[BoneA.AnimVerletBoneIndex], Bones[BoneB.AnimVerletBoneIndex], Bones[BoneC.AnimVerletBoneIndex]);
	else if (bValidA && bValidB)
		return FLKAnimVerletBone::MakePairBound(Bones[BoneA.AnimVerletBoneIndex], Bones[BoneB.AnimVerletBoneIndex]);
	else if (bValidB && bValidC)
		return FLKAnimVerletBone::MakePairBound(Bones[BoneB.AnimVerletBoneIndex], Bones[BoneC.AnimVerletBoneIndex]);
	else if (bValidA && bValidC)
		return FLKAnimVerletBone::MakePairBound(Bones[BoneA.AnimVerletBoneIndex], Bones[BoneC.AnimVerletBoneIndex]);
	else if (bValidA)
		return Bones[BoneA.AnimVerletBoneIndex].MakeBound();
	else if (bValidB)
		return Bones[BoneB.AnimVerletBoneIndex].MakeBound();
	else if (bValidC)
		return Bones[BoneC.AnimVerletBoneIndex].MakeBound();

	return FLKAnimVerletBound();
}