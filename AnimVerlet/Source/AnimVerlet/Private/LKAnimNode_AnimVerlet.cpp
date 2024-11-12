#include "LKAnimNode_AnimVerlet.h"

#include <AnimationRuntime.h>
#include <Animation/AnimInstanceProxy.h>
#include <Animation/AnimTypes.h>
#include <DrawDebugHelpers.h>
#include <Kismet/KismetSystemLibrary.h>
#include "LKAnimVerletBroadphaseContainer.h"
#include "LKAnimVerletCollisionData.h"

#if LK_ENABLE_ANIMVERLET_DEBUG
static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletEnable(TEXT("a.AnimNode.AnimVerlet.Enable"), true, TEXT("Enable/Disable AnimVerlet"));
static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletDebug(TEXT("a.AnimNode.AnimVerlet.Debug"), false, TEXT("Turn on visualization debugging for AnimVerlet"));
static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletDebugBallSocket(TEXT("a.AnimNode.AnimVerlet.Debug.BallSocket"), true, TEXT("Turn on visualization debugging for AnimVerlet`s BallSocket constraints"));
///static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletDebugPlane(TEXT("a.AnimNode.AnimVerlet.Debug.Plane"), true, TEXT("Turn on visualization debugging for AnimVerlet`s Plane constraints"));
static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletDebugSphereCollision(TEXT("a.AnimNode.AnimVerlet.Debug.SphereCollision"), true, TEXT("Turn on visualization debugging for AnimVerlet`s Sphere collision constraints"));
#if (ENGINE_MINOR_VERSION >= 4)
static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletDebugCapsuleCollision(TEXT("a.AnimNode.AnimVerlet.Debug.CapsuleCollision"), true, TEXT("Turn on visualization debugging for AnimVerlet`s Capsule collision constraints"));
#endif
///static TAutoConsoleVariable<bool> CVarAnimNodeAnimVerletDebugBoxCollision(TEXT("a.AnimNode.AnimVerlet.Debug.BoxCollision"), true, TEXT("Turn on visualization debugging for AnimVerlet`s Box collision constraints"));
#endif

DECLARE_CYCLE_STAT(TEXT("AnimVerlet_PrepareSimulation"), STAT_AnimVerlet_PrepareSimulation, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_PrepareLocalCollisionConstraints"), STAT_AnimVerlet_PrepareLocalCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SimulateVerlet"), STAT_AnimVerlet_SimulateVerlet, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_PreUpdateBones"), STAT_AnimVerlet_PreUpdateBones, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints"), STAT_AnimVerlet_SolveConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_PinConstraints"), STAT_AnimVerlet_SolveConstraints_PinConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_DistanceConstraints"), STAT_AnimVerlet_SolveConstraints_DistanceConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BendingConstraints"), STAT_AnimVerlet_SolveConstraints_BendingConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_StraightenConstraints"), STAT_AnimVerlet_SolveConstraints_StraightenConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BallSocketConstraints"), STAT_AnimVerlet_SolveConstraints_BallSocketConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_SphereCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_SphereCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_CapsuleCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_CapsuleCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BoxCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_BoxCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_PlaneCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_PlaneCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_WorldCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_WorldCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_FixedDistanceConstraints"), STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_UpdateBroadphase"), STAT_AnimVerlet_UpdateBroadphase, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_UpdateSleep"), STAT_AnimVerlet_UpdateSleep, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_PostUpdateBones"), STAT_AnimVerlet_PostUpdateBones, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_ApplyResult"), STAT_AnimVerlet_ApplyResult, STATGROUP_Anim);


FLKAnimNode_AnimVerlet::FLKAnimNode_AnimVerlet()
	: FAnimNode_SkeletalControlBase()
{

}

void FLKAnimNode_AnimVerlet::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	
	FBoneContainer& RequiredBones = Context.AnimInstanceProxy->GetRequiredBones();
	InitializeBoneReferences(RequiredBones);

	ClearSimulateBones();
}

void FLKAnimNode_AnimVerlet::ResetDynamics(ETeleportType InTeleportType)
{
	if (InTeleportType == ETeleportType::ResetPhysics)
		ResetSimulation();
}

void FLKAnimNode_AnimVerlet::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	/// Validity check
	if (Output.AnimInstanceProxy == nullptr || Output.AnimInstanceProxy->GetSkelMeshComponent() == nullptr)
		return;
	if (Output.AnimInstanceProxy->GetSkelMeshComponent()->GetWorld() == nullptr)
		return;
	if (bSkipUpdateOnDedicatedServer && UKismetSystemLibrary::IsDedicatedServer(Output.AnimInstanceProxy->GetSkelMeshComponent()))
		return;

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();
	for (const FLKAnimVerletBoneSetting& CurBoneSetting : VerletBones)
	{
		if (CurBoneSetting.RootBone.IsValidToEvaluate(BoneContainer) == false)
			return;
	}
	

	/// Initialize simulate bones
	const FTransform CurComponentT = Output.AnimInstanceProxy->GetComponentTransform();
	if (SimulateBones.Num() == 0)
	{
		InitializeSimulateBones(Output, BoneContainer);
		PrevComponentT = CurComponentT;
	}

	/// Prepare each SimulateBones
	PrepareSimulation(Output, BoneContainer, CurComponentT);

	/// Simulate verlet integration
	if (DeltaTime > 0.0f)
	{
		const USkeletalMeshComponent* SkeletalMeshComponent = Output.AnimInstanceProxy->GetSkelMeshComponent();
		const UWorld* World = SkeletalMeshComponent->GetWorld();
		SimulateVerlet(World, DeltaTime, CurComponentT, PrevComponentT);
	}

	/// Apply simulation to bone
	ApplyResult(OutBoneTransforms, BoneContainer);

	PrevComponentT = CurComponentT;

#if LK_ENABLE_ANIMVERLET_DEBUG
	if (CVarAnimNodeAnimVerletDebug.GetValueOnAnyThread())
	{
		DebugDrawAnimVerlet(Output);
	}
#endif
}

void FLKAnimNode_AnimVerlet::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	for (FLKAnimVerletBoneSetting& CurBoneSetting : VerletBones)
	{
		CurBoneSetting.RootBone.Initialize(RequiredBones);

		for (FBoneReference& CurExcludeBoneSetting : CurBoneSetting.ExcludeBones)
		{
			CurExcludeBoneSetting.Initialize(RequiredBones);
		}

		for (FLKAnimVerletBoneUnitSetting& CurBoneUnitSetting : CurBoneSetting.BoneUnitSettingOverride)
		{
			CurBoneUnitSetting.Bone.Initialize(RequiredBones);
		}
	}

	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		SimulateBones[i].BoneReference.Initialize(RequiredBones);
	}

	for (int32 i = 0; i < ExcludedBones.Num(); ++i)
	{
		ExcludedBones[i].BoneReference.Initialize(RequiredBones);
	}
}

bool FLKAnimNode_AnimVerlet::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
#if LK_ENABLE_ANIMVERLET_DEBUG
	if (CVarAnimNodeAnimVerletEnable.GetValueOnAnyThread() == false)
	{
		return false;
	}
#endif

	for (const FLKAnimVerletBoneSetting& CurBoneSetting : VerletBones)
	{
		if (CurBoneSetting.RootBone.IsValidToEvaluate(RequiredBones) == false)
		{
			return false;
		}
	}
	return true;
}

void FLKAnimNode_AnimVerlet::UpdateInternal(const FAnimationUpdateContext& Context)
{
	FAnimNode_SkeletalControlBase::UpdateInternal(Context);

	UpdateDeltaTime(Context.GetDeltaTime(), Context.AnimInstanceProxy != nullptr ? Context.AnimInstanceProxy->GetTimeDilation() : 1.0f);
}

void FLKAnimNode_AnimVerlet::InitializeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
	verify(SimulateBones.Num() == 0);

	USkeleton* Skeleton = BoneContainer.GetSkeletonAsset();
	const FReferenceSkeleton& ReferenceSkeleton = Skeleton->GetReferenceSkeleton();

	/// Create simulate bones
	for (const FLKAnimVerletBoneSetting& CurBoneSetting : VerletBones)
	{
		const int32 FoundBoneIndex = ReferenceSkeleton.FindBoneIndex(CurBoneSetting.RootBone.BoneName);
		if (FoundBoneIndex != INDEX_NONE)
			MakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, FoundBoneIndex, INDEX_NONE, INDEX_NONE, CurBoneSetting, false, INDEX_NONE);
	}

	/// Create constraints
	FLKAnimVerletBound BoneBound;
	FLKAnimVerletBound BoneMaxBound;
	const double Compliance = static_cast<double>(1.0 / InvCompliance);
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurSimulateBone = SimulateBones[i];
		const FLKAnimVerletBound CurBound = CurSimulateBone.MakeBound();
		BoneBound.Expand(CurBound);

		if (CurBound.GetHalfExtents().SizeSquared() > BoneMaxBound.GetHalfExtents().SizeSquared())
			BoneMaxBound = CurBound;

		if (CurSimulateBone.HasParentBone() == false)
		{
			const FLKAnimVerletBoneIndicatorPair DistancePair(FLKAnimVerletBoneIndicator(INDEX_NONE, false), FLKAnimVerletBoneIndicator(i, false));
			SimulateBonePairIndicators.Emplace(DistancePair);

			const FLKAnimVerletConstraint_Pin PinConstraint(&CurSimulateBone, StartBoneLockMargin);
			PinConstraints.Emplace(PinConstraint);

			CurSimulateBone.bPinned = true;
			CurSimulateBone.PinMargin = StartBoneLockMargin;
		}
		else
		{
			FLKAnimVerletBone& ParentSimulateBone = SimulateBones[CurSimulateBone.ParentVerletBoneIndex];
			const FLKAnimVerletBoneIndicatorPair DistancePair(FLKAnimVerletBoneIndicator(CurSimulateBone.ParentVerletBoneIndex, false), FLKAnimVerletBoneIndicator(i, false));
			SimulateBonePairIndicators.Emplace(DistancePair);

			const FLKAnimVerletConstraint_Distance DistanceConstraint(&ParentSimulateBone, &CurSimulateBone, bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
			DistanceConstraints.Emplace(DistanceConstraint);

			if (bPreserveLengthFromParent)
			{
				const FLKAnimVerletConstraint_FixedDistance FixedDistanceConstraint(&ParentSimulateBone, &CurSimulateBone, bStretchEachBone, StretchStrength, false, LengthFromParentMargin);
				FixedDistanceConstraints.Emplace(FixedDistanceConstraint);
			}

			if (CurSimulateBone.ConeAngleConstraint > 0.0f)
			{
				FLKAnimVerletBone* GrandParentBoneNullable = nullptr;
				FLKAnimVerletBone* ParentBoneNullable = nullptr;
				if (CurSimulateBone.bConstrainConeAngleFromParent && ParentSimulateBone.HasParentBone())
				{
					GrandParentBoneNullable = &SimulateBones[ParentSimulateBone.ParentVerletBoneIndex];
					ParentBoneNullable = &ParentSimulateBone;
				}

				const FLKAnimVerletConstraint_BallSocket BallSocketConstraint(&ParentSimulateBone, &CurSimulateBone, GrandParentBoneNullable, ParentBoneNullable,
																			  CurSimulateBone.ConeAngleConstraint, bUseXPBDSolver, Compliance);
				BallSocketConstraints.Emplace(BallSocketConstraint);
			}
			else if (ConeAngle > 0.0f)
			{
				FLKAnimVerletBone* GrandParentBoneNullable = nullptr;
				FLKAnimVerletBone* ParentBoneNullable = nullptr;
				if (bConstrainConeAngleFromParent && ParentSimulateBone.HasParentBone())
				{
					GrandParentBoneNullable = &SimulateBones[ParentSimulateBone.ParentVerletBoneIndex];
					ParentBoneNullable = &ParentSimulateBone;
				}

				const FLKAnimVerletConstraint_BallSocket BallSocketConstraint(&ParentSimulateBone, &CurSimulateBone, GrandParentBoneNullable, ParentBoneNullable,
																			  ConeAngle, bUseXPBDSolver, Compliance);
				BallSocketConstraints.Emplace(BallSocketConstraint);
			}

			/// Force Lock
			if (CurSimulateBone.IsPinned())
			{
				const FLKAnimVerletConstraint_Pin PinConstraint(&CurSimulateBone, CurSimulateBone.PinMargin);
				PinConstraints.Emplace(PinConstraint);

				CurSimulateBone.bPinned = true;
			}
			else if (bLockTipBone)
			{
				if (CurSimulateBone.IsTipBone())
				{
					const FLKAnimVerletConstraint_Pin PinConstraint(&CurSimulateBone, TipBoneLockMargin);
					PinConstraints.Emplace(PinConstraint);

					CurSimulateBone.bPinned = true;
					CurSimulateBone.PinMargin = TipBoneLockMargin;
				}
			}

			if (bStraightenBendedBone)
			{
				if (ParentSimulateBone.HasParentBone())
				{
					FLKAnimVerletBone& GrandParentSimulateBone = SimulateBones[ParentSimulateBone.ParentVerletBoneIndex];
					const FLKAnimVerletConstraint_Straighten StraightenConstraint(&GrandParentSimulateBone, &ParentSimulateBone, &CurSimulateBone, StraightenBendedBoneStrength, false);
					StraightenConstraints.Emplace(StraightenConstraint);
				}
			}
		}
	}

	/// Side bone constraints for cloth
	if (BoneChainIndexes.Num() > 0)
	{
		for (int32 i = 0; i < MaxBoneChainLength; ++i)
		{
			/// Solve order is important. Make a heuristic order from the center to the outside so that the cloth is stretched out as much as possible.
			const int32 NumBoneChainIndexes = BoneChainIndexes.Num();
			const int32 MidIndex = NumBoneChainIndexes / 2;
			int32 LeftCurIndex = MidIndex;
			int32 LeftIndex = LeftCurIndex - 1;
			int32 RightCurIndex = MidIndex;
			int32 RightIndex = RightCurIndex + 1;

			while (BoneChainIndexes.IsValidIndex(RightIndex) || BoneChainIndexes.IsValidIndex(LeftIndex))
			{
				if (bStraightenBendedBone)
				{
					if (BoneChainIndexes.IsValidIndex(LeftIndex) && BoneChainIndexes.IsValidIndex(RightIndex))
					{
						const TArray<int32>& CurBoneChain = BoneChainIndexes[LeftCurIndex];
						const TArray<int32>& LeftBoneChain = BoneChainIndexes[LeftIndex];
						const TArray<int32>& RightBoneChain = BoneChainIndexes[RightIndex];
						if (i < CurBoneChain.Num() && i < LeftBoneChain.Num() && i < RightBoneChain.Num())
						{
							verify(SimulateBones.IsValidIndex(CurBoneChain[i]));
							verify(SimulateBones.IsValidIndex(LeftBoneChain[i]));
							verify(SimulateBones.IsValidIndex(RightBoneChain[i]));

							const FLKAnimVerletConstraint_Straighten StraightenConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], StraightenBendedBoneStrength, true);
							StraightenConstraints.Emplace(StraightenConstraint);
						}
					}
				}

				if (BoneChainIndexes.IsValidIndex(LeftIndex))
				{
					const TArray<int32>& CurBoneChain = BoneChainIndexes[LeftCurIndex];
					const TArray<int32>& LeftBoneChain = BoneChainIndexes[LeftIndex];
					if (i < CurBoneChain.Num() && i < LeftBoneChain.Num())
					{
						verify(SimulateBones.IsValidIndex(CurBoneChain[i]));
						verify(SimulateBones.IsValidIndex(LeftBoneChain[i]));

						const FLKAnimVerletBoneIndicatorPair DistancePair(FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(LeftBoneChain[i], false));
						SimulateBonePairIndicators.Emplace(DistancePair);

						const FLKAnimVerletConstraint_Distance DistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
						DistanceConstraints.Emplace(DistanceConstraint);

						if (FMath::IsNearlyZero(SideStraightenForce, KINDA_SMALL_NUMBER) == false)
						{
							const FVector SideStraightenDir = (SimulateBones[LeftBoneChain[i]].PoseLocation - SimulateBones[CurBoneChain[i]].PoseLocation).GetSafeNormal();
							const FQuat InvRot = SimulateBones[LeftBoneChain[i]].PoseRotation.Inverse();
							const FVector LocalSideStraightenDir = InvRot.RotateVector(SideStraightenDir);
							SimulateBones[LeftBoneChain[i]].SetSideStraightenDirInLocal(LocalSideStraightenDir);
						}

						if (bPreserveSideLength)
						{
							const FLKAnimVerletConstraint_FixedDistance FixedDistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], bStretchEachBone, StretchStrength, false, SideLengthMargin);
							FixedDistanceConstraints.Emplace(FixedDistanceConstraint);
						}

						if (bConstrainRightDiagonalDistance)
						{
							if (i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletBoneIndicatorPair RightDiagonalDistancePair(FLKAnimVerletBoneIndicator(CurBoneChain[i + 1], false), FLKAnimVerletBoneIndicator(LeftBoneChain[i], false));
								SimulateBonePairIndicators.Emplace(RightDiagonalDistancePair);

								const FLKAnimVerletConstraint_Distance RightDiagonalConstraint(&SimulateBones[CurBoneChain[i + 1]], &SimulateBones[LeftBoneChain[i]], bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
								DistanceConstraints.Emplace(RightDiagonalConstraint);
							}
						}
						if (bConstrainLeftDiagonalDistance)
						{
							if (i + 1 < LeftBoneChain.Num())
							{
								const FLKAnimVerletBoneIndicatorPair LeftDiagonalDistancePair(FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(LeftBoneChain[i + 1], false));
								SimulateBonePairIndicators.Emplace(LeftDiagonalDistancePair);

								const FLKAnimVerletConstraint_Distance LeftDiagonalConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i + 1]], bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
								DistanceConstraints.Emplace(LeftDiagonalConstraint);
							}
						}

						if (bUseIsometricBendingConstraint)
						{
							if (i + 1 < LeftBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending BendingConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[LeftBoneChain[i + 1]], bUseXPBDSolver, (bUseXPBDSolver ? BendingCompliance : BendingStiffness));
								BendingConstraints.Emplace(BendingConstraint);
							}
						}
					}
					LeftCurIndex = LeftIndex;
					--LeftIndex;
				}

				if (BoneChainIndexes.IsValidIndex(RightIndex))
				{
					const TArray<int32>& CurBoneChain = BoneChainIndexes[RightCurIndex];
					const TArray<int32>& RightBoneChain = BoneChainIndexes[RightIndex];
					if (i < CurBoneChain.Num() && i < RightBoneChain.Num())
					{
						verify(SimulateBones.IsValidIndex(CurBoneChain[i]));
						verify(SimulateBones.IsValidIndex(RightBoneChain[i]));

						const FLKAnimVerletBoneIndicatorPair DistancePair(FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(RightBoneChain[i], false));
						SimulateBonePairIndicators.Emplace(DistancePair);

						const FLKAnimVerletConstraint_Distance DistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
						DistanceConstraints.Emplace(DistanceConstraint);

						if (FMath::IsNearlyZero(SideStraightenForce, KINDA_SMALL_NUMBER) == false)
						{
							const FVector SideStraightenDir = (SimulateBones[RightBoneChain[i]].PoseLocation - SimulateBones[CurBoneChain[i]].PoseLocation).GetSafeNormal();
							const FQuat InvRot = SimulateBones[RightBoneChain[i]].PoseRotation.Inverse();
							const FVector LocalSideStraightenDir = InvRot.RotateVector(SideStraightenDir);
							SimulateBones[RightBoneChain[i]].SetSideStraightenDirInLocal(LocalSideStraightenDir);
						}

						if (bPreserveSideLength)
						{
							const FLKAnimVerletConstraint_FixedDistance FixedDistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], bStretchEachBone, StretchStrength, false, SideLengthMargin);
							FixedDistanceConstraints.Emplace(FixedDistanceConstraint);
						}

						if (bConstrainRightDiagonalDistance)
						{
							if (i + 1 < RightBoneChain.Num())
							{
								const FLKAnimVerletBoneIndicatorPair RightDiagonalDistancePair(FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(RightBoneChain[i + 1], false));
								SimulateBonePairIndicators.Emplace(RightDiagonalDistancePair);

								const FLKAnimVerletConstraint_Distance RightDiagonalConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i + 1]], bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
								DistanceConstraints.Emplace(RightDiagonalConstraint);
							}
						}
						if (bConstrainLeftDiagonalDistance)
						{
							if (i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletBoneIndicatorPair LeftDiagonalDistancePair(FLKAnimVerletBoneIndicator(CurBoneChain[i + 1], false), FLKAnimVerletBoneIndicator(RightBoneChain[i], false));
								SimulateBonePairIndicators.Emplace(LeftDiagonalDistancePair);

								const FLKAnimVerletConstraint_Distance LeftDiagonalConstraint(&SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i]], bUseXPBDSolver, (bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness)), bStretchEachBone, StretchStrength);
								DistanceConstraints.Emplace(LeftDiagonalConstraint);
							}
						}

						if (bUseIsometricBendingConstraint)
						{
							if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending BendingConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i + 1]], bUseXPBDSolver, (bUseXPBDSolver ? BendingCompliance : BendingStiffness));
								BendingConstraints.Emplace(BendingConstraint);
							}
						}
					}
					RightCurIndex = RightIndex;
					++RightIndex;
				}
			}
		}
	}

	/// WorldCollision(Contact) constraints
	if (WorldCollisionProfile != NAME_None)
	{
		if (PoseContext.AnimInstanceProxy != nullptr)
		{
			USkeletalMeshComponent* SkeletalMeshComponent = PoseContext.AnimInstanceProxy->GetSkelMeshComponent();
			if (SkeletalMeshComponent != nullptr)
			{
				FLKAnimVerletCollisionConstraintInput CollisionConstraintInput;
				{
					CollisionConstraintInput.Bones = &SimulateBones;
					CollisionConstraintInput.bUseCapsuleCollisionForChain = bUseCapsuleCollisionForChain;
					CollisionConstraintInput.SimulateBonePairIndicators = &SimulateBonePairIndicators;
					MakeBroadphaseInput(OUT CollisionConstraintInput.BroadphaseInput);
					CollisionConstraintInput.bUseXPBDSolver = bUseXPBDSolver;
					CollisionConstraintInput.Compliance = Compliance;

					CollisionConstraintInput.ExcludeBones.Init(false, SimulateBones.Num());
					for (int32 i = 0; i < WorldCollisionExcludeBones.Num(); ++i)
					{
						WorldCollisionExcludeBones[i].Initialize(BoneContainer);
						const int32 FoundIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(WorldCollisionExcludeBones[i]));
						if (FoundIndex != INDEX_NONE)
							CollisionConstraintInput.ExcludeBones[FoundIndex] = true;
					}
				}

				const UWorld* World = SkeletalMeshComponent->GetWorld();

				const FLKAnimVerletConstraint_World WorldCollisionConstraint(World, SkeletalMeshComponent, WorldCollisionProfile, CollisionConstraintInput);
				WorldCollisionConstraints.Emplace(WorldCollisionConstraint);
			}
		}
	}

	/// LocalCollision(Contact) constraints
	InitializeLocalCollisionConstraints(BoneContainer);

	const float BoundExtentMax = BoneBound.GetHalfExtents().GetMax();
	BoneBound.Expand(BoundExtentMax);
	BroadphaseContainer.Initialize(BoneBound.GetCenter(), BoneBound.GetHalfExtents(), BoneMaxBound.GetHalfExtents() * 30.0f);
}

void FLKAnimNode_AnimVerlet::InitializeLocalCollisionConstraints(const FBoneContainer& BoneContainer)
{
	SimulatingCollisionShapes.SphereCollisionShapes = SphereCollisionShapes;
	SimulatingCollisionShapes.CapsuleCollisionShapes = CapsuleCollisionShapes;
	SimulatingCollisionShapes.BoxCollisionShapes = BoxCollisionShapes;
	SimulatingCollisionShapes.PlaneCollisionShapes = PlaneCollisionShapes;
	if (CollisionDataAsset != nullptr)
		CollisionDataAsset->ConvertToShape(OUT SimulatingCollisionShapes);

	for (FLKAnimVerletCollisionSphere& CurShape : SimulatingCollisionShapes.SphereCollisionShapes)
	{
		InitializeAttachedShape(CurShape, BoneContainer);
	}
	for (FLKAnimVerletCollisionCapsule& CurShape : SimulatingCollisionShapes.CapsuleCollisionShapes)
	{
		InitializeAttachedShape(CurShape, BoneContainer);
	}
	for (FLKAnimVerletCollisionBox& CurShape : SimulatingCollisionShapes.BoxCollisionShapes)
	{
		InitializeAttachedShape(CurShape, BoneContainer);
	}
	for (FLKAnimVerletCollisionPlane& CurShape : SimulatingCollisionShapes.PlaneCollisionShapes)
	{
		InitializeAttachedShape(CurShape, BoneContainer);
	}
}

void FLKAnimNode_AnimVerlet::InitializeAttachedShape(FLKAnimVerletCollisionShape& InShape, const FBoneContainer& BoneContainer)
{
	if (InShape.bUseAbsoluteWorldTransform == false)
		InShape.AttachedBone.Initialize(BoneContainer);

	InShape.ExcludeBoneBits.Init(false, SimulateBones.Num());
	for (int32 i = 0; i < InShape.ExcludeBones.Num(); ++i)
	{
		InShape.ExcludeBones[i].Initialize(BoneContainer);
		const int32 FoundIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(InShape.ExcludeBones[i]));
		if (FoundIndex != INDEX_NONE)
			InShape.ExcludeBoneBits[FoundIndex] = true;
	}
}

bool FLKAnimNode_AnimVerlet::MakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton, int32 BoneIndex, 
											   int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting, bool bParentExcluded, int32 ParentExcludedBoneIndex)
{
	verify(BoneIndex >= 0 && BoneIndex < ReferenceSkeleton.GetNum());

	FBoneReference CurBoneRef;
	CurBoneRef.BoneName = ReferenceSkeleton.GetBoneName(BoneIndex);
	
	int32 CurSimulateBoneIndex = ParentSimulateBoneIndex;
	int32 CurExcludedBoneIndex = ParentExcludedBoneIndex;
	bool bNewlyExcluded = false;
	const bool bExcludedBone = (BoneSetting.ExcludeBones.Find(CurBoneRef) != INDEX_NONE);
	const FLKAnimVerletBoneUnitSetting* FoundBoneUnitSettingNullable = BoneSetting.BoneUnitSettingOverride.FindByKey(CurBoneRef);
	if (bExcludedBone == false)
	{
		FLKAnimVerletBone NewSimulateBone;
		NewSimulateBone.BoneReference = CurBoneRef;
		NewSimulateBone.ParentVerletBoneIndex = ParentSimulateBoneIndex;
		NewSimulateBone.BoneReference.Initialize(BoneContainer);
		if (NewSimulateBone.BoneReference.CachedCompactPoseIndex == INDEX_NONE)
			return false;

		FTransform ReferenceBonePoseT = PoseContext.Pose.GetComponentSpaceTransform(NewSimulateBone.BoneReference.CachedCompactPoseIndex);
		NewSimulateBone.bFakeBone = BoneSetting.bFakeBone;
		NewSimulateBone.bUseXPBDSolver = bUseXPBDSolver;
		NewSimulateBone.InvMass = FMath::IsNearlyZero(BoneSetting.Mass, KINDA_SMALL_NUMBER) ? 1.0f : 1.0f / BoneSetting.Mass;
		NewSimulateBone.Thickness = Thickness;
		if (FoundBoneUnitSettingNullable != nullptr)
		{
			if (FoundBoneUnitSettingNullable->bOverrideConstrainConeAngleFromParent)
				NewSimulateBone.bConstrainConeAngleFromParent = FoundBoneUnitSettingNullable->bConstrainConeAngleFromParent;
			if (FoundBoneUnitSettingNullable->bOverrideConeAngle)
				NewSimulateBone.ConeAngleConstraint = FoundBoneUnitSettingNullable->ConeAngle;
			if (FoundBoneUnitSettingNullable->bOverrideMass)
				NewSimulateBone.InvMass = FMath::IsNearlyZero(FoundBoneUnitSettingNullable->Mass, KINDA_SMALL_NUMBER) ? 1.0f : 1.0f / FoundBoneUnitSettingNullable->Mass;
			if (FoundBoneUnitSettingNullable->bOverrideThickness)
				NewSimulateBone.Thickness = FoundBoneUnitSettingNullable->Thickness;
			NewSimulateBone.bOverrideToUseSphereCollisionForChain = FoundBoneUnitSettingNullable->bOverrideToUseSphereCollisionForChain;

			if (FoundBoneUnitSettingNullable->bLockBone)
			{
				NewSimulateBone.bPinned = true;	///Reserve
				NewSimulateBone.PinMargin = FoundBoneUnitSettingNullable->LockMargin;
			}
		}

		if (NewSimulateBone.bFakeBone)
		{
			NewSimulateBone.SetFakeBoneOffset(BoneSetting.FakeBoneOffsetDir.GetSafeNormal() * BoneSetting.FakeBoneOffsetSize);
			ReferenceBonePoseT = NewSimulateBone.MakeFakeBonePoseTransform(ReferenceBonePoseT);
		}
		NewSimulateBone.InitializeTransform(ReferenceBonePoseT);

		/// Make subdivided simulate bones
		if (bSubDivideBones && NumSubDividedBone > 0 && ParentSimulateBoneIndex != INDEX_NONE)
		{
			int32 SubDividedParentSimulateBoneIndex = ParentSimulateBoneIndex;
			const FLKAnimVerletBone& ParentSimulateBone = SimulateBones[ParentSimulateBoneIndex];
			FVector DirToParent = FVector::ZeroVector;
			float DirToParentSize = 0.0f;
			(ParentSimulateBone.PoseLocation - NewSimulateBone.PoseLocation).ToDirectionAndLength(OUT DirToParent, OUT DirToParentSize);
			const float SubDivideSize = DirToParentSize / static_cast<float>(NumSubDividedBone + 1);

			for (uint8 SubDivideCount = NumSubDividedBone; SubDivideCount > 0; --SubDivideCount)
			{
				FLKAnimVerletBone SubDividedSimulateBone = NewSimulateBone;
				SubDividedSimulateBone.ParentVerletBoneIndex = SubDividedParentSimulateBoneIndex;
				SubDividedSimulateBone.bFakeBone = true;
				SubDividedSimulateBone.SetFakeBoneOffset(DirToParent * SubDivideSize * SubDivideCount);
				const FTransform SubDividedBonePoseT = SubDividedSimulateBone.MakeFakeBonePoseTransform(ReferenceBonePoseT);
				SubDividedSimulateBone.InitializeTransform(SubDividedBonePoseT);
				const int32 SubDividedSimulateBoneIndex = SimulateBones.Emplace(SubDividedSimulateBone);
				if (SubDividedSimulateBone.ParentVerletBoneIndex != INDEX_NONE)
				{
					/// Add child indexes
					FLKAnimVerletBone& CurParentVerletBone = SimulateBones[SubDividedSimulateBone.ParentVerletBoneIndex];
					bool bAlreadyInSet = false;
					CurParentVerletBone.ChildVerletBoneIndexes.Add(SubDividedSimulateBoneIndex, &bAlreadyInSet);
				}

				const int32 CurAddedBoneChainLength = BoneChainIndexes[RootSimulateBoneIndex].Add(SubDividedSimulateBoneIndex) + 1;
				MaxBoneChainLength = FMath::Max(MaxBoneChainLength, CurAddedBoneChainLength);

				SubDividedParentSimulateBoneIndex = SubDividedSimulateBoneIndex;
			}
			NewSimulateBone.ParentVerletBoneIndex = SubDividedParentSimulateBoneIndex;
		}

		CurSimulateBoneIndex = SimulateBones.Emplace(NewSimulateBone);
		if (BoneSetting.bFakeBone == false)
			RelevantBoneIndicators.Emplace(CurSimulateBoneIndex, false, bParentExcluded ? ParentExcludedBoneIndex : ParentSimulateBoneIndex, bParentExcluded);
		if (NewSimulateBone.ParentVerletBoneIndex != INDEX_NONE)
		{
			/// Add child indexes
			FLKAnimVerletBone& CurParentVerletBone = SimulateBones[NewSimulateBone.ParentVerletBoneIndex];
			bool bAlreadyInSet = false;
			CurParentVerletBone.ChildVerletBoneIndexes.Add(CurSimulateBoneIndex, &bAlreadyInSet);
		}

		if (ParentSimulateBoneIndex == INDEX_NONE)
		{
			RootSimulateBoneIndex = BoneChainIndexes.Emplace();
		}
		else
		{
			verify(RootSimulateBoneIndex != INDEX_NONE);
		}
		const int32 CurAddedBoneChainLength = BoneChainIndexes[RootSimulateBoneIndex].Add(CurSimulateBoneIndex) + 1;
		MaxBoneChainLength = FMath::Max(MaxBoneChainLength, CurAddedBoneChainLength);
	}
	else
	{
		FLKAnimVerletExcludedBone NewExcludedBone(CurBoneRef, ParentSimulateBoneIndex, bParentExcluded ? ParentExcludedBoneIndex : INDEX_NONE);
		NewExcludedBone.BoneReference.Initialize(BoneContainer);
		if (NewExcludedBone.BoneReference.CachedCompactPoseIndex != INDEX_NONE)
		{
			const FTransform ReferenceBonePoseT = PoseContext.Pose.GetComponentSpaceTransform(NewExcludedBone.BoneReference.CachedCompactPoseIndex);
			NewExcludedBone.PrepareSimulation(ReferenceBonePoseT);

			if (bParentExcluded == false && ParentSimulateBoneIndex != INDEX_NONE)
				NewExcludedBone.LengthToParent = (SimulateBones[ParentSimulateBoneIndex].PoseLocation - ReferenceBonePoseT.GetLocation()).Size();
			else if (bParentExcluded && ParentExcludedBoneIndex != INDEX_NONE)
				NewExcludedBone.LengthToParent = (ExcludedBones[ParentExcludedBoneIndex].PoseLocation - ReferenceBonePoseT.GetLocation()).Size();
			NewExcludedBone.bStraightenExcludedBonesByParent = BoneSetting.bStraightenExcludedBonesByParent;

			CurExcludedBoneIndex = ExcludedBones.Emplace(NewExcludedBone);
			if (BoneSetting.bFakeBone == false)
				RelevantBoneIndicators.Emplace(CurExcludedBoneIndex, true, bParentExcluded ? ParentExcludedBoneIndex : ParentSimulateBoneIndex, bParentExcluded);
			bNewlyExcluded = true;
		}
	}
	const bool bTipBone = (WalkChildsAndMakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, BoneIndex, CurSimulateBoneIndex, RootSimulateBoneIndex, BoneSetting, bNewlyExcluded, CurExcludedBoneIndex) == false);

	if (bTipBone)
	{
		if (bLockTipBone == false && bMakeFakeTipBone && FakeTipBoneLength > 0.0f)
		{
			FLKAnimVerletBone FakeSimulateBone;
			FakeSimulateBone.bFakeBone = true;
			FakeSimulateBone.bTipBone = true;
			FakeSimulateBone.bUseXPBDSolver = bUseXPBDSolver;
			FakeSimulateBone.InvMass = FMath::IsNearlyZero(BoneSetting.Mass, KINDA_SMALL_NUMBER) ? 1.0f : 1.0f / BoneSetting.Mass;
			FakeSimulateBone.Thickness = Thickness;
			if (FoundBoneUnitSettingNullable != nullptr)
			{
				if (FoundBoneUnitSettingNullable->bOverrideConstrainConeAngleFromParent)
					FakeSimulateBone.bConstrainConeAngleFromParent = FoundBoneUnitSettingNullable->bConstrainConeAngleFromParent;
				if (FoundBoneUnitSettingNullable->bOverrideConeAngle)
					FakeSimulateBone.ConeAngleConstraint = FoundBoneUnitSettingNullable->ConeAngle;
				if (FoundBoneUnitSettingNullable->bOverrideMass)
					FakeSimulateBone.InvMass = FMath::IsNearlyZero(FoundBoneUnitSettingNullable->Mass, KINDA_SMALL_NUMBER) ? 1.0f : 1.0f / FoundBoneUnitSettingNullable->Mass;
				if (FoundBoneUnitSettingNullable->bOverrideThickness)
					FakeSimulateBone.Thickness = FoundBoneUnitSettingNullable->Thickness;
				FakeSimulateBone.bOverrideToUseSphereCollisionForChain = FoundBoneUnitSettingNullable->bOverrideToUseSphereCollisionForChain;

				if (FoundBoneUnitSettingNullable->bLockBone)
				{
					FakeSimulateBone.bPinned = true;	///Reserve
					FakeSimulateBone.PinMargin = FoundBoneUnitSettingNullable->LockMargin;
				}
			}
			FakeSimulateBone.ParentVerletBoneIndex = CurSimulateBoneIndex;

			FTransform FakeBoneT = FTransform::Identity;
			MakeFakeBoneTransform(OUT FakeBoneT, CurSimulateBoneIndex);
			FakeSimulateBone.InitializeTransform(FakeBoneT);

			const int32 FakeBoneIndex = SimulateBones.Emplace(FakeSimulateBone);
			if (BoneSetting.bFakeBone == false)
				RelevantBoneIndicators.Emplace(FakeBoneIndex, false, bNewlyExcluded ? CurExcludedBoneIndex : CurSimulateBoneIndex, bNewlyExcluded);
			if (FakeSimulateBone.ParentVerletBoneIndex != INDEX_NONE)
			{
				/// Add child indexes
				FLKAnimVerletBone& CurParentVerletBone = SimulateBones[FakeSimulateBone.ParentVerletBoneIndex];
				bool bAlreadyInSet = false;
				CurParentVerletBone.ChildVerletBoneIndexes.Add(FakeBoneIndex, &bAlreadyInSet);
			}

			const int32 AddedFakeBoneChainLength = BoneChainIndexes[RootSimulateBoneIndex].Add(FakeBoneIndex) + 1;
			MaxBoneChainLength = FMath::Max(MaxBoneChainLength, AddedFakeBoneChainLength);
		}
		else
		{
			SimulateBones[CurSimulateBoneIndex].bTipBone = true;
		}
	}
	return true;
}

bool FLKAnimNode_AnimVerlet::WalkChildsAndMakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton, int32 BoneIndex, 
															int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting, bool bParentExcluded, int32 ParentExcludedBoneIndex)
{
	bool bWalked = false;
	const int32 NumBones = ReferenceSkeleton.GetNum();
	for (int32 ChildIndex = BoneIndex + 1; ChildIndex < NumBones; ++ChildIndex)
	{
		if (BoneIndex == ReferenceSkeleton.GetParentIndex(ChildIndex))
		{
			MakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, ChildIndex, ParentSimulateBoneIndex, RootSimulateBoneIndex, BoneSetting, bParentExcluded, ParentExcludedBoneIndex);
			bWalked = true;
		}
	}
	return bWalked;
}

void FLKAnimNode_AnimVerlet::MakeFakeBoneTransform(OUT FTransform& OutTransform, int32 ParentSimulateBoneIndex) const
{
	verify(SimulateBones.IsValidIndex(ParentSimulateBoneIndex));
	const FLKAnimVerletBone& ParentBone = SimulateBones[ParentSimulateBoneIndex];

	const FVector DirToParent = ParentBone.HasParentBone() ? (SimulateBones[ParentBone.ParentVerletBoneIndex].PoseLocation - ParentBone.PoseLocation).GetSafeNormal() : ParentBone.PoseRotation.GetUpVector();
	OutTransform = FTransform(ParentBone.Rotation, ParentBone.PoseLocation - DirToParent * FakeTipBoneLength);
}

void FLKAnimNode_AnimVerlet::UpdateDeltaTime(float InDeltaTime, float InTimeDilation)
{
	DeltaTime = FMath::IsNearlyZero(FixedDeltaTime, KINDA_SMALL_NUMBER) ? FMath::Clamp(InDeltaTime, MinDeltaTime, MaxDeltaTime) : FMath::Clamp(FixedDeltaTime * InTimeDilation, MinDeltaTime, MaxDeltaTime);
}

void FLKAnimNode_AnimVerlet::PrepareSimulation(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FTransform& ComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PrepareSimulation);
#endif

	/// Just make sure
	if (DeltaTime <= 0.0f)
		UpdateDeltaTime(KINDA_SMALL_NUMBER, 1.0f);

	for (int32 SimulateBoneIndex = 0; SimulateBoneIndex < SimulateBones.Num(); ++SimulateBoneIndex)
	{
		FLKAnimVerletBone& CurSimulateBone = SimulateBones[SimulateBoneIndex];
		FTransform CurBonePoseT = FTransform::Identity;
		if (CurSimulateBone.bFakeBone && CurSimulateBone.HasBoneSetup() == false)
		{
			/// Virtual TipBone case
			MakeFakeBoneTransform(OUT CurBonePoseT, CurSimulateBone.ParentVerletBoneIndex);
		}
		else
		{
			const FCompactPoseBoneIndex PoseBoneIndex = CurSimulateBone.BoneReference.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			CurBonePoseT = PoseBoneIndex != INDEX_NONE ? PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex) : FTransform(CurSimulateBone.Rotation, CurSimulateBone.Location, CurSimulateBone.PoseScale);
			
			/// Virtual BoneChain case
			if (CurSimulateBone.bFakeBone)
				CurBonePoseT.SetLocation(CurSimulateBone.MakeFakeBonePoseLocation(CurBonePoseT));
		}

		const FVector PoseDirFromParent = CurSimulateBone.HasParentBone() ? (CurBonePoseT.GetLocation() - SimulateBones[CurSimulateBone.ParentVerletBoneIndex].PoseLocation).GetSafeNormal() : FVector::ZeroVector;
		CurSimulateBone.PrepareSimulation(CurBonePoseT, PoseDirFromParent);
	}

	for (int32 ExcludedBoneIndex = 0; ExcludedBoneIndex < ExcludedBones.Num(); ++ExcludedBoneIndex)
	{
		FLKAnimVerletExcludedBone& CurExcludedBone = ExcludedBones[ExcludedBoneIndex];
		const FCompactPoseBoneIndex PoseBoneIndex = CurExcludedBone.BoneReference.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		const FTransform CurBonePoseT = PoseBoneIndex != INDEX_NONE ? PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex) : FTransform::Identity;
		CurExcludedBone.PrepareSimulation(CurBonePoseT);
	}

	PrepareLocalCollisionConstraints(PoseContext, BoneContainer, ComponentTransform);
}

void FLKAnimNode_AnimVerlet::PrepareLocalCollisionConstraints(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FTransform& ComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PrepareLocalCollisionConstraints);
#endif

	const double Compliance = static_cast<double>(1.0 / InvCompliance);

	if (bLocalColliderDirty)
	{
		SimulatingCollisionShapes.ResetCollisionShapeList();
		InitializeLocalCollisionConstraints(BoneContainer);
	}

	FLKAnimVerletCollisionConstraintInput CollisionConstraintInput;
	{
		CollisionConstraintInput.Bones = &SimulateBones;
		CollisionConstraintInput.bUseCapsuleCollisionForChain = bUseCapsuleCollisionForChain;
		CollisionConstraintInput.SimulateBonePairIndicators = &SimulateBonePairIndicators;
		MakeBroadphaseInput(OUT CollisionConstraintInput.BroadphaseInput);
		CollisionConstraintInput.bUseXPBDSolver = bUseXPBDSolver;
		CollisionConstraintInput.Compliance = Compliance;
	}

	///----------------------------------------------------------------------------------------------------------------------------
	/// Sphere
	///----------------------------------------------------------------------------------------------------------------------------
	SphereCollisionConstraints.Reset();
	for (int32 i = 0; i < SimulatingCollisionShapes.SphereCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionSphere& CurShapeSphere = SimulatingCollisionShapes.SphereCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapeSphere.ExcludeBoneBits;

		if (CurShapeSphere.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapeSphere.LocationOffset);
			const FLKAnimVerletConstraint_Sphere SphereConstraint(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
			SphereCollisionConstraints.Emplace(SphereConstraint);
		}
		else
		{
			const FCompactPoseBoneIndex PoseBoneIndex = CurShapeSphere.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(FQuat::Identity, CurShapeSphere.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FLKAnimVerletConstraint_Sphere SphereConstraint(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
				SphereCollisionConstraints.Emplace(SphereConstraint);
			}
		}
	}
	for (int32 i = 0; i < DynamicCollisionShapes.SphereCollisionShapes.Num(); ++i)
	{
		FLKAnimVerletCollisionSphere& CurShapeSphere = DynamicCollisionShapes.SphereCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapeSphere.ExcludeBoneBits;

		if (CurShapeSphere.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapeSphere.LocationOffset);
			const FLKAnimVerletConstraint_Sphere SphereConstraint(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
			SphereCollisionConstraints.Emplace(SphereConstraint);
		}
		else
		{
			if (CurShapeSphere.AttachedBone.BoneName != NAME_None && CurShapeSphere.AttachedBone.HasValidSetup() == false)
			{
				InitializeAttachedShape(CurShapeSphere, BoneContainer);
			}

			const FCompactPoseBoneIndex PoseBoneIndex = CurShapeSphere.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(FQuat::Identity, CurShapeSphere.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FLKAnimVerletConstraint_Sphere SphereConstraint(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
				SphereCollisionConstraints.Emplace(SphereConstraint);
			}
		}
	}

	///----------------------------------------------------------------------------------------------------------------------------
	/// Capsule
	///----------------------------------------------------------------------------------------------------------------------------
	CapsuleCollisionConstraints.Reset();
	for (int32 i = 0; i < SimulatingCollisionShapes.CapsuleCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionCapsule& CurShapeCapsule = SimulatingCollisionShapes.CapsuleCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapeCapsule.ExcludeBoneBits;

		if (CurShapeCapsule.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapeCapsule.LocationOffset);
			const FQuat BoneRotation = ComponentTransform.InverseTransformRotation(CurShapeCapsule.RotationOffset.Quaternion());
			const FLKAnimVerletConstraint_Capsule CapsuleConstraint(BoneLocation, BoneRotation, CurShapeCapsule.Radius, 
																	CurShapeCapsule.HalfHeight, CollisionConstraintInput);
			CapsuleCollisionConstraints.Emplace(CapsuleConstraint);
		}
		else
		{
			const FCompactPoseBoneIndex PoseBoneIndex = CurShapeCapsule.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(CurShapeCapsule.RotationOffset.Quaternion(), CurShapeCapsule.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FQuat BoneRotation = BoneTInCS.GetRotation();
				const FLKAnimVerletConstraint_Capsule CapsuleConstraint(BoneLocation, BoneRotation, CurShapeCapsule.Radius, 
																		CurShapeCapsule.HalfHeight, CollisionConstraintInput);
				CapsuleCollisionConstraints.Emplace(CapsuleConstraint);
			}
		}
	}
	for (int32 i = 0; i < DynamicCollisionShapes.CapsuleCollisionShapes.Num(); ++i)
	{
		FLKAnimVerletCollisionCapsule& CurShapeCapsule = DynamicCollisionShapes.CapsuleCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapeCapsule.ExcludeBoneBits;

		if (CurShapeCapsule.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapeCapsule.LocationOffset);
			const FQuat BoneRotation = ComponentTransform.InverseTransformRotation(CurShapeCapsule.RotationOffset.Quaternion());
			const FLKAnimVerletConstraint_Capsule CapsuleConstraint(BoneLocation, BoneRotation, CurShapeCapsule.Radius, 
																	CurShapeCapsule.HalfHeight, CollisionConstraintInput);
			CapsuleCollisionConstraints.Emplace(CapsuleConstraint);
		}
		else
		{
			if (CurShapeCapsule.AttachedBone.BoneName != NAME_None && CurShapeCapsule.AttachedBone.HasValidSetup() == false)
			{
				InitializeAttachedShape(CurShapeCapsule, BoneContainer);
			}

			const FCompactPoseBoneIndex PoseBoneIndex = CurShapeCapsule.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(CurShapeCapsule.RotationOffset.Quaternion(), CurShapeCapsule.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FQuat BoneRotation = BoneTInCS.GetRotation();
				const FLKAnimVerletConstraint_Capsule CapsuleConstraint(BoneLocation, BoneRotation, CurShapeCapsule.Radius, 
																		CurShapeCapsule.HalfHeight, CollisionConstraintInput);
				CapsuleCollisionConstraints.Emplace(CapsuleConstraint);
			}
		}
	}

	///----------------------------------------------------------------------------------------------------------------------------
	/// Box
	///----------------------------------------------------------------------------------------------------------------------------
	BoxCollisionConstraints.Reset();
	for (int32 i = 0; i < SimulatingCollisionShapes.BoxCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionBox& CurShapeBox = SimulatingCollisionShapes.BoxCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapeBox.ExcludeBoneBits;

		if (CurShapeBox.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapeBox.LocationOffset);
			const FQuat BoneRotation = ComponentTransform.InverseTransformRotation(CurShapeBox.RotationOffset.Quaternion());

			const FLKAnimVerletConstraint_Box BoxConstraint(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
			BoxCollisionConstraints.Emplace(BoxConstraint);
		}
		else
		{
			const FCompactPoseBoneIndex PoseBoneIndex = CurShapeBox.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(CurShapeBox.RotationOffset.Quaternion(), CurShapeBox.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FQuat BoneRotation = BoneTInCS.GetRotation();

				const FLKAnimVerletConstraint_Box BoxConstraint(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
				BoxCollisionConstraints.Emplace(BoxConstraint);
			}
		}
	}
	for (int32 i = 0; i < DynamicCollisionShapes.BoxCollisionShapes.Num(); ++i)
	{
		FLKAnimVerletCollisionBox& CurShapeBox = DynamicCollisionShapes.BoxCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapeBox.ExcludeBoneBits;

		if (CurShapeBox.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapeBox.LocationOffset);
			const FQuat BoneRotation = ComponentTransform.InverseTransformRotation(CurShapeBox.RotationOffset.Quaternion());

			const FLKAnimVerletConstraint_Box BoxConstraint(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
			BoxCollisionConstraints.Emplace(BoxConstraint);
		}
		else
		{
			if (CurShapeBox.AttachedBone.BoneName != NAME_None && CurShapeBox.AttachedBone.HasValidSetup() == false)
			{
				InitializeAttachedShape(CurShapeBox, BoneContainer);
			}

			const FCompactPoseBoneIndex PoseBoneIndex = CurShapeBox.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(CurShapeBox.RotationOffset.Quaternion(), CurShapeBox.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FQuat BoneRotation = BoneTInCS.GetRotation();

				const FLKAnimVerletConstraint_Box BoxConstraint(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
				BoxCollisionConstraints.Emplace(BoxConstraint);
			}
		}
	}

	///----------------------------------------------------------------------------------------------------------------------------
	/// Plane
	///----------------------------------------------------------------------------------------------------------------------------
	PlaneCollisionConstraints.Reset();
	for (int32 i = 0; i < SimulatingCollisionShapes.PlaneCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionPlane& CurShapePlane = SimulatingCollisionShapes.PlaneCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapePlane.ExcludeBoneBits;

		if (CurShapePlane.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapePlane.LocationOffset);
			const FQuat BoneRotation = ComponentTransform.InverseTransformRotation(CurShapePlane.RotationOffset.Quaternion());
			const FLKAnimVerletConstraint_Plane PlaneConstraint(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
																CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
																CollisionConstraintInput);
			PlaneCollisionConstraints.Emplace(PlaneConstraint);
		}
		else
		{
			const FCompactPoseBoneIndex PoseBoneIndex = CurShapePlane.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(CurShapePlane.RotationOffset.Quaternion(), CurShapePlane.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FQuat BoneRotation = BoneTInCS.GetRotation();

				const FLKAnimVerletConstraint_Plane PlaneConstraint(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
																	CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
																	CollisionConstraintInput);
				PlaneCollisionConstraints.Emplace(PlaneConstraint);
			}
		}
	}
	for (int32 i = 0; i < DynamicCollisionShapes.PlaneCollisionShapes.Num(); ++i)
	{
		FLKAnimVerletCollisionPlane& CurShapePlane = DynamicCollisionShapes.PlaneCollisionShapes[i];
		CollisionConstraintInput.ExcludeBones = CurShapePlane.ExcludeBoneBits;

		if (CurShapePlane.bUseAbsoluteWorldTransform)
		{
			const FVector BoneLocation = ComponentTransform.InverseTransformPosition(CurShapePlane.LocationOffset);
			const FQuat BoneRotation = ComponentTransform.InverseTransformRotation(CurShapePlane.RotationOffset.Quaternion());
			const FLKAnimVerletConstraint_Plane PlaneConstraint(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
																CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
																CollisionConstraintInput);
			PlaneCollisionConstraints.Emplace(PlaneConstraint);
		}
		else
		{
			if (CurShapePlane.AttachedBone.BoneName != NAME_None && CurShapePlane.AttachedBone.HasValidSetup() == false)
			{
				InitializeAttachedShape(CurShapePlane, BoneContainer);
			}

			const FCompactPoseBoneIndex PoseBoneIndex = CurShapePlane.AttachedBone.GetCompactPoseIndex(BoneContainer);
			/// LOD case?
			if (PoseBoneIndex != INDEX_NONE)
			{
				FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
				const FTransform OffsetT(CurShapePlane.RotationOffset.Quaternion(), CurShapePlane.LocationOffset);

				BoneTInCS = OffsetT * BoneTInCS;
				const FVector BoneLocation = BoneTInCS.GetLocation();
				const FQuat BoneRotation = BoneTInCS.GetRotation();
				const FLKAnimVerletConstraint_Plane PlaneConstraint(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
																	CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
																	CollisionConstraintInput);
				PlaneCollisionConstraints.Emplace(PlaneConstraint);
			}
		}
	}
}

void FLKAnimNode_AnimVerlet::SimulateVerlet(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SimulateVerlet);
#endif

	verify(World != nullptr);

	PreUpdateBones(World, InDeltaTime, ComponentTransform, PrevComponentTransform);

	/// Solve
	SolveConstraints(InDeltaTime);
	
	if (bUseSleep)
		UpdateSleep(InDeltaTime);

	PostUpdateBones(InDeltaTime);
}

void FLKAnimNode_AnimVerlet::PreUpdateBones(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PreUpdateBones);
#endif

	const bool bUseRandomWind = (RandomWindDirection.IsNearlyZero(KINDA_SMALL_NUMBER) == false);
	const bool bUseWindComponentInWorld = (bAdjustWindComponent && World->Scene != nullptr);
	FLKAnimVerletUpdateParam VerletUpdateParam;
	{
		/// Clamp Move Intertia
		{
			const FVector PrevComponentLocation = PrevComponentTransform.GetLocation();
			VerletUpdateParam.ComponentMoveDiff = ComponentTransform.InverseTransformPosition(PrevComponentLocation);
			FVector MoveDiffDir = FVector::ZeroVector;
			float MoveDiffDist = 0.0f;
			VerletUpdateParam.ComponentMoveDiff.ToDirectionAndLength(OUT MoveDiffDir, OUT MoveDiffDist);

			MoveDiffDist = bClampMoveInertia ? FMath::Clamp(MoveDiffDist, 0.0f, MoveInertiaClampMaxDistance) : MoveDiffDist;
			VerletUpdateParam.ComponentMoveDiff = MoveDiffDir * MoveDiffDist * MoveInertiaScale;
		}

		/// Clamp Rotation Intertia
		{
			const FQuat PrevComponentRotation = PrevComponentTransform.GetRotation();
			VerletUpdateParam.ComponentRotDiff = ComponentTransform.InverseTransformRotation(PrevComponentRotation);
			FVector RotDiffAxis = FVector::ZeroVector;
			float RotDiffAngle = 0.0f;
			VerletUpdateParam.ComponentRotDiff.ToAxisAndAngle(OUT RotDiffAxis, OUT RotDiffAngle);

			float DiffAngeDegrees = FMath::RadiansToDegrees(RotDiffAngle);
			if (bClampRotationInertia && FMath::Abs(DiffAngeDegrees) > RotationInertiaClampDegrees)
				DiffAngeDegrees = FMath::Sign(DiffAngeDegrees) * RotationInertiaClampDegrees;
			VerletUpdateParam.ComponentRotDiff = FQuat(RotDiffAxis, FMath::DegreesToRadians(DiffAngeDegrees * RotationInertiaScale));
		}

		VerletUpdateParam.bUseSquaredDeltaTime = bUseSquaredDeltaTime;
		VerletUpdateParam.StretchForce = StretchForce;
		VerletUpdateParam.SideStraightenForce = SideStraightenForce;
		VerletUpdateParam.ShapeMemoryForce = ShapeMemoryForce;
		VerletUpdateParam.Gravity = (bGravityInWorldSpace == false || Gravity.IsNearlyZero(KINDA_SMALL_NUMBER)) ? Gravity : ComponentTransform.InverseTransformVector(Gravity);
		VerletUpdateParam.ExternalForce = (bExternalForceInWorldSpace == false || ExternalForce.IsNearlyZero(KINDA_SMALL_NUMBER)) ? ExternalForce : ComponentTransform.InverseTransformVector(ExternalForce);
		VerletUpdateParam.RandomWindDir = (bRandomWindDirectionInWorldSpace == false && bUseRandomWind) ? RandomWindDirection : ComponentTransform.InverseTransformVector(RandomWindDirection);
		VerletUpdateParam.RandomWindSizeMin = RandomWindSizeMin;
		VerletUpdateParam.RandomWindSizeMax = RandomWindSizeMax;
		VerletUpdateParam.Damping = Damping;
	}

	/// Simulate each bones	
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		CurVerletBone.Update(InDeltaTime, VerletUpdateParam);

		/// UWindDirectionalSourceComponent
		if (bUseWindComponentInWorld)
		{
			/// From UE4 AnimDynamics
			float WindMinGust = 0.0f;
			float WindMaxGust = 0.0f;

			FVector WindDirection = FVector::ZeroVector;
			float WindSpeed = 0.0f;
			World->Scene->GetWindParameters_GameThread(ComponentTransform.TransformPosition(CurVerletBone.PoseLocation), WindDirection, WindSpeed, WindMinGust, WindMaxGust);
			WindDirection = ComponentTransform.Inverse().TransformVector(WindDirection);
			const FVector WindVelocity = WindDirection * WindSpeed * FMath::FRandRange(0.0f, 2.0f);
			CurVerletBone.Location += WindVelocity * InDeltaTime;
		}

		/// Adjust animation pose transform
		if (bIgnoreAnimationPose == false && CurVerletBone.HasParentBone())
		{
			FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurVerletBone.ParentVerletBoneIndex];
			const float AnimPoseDeltaInertiaScaled = AnimationPoseDeltaInertia * AnimationPoseDeltaInertiaScale;
			CurVerletBone.AdjustPoseTransform(InDeltaTime, ParentVerletBone.Location, ParentVerletBone.PoseLocation, AnimationPoseInertia, AnimPoseDeltaInertiaScaled, bClampAnimationPoseDeltaInertia, AnimationPoseDeltaInertiaClampMax);
		}
	}
}

void FLKAnimNode_AnimVerlet::SolveConstraints(float InDeltaTime)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints);
#endif

	/// Solve Constraints
	const float SubStepDeltaTime = FMath::Max(bUseXPBDSolver ? InDeltaTime / SolveIteration : InDeltaTime, KINDA_SMALL_NUMBER);
	for (int32 Iteration = 0; Iteration < SolveIteration; ++Iteration)
	{
		const bool bFinalizeUpdate = (Iteration == SolveIteration - 1);

		/// Simulate each constraints
		/// Solve order is important. Make a heuristic order by constraint priority.
		/*for (int32 i = 0; i < Constraints.Num(); ++i)
		{
			verify(Constraints[i] != nullptr);
			Constraints[i]->Update(SubStepDeltaTime, bFinalizeUpdate);
		}*/
		for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].Update(SubStepDeltaTime, false);
		}
		for (int32 i = 0; i < DistanceConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_DistanceConstraints);
		#endif
			DistanceConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BendingConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BendingConstraints);
		#endif
			BendingConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < StraightenConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_StraightenConstraints);
		#endif
			StraightenConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BallSocketConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BallSocketConstraints);
		#endif
			BallSocketConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}

		///-------------------------------------------------------------------------------------
		/// Local collision constratins
		UpdateBroadphase();
		for (int32 i = 0; i < SphereCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_SphereCollisionConstraints);
		#endif
			SphereCollisionConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < CapsuleCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_CapsuleCollisionConstraints);
		#endif
			CapsuleCollisionConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BoxCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BoxCollisionConstraints);
		#endif
			BoxCollisionConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < PlaneCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PlaneCollisionConstraints);
		#endif
			PlaneCollisionConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		///-------------------------------------------------------------------------------------

		/*for (int32 i = 0; i < WorldCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_WorldCollisionConstraints);
		#endif
			WorldCollisionConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}*/

		/*for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}
		for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
		#endif
			FixedDistanceConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
		}*/

		/*if (bLockTipBone)
		{
			/// Backward Update
			for (int32 i = PinConstraints.Num() - 1; i >= 0; --i)
			{
			#if LK_ENABLE_STAT
				SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
			#endif
				PinConstraints[i].BackwardUpdate(SubStepDeltaTime, bFinalizeUpdate);
			}
			for (int32 i = FixedDistanceConstraints.Num() - 1; i >= 0; --i)
			{
			#if LK_ENABLE_STAT
				SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
			#endif
				FixedDistanceConstraints[i].BackwardUpdate(SubStepDeltaTime, bFinalizeUpdate);
			}

			for (int32 i = 0; i < PinConstraints.Num(); ++i)
			{
				#if LK_ENABLE_STAT
					SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
				#endif
				PinConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
			}
			for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
			{
			#if LK_ENABLE_STAT
				SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
			#endif
				FixedDistanceConstraints[i].Update(SubStepDeltaTime, bFinalizeUpdate);
			}
		}*/
	}

	/// Finalize special constraints
	for (int32 i = 0; i < WorldCollisionConstraints.Num(); ++i)
	{
	#if LK_ENABLE_STAT
		SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_WorldCollisionConstraints);
	#endif
		WorldCollisionConstraints[i].Update(InDeltaTime, true);
	}

	for (int32 i = 0; i < PinConstraints.Num(); ++i)
	{
	#if LK_ENABLE_STAT
		SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
	#endif
		PinConstraints[i].Update(InDeltaTime, true);
	}
	for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
	{
	#if LK_ENABLE_STAT
		SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
	#endif
		FixedDistanceConstraints[i].Update(InDeltaTime, true);
	}

	if (bLockTipBone)
	{
		/// Backward Update
		for (int32 i = PinConstraints.Num() - 1; i >= 0; --i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].BackwardUpdate(InDeltaTime, true);
		}
		for (int32 i = FixedDistanceConstraints.Num() - 1; i >= 0; --i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
		#endif
			FixedDistanceConstraints[i].BackwardUpdate(InDeltaTime, true);
		}

		for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].Update(InDeltaTime, true);
		}
		for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
		#endif
			FixedDistanceConstraints[i].Update(InDeltaTime, true);
		}
	}

	/// PostUpdate each bones
	ForEachConstraints([InDeltaTime](FLKAnimVerletConstraint& CurConstraint) {
		CurConstraint.PostUpdate(InDeltaTime);
	});
}

void FLKAnimNode_AnimVerlet::MakeBroadphaseInput(OUT FLKAnimVerletBroadphaseInput& OutBroadphaseInput)
{
	OutBroadphaseInput.ResetBroadphaseInput();
	
	OutBroadphaseInput.bUseBroadphase = bUseBroadphaseCollisionDetection;
	OutBroadphaseInput.BroadphaseContainer = &BroadphaseContainer;
	OutBroadphaseInput.BonePairIndicatorSpace = &BroadphaseSpace.BonePairIndicatorSpace;
	if (OutBroadphaseInput.bUseBroadphase)
	{
		OutBroadphaseInput.TargetBonePairIndicators = OutBroadphaseInput.BonePairIndicatorSpace;
	}
	else
	{
		OutBroadphaseInput.TargetBonePairIndicators = &SimulateBonePairIndicators;
	}
}

void FLKAnimNode_AnimVerlet::UpdateBroadphase()
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_UpdateBroadphase);
#endif

	if (bUseBroadphaseCollisionDetection == false)
		return;

	BroadphaseContainer.Reset();
	if (bUseCapsuleCollisionForChain)
	{
		for (int32 i = 0; i < SimulateBonePairIndicators.Num(); ++i)
		{
			const FLKAnimVerletBoneIndicatorPair& CurPair = SimulateBonePairIndicators[i];

			verify(CurPair.BoneB.IsValidBoneIndicator());
			verify(SimulateBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex));
			const FLKAnimVerletBone& CurVerletBone = SimulateBones[CurPair.BoneB.AnimVerletBoneIndex];

			if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
			{
				const FLKBroadphaseElement BroadphaseTreeElem(CurPair, CurVerletBone.MakeBound());
				BroadphaseContainer.AddElement(BroadphaseTreeElem);
			}
			else
			{
				verify(SimulateBones.IsValidIndex(CurPair.BoneA.AnimVerletBoneIndex));
				const FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurPair.BoneA.AnimVerletBoneIndex];

				const FLKAnimVerletBound ParentBound = FLKAnimVerletBone::MakeBound(ParentVerletBone.Location, CurVerletBone.Thickness);
				FLKAnimVerletBound CurBound = CurVerletBone.MakeBound();
				CurBound.Expand(ParentBound);

				const FLKBroadphaseElement BroadphaseTreeElem(CurPair, CurBound);
				BroadphaseContainer.AddElement(BroadphaseTreeElem);
			}
		}
	}
	else
	{
		for (int32 i = 0; i < SimulateBones.Num(); ++i)
		{
			const FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
			const FLKBroadphaseElement BroadphaseTreeElem(CurVerletBone, i, CurVerletBone.MakeBound());
			BroadphaseContainer.AddElement(BroadphaseTreeElem);
		}
	}
}

void FLKAnimNode_AnimVerlet::UpdateSleep(float InDeltaTime)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_UpdateSleep);
#endif

	const float SleepThresholdSQ = SleepDeltaThreshold * SleepDeltaThreshold;
	const float WakeUpThresholdSQ = WakeUpDeltaThreshold * WakeUpDeltaThreshold;
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];

		bool bForceWakeUp = false;
		if (bIgnoreSleepWhenParentWakedUp && CurVerletBone.HasParentBone())
		{
			const FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurVerletBone.ParentVerletBoneIndex];
			if (ParentVerletBone.IsSleep() == false)
				bForceWakeUp = true;
		}

		if (bForceWakeUp)
		{
			CurVerletBone.WakeUp();
		}
		else
		{
			const float CurDeltaSQ = (CurVerletBone.Location - CurVerletBone.PrevLocation).SizeSquared();
			if (CurVerletBone.IsSleep())
			{
				if (CurDeltaSQ >= WakeUpThresholdSQ)
				{
					CurVerletBone.WakeUp();
				}
				else
				{
					CurVerletBone.Sleep();
				}
			}
			else
			{
				if (CurDeltaSQ <= SleepThresholdSQ)
				{
					CurVerletBone.SleepTriggerElapsedTime += InDeltaTime;
					if (CurVerletBone.SleepTriggerElapsedTime >= SleepTriggerDuration)
					{
						CurVerletBone.Sleep();
					}
				}
				else
				{
					CurVerletBone.WakeUp();
				}
			}
		}
	}
}

void FLKAnimNode_AnimVerlet::PostUpdateBones(float InDeltaTime)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PostUpdateBones);
#endif

	/// PostUpdate each simulating bones
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		CurVerletBone.PostUpdate(InDeltaTime);
	}

	/// Calculate ExcludedBone`s Location(bStraightenExcludedBonesByParent)
	for (int32 i = 0; i < ExcludedBones.Num(); ++i)
	{
		FLKAnimVerletExcludedBone& CurExcludedVerletBone = ExcludedBones[i];
		if (CurExcludedVerletBone.bStraightenExcludedBonesByParent == false)
			continue;

		if (CurExcludedVerletBone.HasVerletParentBone())
		{
			FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurExcludedVerletBone.ParentVerletBoneIndex];
			if (ParentVerletBone.ChildVerletBoneIndexes.Num() > 0)
			{
				FVector ChildPoseCenterLocation = FVector::ZeroVector;
				FVector ChildCenterLocation = FVector::ZeroVector;
				for (const int32 CurChildVerletBoneIndex : ParentVerletBone.ChildVerletBoneIndexes)
				{
					const FLKAnimVerletBone& ChildVerletBone = SimulateBones[CurChildVerletBoneIndex];
					ChildPoseCenterLocation += ChildVerletBone.PoseLocation;
					ChildCenterLocation += ChildVerletBone.Location;
				}
				ChildPoseCenterLocation /= ParentVerletBone.ChildVerletBoneIndexes.Num();
				ChildCenterLocation /= ParentVerletBone.ChildVerletBoneIndexes.Num();

				const FVector ParentToChildDir = (ChildCenterLocation - ParentVerletBone.Location).GetSafeNormal();
				const FVector CurExcludedVerletBoneSrcLoc = CurExcludedVerletBone.HasExcludedParentBone() ? ExcludedBones[CurExcludedVerletBone.ParentExcludedBoneIndex].Location : ParentVerletBone.Location;
				CurExcludedVerletBone.Location = CurExcludedVerletBoneSrcLoc + ParentToChildDir * CurExcludedVerletBone.LengthToParent;
			}
			else if (ParentVerletBone.HasParentBone())
			{
				const FLKAnimVerletBone& GrandParentVerletBone = SimulateBones[ParentVerletBone.ParentVerletBoneIndex];
				const FVector GrandParentToParentPose = ParentVerletBone.PoseLocation - GrandParentVerletBone.PoseLocation;
				const FVector GrandParentToParentVerlet = ParentVerletBone.Location - GrandParentVerletBone.Location;

				FVector GrandParentToParentVerletDir = FVector::ZeroVector;
				float GrandParentToParentVerletSize = 0.0f;
				GrandParentToParentVerlet.ToDirectionAndLength(OUT GrandParentToParentVerletDir, OUT GrandParentToParentVerletSize);

				const FVector CurExcludedVerletBoneSrcLoc = CurExcludedVerletBone.HasExcludedParentBone() ? ExcludedBones[CurExcludedVerletBone.ParentExcludedBoneIndex].Location : ParentVerletBone.Location;
				CurExcludedVerletBone.Location = CurExcludedVerletBoneSrcLoc + GrandParentToParentVerletDir * CurExcludedVerletBone.LengthToParent;
			}
			/// Parent does not have a child and grand parent == Single dot simulation == do nothing
			///else
			///{
			///}
		}
	}

	/// Calculate all relevant bone`s final rotation
	for (int32 i = RelevantBoneIndicators.Num() - 1; i >= 0; --i)
	{
		const FLKAnimVerletBoneIndicator& CurBoneIndicator = RelevantBoneIndicators[i];		verify(CurBoneIndicator.IsValidBoneIndicator());
		if (CurBoneIndicator.HasParentSimulateBone() == false && CurBoneIndicator.HasParentExcludedBone() == false)
			continue;

		const FLKAnimVerletBoneBase* CurBone = nullptr;
		if (CurBoneIndicator.bExcludedBone == false) 
			CurBone = &SimulateBones[CurBoneIndicator.AnimVerletBoneIndex];
		else
			CurBone = &ExcludedBones[CurBoneIndicator.AnimVerletBoneIndex];

		FLKAnimVerletBoneBase* ParentBone = nullptr;
		if (CurBoneIndicator.HasParentSimulateBone())
			ParentBone = &SimulateBones[CurBoneIndicator.ParentAnimVerletBoneIndex];
		else
			ParentBone = &ExcludedBones[CurBoneIndicator.ParentAnimVerletBoneIndex];


		const FVector ParentToCurPose = CurBone->PoseLocation - ParentBone->PoseLocation;
		const FVector ParentToCurVerlet = CurBone->Location - ParentBone->Location;

		FVector ParentToCurVerletDir = FVector::ZeroVector;
		float ParentToCurVerletSize = 0.0f;
		ParentToCurVerlet.ToDirectionAndLength(OUT ParentToCurVerletDir, OUT ParentToCurVerletSize);

		FVector ParentToCurPoseDir = FVector::ZeroVector;
		float ParentToCurPoseSize = 0.0f;
		ParentToCurPose.ToDirectionAndLength(OUT ParentToCurPoseDir, OUT ParentToCurPoseSize);

		/// Calculate rotation
		const FQuat DeltaRotation = FQuat::FindBetweenNormals(ParentToCurPoseDir, ParentToCurVerletDir);
		///const FVector RotationAxis = FVector::CrossProduct(ParentToCurPoseDir, ParentToCurVerletDir).GetSafeNormal();
		///const float RotationAngle = FMath::Acos(FVector::DotProduct(ParentToCurPoseDir, ParentToCurVerletDir));
		///const FQuat DeltaRotation = FQuat(RotationAxis, RotationAngle);
		ParentBone->Rotation = DeltaRotation * ParentBone->PoseRotation;
		ParentBone->Rotation.Normalize();
	}
}

void FLKAnimNode_AnimVerlet::ApplyResult(OUT TArray<FBoneTransform>& OutBoneTransforms, const FBoneContainer& BoneContainer)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_ApplyResult);
#endif

	for (int32 i = 0; i < RelevantBoneIndicators.Num(); ++i)
	{
		const FLKAnimVerletBoneIndicator& CurBoneIndicator = RelevantBoneIndicators[i];		verify(CurBoneIndicator.IsValidBoneIndicator());
		
		bool bFakeBone = false;
		const FLKAnimVerletBoneBase* CurBone = nullptr;
		if (CurBoneIndicator.bExcludedBone == false)
		{
			const FLKAnimVerletBone& CurVerletBone = SimulateBones[CurBoneIndicator.AnimVerletBoneIndex];
			CurBone = &CurVerletBone;
			bFakeBone = CurVerletBone.bFakeBone;
		}
		else
		{
			CurBone = &ExcludedBones[CurBoneIndicator.AnimVerletBoneIndex];
		}

		if (bFakeBone)
			continue;

		const FCompactPoseBoneIndex BonePoseIndex = CurBone->BoneReference.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		if (BonePoseIndex != INDEX_NONE)
		{
			const FTransform ResultBoneT(CurBone->Rotation, CurBone->Location, CurBone->PoseScale);
			OutBoneTransforms.Emplace(FBoneTransform(BonePoseIndex, ResultBoneT));
		}
	}

	/// Need to sort by UE4 rules (SimulateBones != OutBoneTransforms)
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}

void FLKAnimNode_AnimVerlet::ClearSimulateBones()
{
	SimulatingCollisionShapes.ResetCollisionShapeList();

	///Constraints.Reset();
	PinConstraints.Reset();
	DistanceConstraints.Reset();
	BendingConstraints.Reset();
	StraightenConstraints.Reset();
	FixedDistanceConstraints.Reset();
	BallSocketConstraints.Reset();
	SphereCollisionConstraints.Reset();
	CapsuleCollisionConstraints.Reset();
	BoxCollisionConstraints.Reset();
	PlaneCollisionConstraints.Reset();
	WorldCollisionConstraints.Reset();

	BoneChainIndexes.Reset();
	MaxBoneChainLength = 0;
	BroadphaseSpace.ResetBroadphaseSpace();
	BroadphaseContainer.Destroy();
	RelevantBoneIndicators.Reset();
	SimulateBonePairIndicators.Reset();
	ExcludedBones.Reset();
	SimulateBones.Reset();
}

void FLKAnimNode_AnimVerlet::ResetSimulation()
{
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		CurVerletBone.ResetSimulation();
	}

	ForEachConstraints([](FLKAnimVerletConstraint& CurConstraint) {
		CurConstraint.ResetSimulation();
	});
}

template <typename Predicate>
void FLKAnimNode_AnimVerlet::ForEachConstraints(Predicate Pred)
{
	for (FLKAnimVerletConstraint_Pin& CurConstraint : PinConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_Distance& CurConstraint : DistanceConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_IsometricBending& CurConstraint : BendingConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_Straighten& CurConstraint : StraightenConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_FixedDistance& CurConstraint : FixedDistanceConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_BallSocket& CurConstraint : BallSocketConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_Sphere& CurConstraint : SphereCollisionConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_Capsule& CurConstraint : CapsuleCollisionConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_Box& CurConstraint : BoxCollisionConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_Plane& CurConstraint : PlaneCollisionConstraints)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_World& CurConstraint : WorldCollisionConstraints)
	{
		Pred(CurConstraint);
	}
}

void FLKAnimNode_AnimVerlet::ResetCollisionShapes()
{
	SphereCollisionShapes.Reset();
	CapsuleCollisionShapes.Reset();
	BoxCollisionShapes.Reset();
	PlaneCollisionShapes.Reset();
}

void FLKAnimNode_AnimVerlet::CollisionShapesToCollisionShapeList(OUT FLKAnimVerletCollisionShapeList& OutShapeList) const
{
	OutShapeList.SphereCollisionShapes = SphereCollisionShapes;
	OutShapeList.CapsuleCollisionShapes = CapsuleCollisionShapes;
	OutShapeList.BoxCollisionShapes = BoxCollisionShapes;
	OutShapeList.PlaneCollisionShapes = PlaneCollisionShapes;
}

void FLKAnimNode_AnimVerlet::CollisionShapesFromCollisionShapeList(const FLKAnimVerletCollisionShapeList& InShapeList)
{
	SphereCollisionShapes = InShapeList.SphereCollisionShapes;
	CapsuleCollisionShapes = InShapeList.CapsuleCollisionShapes;
	BoxCollisionShapes = InShapeList.BoxCollisionShapes;
	PlaneCollisionShapes = InShapeList.PlaneCollisionShapes;
}

bool FLKAnimNode_AnimVerlet::ConvertCollisionShapesToDataAsset()
{
	if (CollisionDataAsset == nullptr)
		return false;

	FLKAnimVerletCollisionShapeList ShapeList;
	CollisionShapesToCollisionShapeList(OUT ShapeList);

	CollisionDataAsset->Reset();
	CollisionDataAsset->ConvertFromShape(ShapeList);

	return true;
}

bool FLKAnimNode_AnimVerlet::ConvertCollisionShapesFromDataAsset()
{
	if (CollisionDataAsset == nullptr)
		return false;

	ResetCollisionShapes();

	FLKAnimVerletCollisionShapeList ShapeList;
	CollisionDataAsset->ConvertToShape(OUT ShapeList);
	CollisionShapesFromCollisionShapeList(ShapeList);

	return true;
}

void FLKAnimNode_AnimVerlet::SyncFromOtherAnimVerletNode(const FLKAnimNode_AnimVerlet& Other)
{
	///VerletBones = Other.VerletBones;

	bSubDivideBones = Other.bSubDivideBones;
	NumSubDividedBone = Other.NumSubDividedBone;
	bSkipUpdateOnDedicatedServer = Other.bSkipUpdateOnDedicatedServer;

	bMakeFakeTipBone = Other.bMakeFakeTipBone;
	FakeTipBoneLength = Other.FakeTipBoneLength;
	bLockTipBone = Other.bLockTipBone;
	TipBoneLockMargin = Other.TipBoneLockMargin;
	StartBoneLockMargin = Other.StartBoneLockMargin;

	AnimationPoseDeltaInertia = Other.AnimationPoseDeltaInertia;
	AnimationPoseDeltaInertiaScale = Other.AnimationPoseDeltaInertiaScale;
	bClampAnimationPoseDeltaInertia = Other.bClampAnimationPoseDeltaInertia;
	AnimationPoseDeltaInertiaClampMax = Other.AnimationPoseDeltaInertiaClampMax;
	bIgnoreAnimationPose = Other.bIgnoreAnimationPose;
	AnimationPoseInertia = Other.AnimationPoseInertia;

	Damping = Other.Damping;

	bUseXPBDSolver = Other.bUseXPBDSolver;
	InvCompliance = Other.InvCompliance;
	Stiffness = Other.Stiffness;

	bUseSleep = Other.bUseSleep;
	bIgnoreSleepWhenParentWakedUp = Other.bIgnoreSleepWhenParentWakedUp;
	SleepDeltaThreshold = Other.SleepDeltaThreshold;
	SleepTriggerDuration = Other.SleepTriggerDuration;
	WakeUpDeltaThreshold = Other.WakeUpDeltaThreshold;

	bConstrainRightDiagonalDistance = Other.bConstrainRightDiagonalDistance;
	bConstrainLeftDiagonalDistance = Other.bConstrainLeftDiagonalDistance;
	bUseIsometricBendingConstraint = Other.bUseIsometricBendingConstraint;
	BendingCompliance = Other.BendingCompliance;
	BendingStiffness = Other.BendingStiffness;

	bPreserveLengthFromParent = Other.bPreserveLengthFromParent;
	LengthFromParentMargin = Other.LengthFromParentMargin;
	bPreserveSideLength = Other.bPreserveSideLength;
	SideLengthMargin = Other.SideLengthMargin;

	bStretchEachBone = Other.bStretchEachBone;
	StretchStrength = Other.StretchStrength;

	bStraightenBendedBone = Other.bStraightenBendedBone;
	StraightenBendedBoneStrength = Other.StraightenBendedBoneStrength;

	SolveIteration = Other.SolveIteration;
	bUseBroadphaseCollisionDetection = Other.bUseBroadphaseCollisionDetection;

	FixedDeltaTime = Other.FixedDeltaTime;
	MinDeltaTime = Other.MinDeltaTime;
	MaxDeltaTime = Other.MaxDeltaTime;
	bUseSquaredDeltaTime = Other.bUseSquaredDeltaTime;

	ConeAngle = Other.ConeAngle;
	Thickness = Other.Thickness;
	bUseCapsuleCollisionForChain = Other.bUseCapsuleCollisionForChain;

	WorldCollisionProfile = Other.WorldCollisionProfile;
	WorldCollisionExcludeBones = Other.WorldCollisionExcludeBones;

	SphereCollisionShapes = Other.SphereCollisionShapes;
	CapsuleCollisionShapes = Other.CapsuleCollisionShapes;
	BoxCollisionShapes = Other.BoxCollisionShapes;
	PlaneCollisionShapes = Other.PlaneCollisionShapes;
	CollisionDataAsset = Other.CollisionDataAsset;
	DynamicCollisionShapes = Other.DynamicCollisionShapes;

	Gravity = Other.Gravity;
	bGravityInWorldSpace = Other.bGravityInWorldSpace;

	StretchForce = Other.StretchForce;
	SideStraightenForce = Other.SideStraightenForce;
	ShapeMemoryForce = Other.ShapeMemoryForce;
	ExternalForce = Other.ExternalForce;
	bExternalForceInWorldSpace = Other.bExternalForceInWorldSpace;

	RandomWindDirection = Other.RandomWindDirection;
	RandomWindSizeMin = Other.RandomWindSizeMin;
	RandomWindSizeMax = Other.RandomWindSizeMax;
	bRandomWindDirectionInWorldSpace = Other.bRandomWindDirectionInWorldSpace;

	bAdjustWindComponent = Other.bAdjustWindComponent;
	WindComponentScale = Other.WindComponentScale;

	MoveInertiaScale = Other.MoveInertiaScale;
	bClampMoveInertia = Other.bClampMoveInertia;
	MoveInertiaClampMaxDistance = Other.MoveInertiaClampMaxDistance;
	RotationInertiaScale = Other.RotationInertiaScale;
	bClampRotationInertia = Other.bClampRotationInertia;
	RotationInertiaClampDegrees = Other.RotationInertiaClampDegrees;
}

void FLKAnimNode_AnimVerlet::DebugDrawAnimVerlet(const FComponentSpacePoseContext& Output)
{
	FAnimInstanceProxy* AnimInstanceProxy = Output.AnimInstanceProxy;
	if (AnimInstanceProxy == nullptr)
		return;

	USkeletalMeshComponent* SkeletalMeshComponent = AnimInstanceProxy->GetSkelMeshComponent();
	if (SkeletalMeshComponent == nullptr)
		return;

	const UWorld* World = SkeletalMeshComponent->GetWorld();
	const FTransform ComponentToWorld = AnimInstanceProxy->GetComponentTransform();
	for (const FLKAnimVerletBone& CurBone : SimulateBones)
	{
		const FVector WorldLocation = ComponentToWorld.TransformPosition(CurBone.Location);

		const bool bSleep = CurBone.IsSleep();
		AnimInstanceProxy->AnimDrawDebugSphere(WorldLocation, CurBone.Thickness, 16, bSleep ? FColor::Turquoise : (CurBone.bFakeBone ? FColor::Black : FColor::Yellow), false, -1.0f, 0.0f, SDPG_Foreground);
	}

#if (ENGINE_MINOR_VERSION >= 4)
	for (const FLKAnimVerletBoneIndicatorPair& CurPair : SimulateBonePairIndicators)
	{
		if (CurPair.BoneB.IsValidBoneIndicator() == false || SimulateBones.IsValidIndex(CurPair.BoneB.AnimVerletBoneIndex) == false)
			continue;

		const FLKAnimVerletBone& CurVerletBone = SimulateBones[CurPair.BoneB.AnimVerletBoneIndex];
		if (CurPair.BoneA.IsValidBoneIndicator() == false || CurVerletBone.bOverrideToUseSphereCollisionForChain)
			continue;

		const FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurPair.BoneA.AnimVerletBoneIndex];

		const FVector WorldLocation = ComponentToWorld.TransformPosition((CurVerletBone.Location + ParentVerletBone.Location) * 0.5f);
		const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(ParentVerletBone.Location - CurVerletBone.Location).ToQuat();
		const FQuat WorldRotation = ComponentToWorld.TransformRotation(CapsuleRotation);
		AnimInstanceProxy->AnimDrawDebugCapsule(WorldLocation, (CurVerletBone.Location - ParentVerletBone.Location).Size() * 0.5f + CurVerletBone.Thickness, CurVerletBone.Thickness, WorldRotation.Rotator(), FColor::Blue, false, -1.0f, SDPG_Foreground);
	}
#endif

	for (const FLKAnimVerletConstraint_Distance& CurConstraint : DistanceConstraints)
	{
		const FVector WorldLocationA = ComponentToWorld.TransformPosition(CurConstraint.BoneA->Location);
		const FVector WorldLocationB = ComponentToWorld.TransformPosition(CurConstraint.BoneB->Location);

		AnimInstanceProxy->AnimDrawDebugLine(WorldLocationA, WorldLocationB, FColor::White, false, -1.0f, 0.0f, SDPG_Foreground);
	}

#if LK_ENABLE_ANIMVERLET_DEBUG
	if (CVarAnimNodeAnimVerletDebugBallSocket.GetValueOnAnyThread())
	{
		for (const FLKAnimVerletConstraint_BallSocket& CurConstraint : BallSocketConstraints)
		{
			const FVector WorldLocationA = ComponentToWorld.TransformPosition(CurConstraint.BoneA->Location);
			FVector WorldLocationATarget = FVector::ZeroVector;
			FVector WorldLocationBTarget = FVector::ZeroVector;

			if (CurConstraint.GrandParentBoneNullable != nullptr && CurConstraint.ParentBoneNullable != nullptr)
			{
				WorldLocationATarget = ComponentToWorld.TransformPosition(CurConstraint.GrandParentBoneNullable->Location);
				WorldLocationBTarget = ComponentToWorld.TransformPosition(CurConstraint.ParentBoneNullable->Location);
			}
			else
			{
				WorldLocationATarget = ComponentToWorld.TransformPosition(CurConstraint.BoneA->PoseLocation);
				WorldLocationBTarget = ComponentToWorld.TransformPosition(CurConstraint.BoneB->PoseLocation);
			}

			FVector Dir = FVector::ZeroVector;
			float Length = 0.0f;
			(WorldLocationBTarget - WorldLocationATarget).ToDirectionAndLength(OUT Dir, OUT Length);
			AnimInstanceProxy->AnimDrawDebugCone(WorldLocationA, Length, Dir, FMath::DegreesToRadians(CurConstraint.AngleDegrees), FMath::DegreesToRadians(CurConstraint.AngleDegrees), 16, FColor::Magenta, false, -1.0f, SDPG_Foreground);
		}
	}

	/*if (CVarAnimNodeAnimVerletDebugPlane.GetValueOnAnyThread())
	{
		for (const FLKAnimVerletConstraint_Plane& CurConstraint : PlaneCollisionConstraints)
		{
			const FVector WorldLocation = ComponentToWorld.TransformPosition(CurConstraint.PlaneBase);
			const FVector WorldNormal = ComponentToWorld.TransformVectorNoScale(CurConstraint.PlaneNormal);
			const FQuat WorldRotation = ComponentToWorld.TransformRotation(CurConstraint.Rotation);

			if (CurConstraint.PlaneHalfExtents.IsNearlyZero() == false)
			{
				DrawDebugBox(World, WorldLocation, FVector(CurConstraint.PlaneHalfExtents * 2.0f, 1.0f), WorldRotation, FColor::Blue, false, -1.0f, SDPG_Foreground);
				DrawDebugDirectionalArrow(World, WorldLocation, WorldLocation + WorldNormal * 50.0f, 20.0f, FColor::Orange, false, -1.0f, SDPG_Foreground);
			}
			else
			{
				DrawDebugBox(World, WorldLocation, FVector(100.0f, 100.0f, 1.0f), WorldRotation, FColor::Blue, false, -1.0f, SDPG_Foreground);
				DrawDebugDirectionalArrow(World, WorldLocation, WorldLocation + WorldNormal * 50.0f, 20.0f, FColor::Orange, false, -1.0f, SDPG_Foreground);
			}
		}
	}*/

	if (CVarAnimNodeAnimVerletDebugSphereCollision.GetValueOnAnyThread())
	{
		for (const FLKAnimVerletConstraint_Sphere& CurConstraint : SphereCollisionConstraints)
		{
			const FVector WorldLocation = ComponentToWorld.TransformPosition(CurConstraint.Location);

			AnimInstanceProxy->AnimDrawDebugSphere(WorldLocation, CurConstraint.Radius, 16, FColor::Blue, false, -1.0f, 0.0f, SDPG_Foreground);
		}
	}

#if (ENGINE_MINOR_VERSION >= 4)
	if (CVarAnimNodeAnimVerletDebugCapsuleCollision.GetValueOnAnyThread())
	{
		for (const FLKAnimVerletConstraint_Capsule& CurConstraint : CapsuleCollisionConstraints)
		{
			const FVector WorldLocation = ComponentToWorld.TransformPosition(CurConstraint.Location);
			const FQuat WorldRotation = ComponentToWorld.TransformRotation(CurConstraint.Rotation);

			AnimInstanceProxy->AnimDrawDebugCapsule(WorldLocation, CurConstraint.HalfHeight + CurConstraint.Radius, CurConstraint.Radius, WorldRotation.Rotator(), FColor::Blue, false, -1.0f, SDPG_Foreground);
		}
	}
#endif

	/*if (CVarAnimNodeAnimVerletDebugBoxCollision.GetValueOnAnyThread())
	{
		for (const FLKAnimVerletConstraint_Box& CurConstraint : BoxCollisionConstraints)
		{
			const FVector WorldLocation = ComponentToWorld.TransformPosition(CurConstraint.Location);
			const FQuat WorldRotation = ComponentToWorld.TransformRotation(CurConstraint.Rotation);

			DrawDebugBox(World, WorldLocation, CurConstraint.HalfExtents * 2.0f, WorldRotation, FColor::Blue, false, -1.0f, SDPG_Foreground);
		}
	}*/
#endif
}