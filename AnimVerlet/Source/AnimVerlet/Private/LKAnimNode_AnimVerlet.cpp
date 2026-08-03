#include "LKAnimNode_AnimVerlet.h"

#include <AnimationRuntime.h>
#include <Animation/AnimInstanceProxy.h>
#include <Animation/AnimTypes.h>
#include <DrawDebugHelpers.h>
#include <Kismet/KismetSystemLibrary.h>
#include <PhysicsEngine/PhysicsAsset.h>
#if (ENGINE_MINOR_VERSION >= 5)
#include <PhysicsEngine/SkeletalBodySetup.h>
#endif
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
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_UpdateBroadphase"), STAT_AnimVerlet_UpdateBroadphase, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints"), STAT_AnimVerlet_SolveConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_PinConstraints"), STAT_AnimVerlet_SolveConstraints_PinConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_DistanceConstraints"), STAT_AnimVerlet_SolveConstraints_DistanceConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BendingConstraints"), STAT_AnimVerlet_SolveConstraints_BendingConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BendingConstraints_1D"), STAT_AnimVerlet_SolveConstraints_BendingConstraints_1D, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_FlatBendingConstraints"), STAT_AnimVerlet_SolveConstraints_FlatBendingConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_StraightenConstraints"), STAT_AnimVerlet_SolveConstraints_StraightenConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BallSocketConstraints"), STAT_AnimVerlet_SolveConstraints_BallSocketConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_SphereCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_SphereCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_CapsuleCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_CapsuleCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_BoxCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_BoxCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_PlaneCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_PlaneCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_WorldCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_WorldCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_SelfCollisionConstraints"), STAT_AnimVerlet_SolveConstraints_SelfCollisionConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_SolveConstraints_FixedDistanceConstraints"), STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_UpdateSleep"), STAT_AnimVerlet_UpdateSleep, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_PostUpdateBones"), STAT_AnimVerlet_PostUpdateBones, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("AnimVerlet_ApplyResult"), STAT_AnimVerlet_ApplyResult, STATGROUP_Anim);

static constexpr float LKG_MINFPS = 30.0f;
static constexpr float LKG_MAXFPS = 500.0f;

FLKAnimNode_AnimVerlet::FLKAnimNode_AnimVerlet()
	: FAnimNode_SkeletalControlBase()
{

}

void FLKAnimNode_AnimVerlet::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);

	bPendingSimulationLODRebuild = false;
	bPendingDynamicsReset = false;
	bWarmupPending = false;
	CachedSimulationLOD = INDEX_NONE;
	FixedStepAccumulator = 0.0f;
	NumPendingSimulationSteps = 0;
	ResetOutputBlend();

	FBoneContainer& RequiredBones = Context.AnimInstanceProxy->GetRequiredBones();
	InitializeBoneReferences(RequiredBones);

	ClearSimulateBones();
}

void FLKAnimNode_AnimVerlet::ResetDynamics(ETeleportType InTeleportType)
{
	if (InTeleportType == ETeleportType::ResetPhysics)
	{
		/// Defer the reset until Evaluate so the simulation is reset against the current animation pose and component transform rather than stale state.
		bPendingDynamicsReset = true;
	}
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
	const int32 CurrentLOD = Output.AnimInstanceProxy->GetLODLevel();
	if (CachedSimulationLOD != INDEX_NONE && CurrentLOD != CachedSimulationLOD && bRebuildSimulationOnLODChange && SimulateBones.Num() > 0)
		bPendingSimulationLODRebuild = true;
	CachedSimulationLOD = CurrentLOD;

	const FTransform CurComponentT = Output.AnimInstanceProxy->GetComponentTransform();
	bool bInitializedThisFrame = false;
	if (bPendingSimulationLODRebuild)
	{
		if (bRebuildSimulationOnLODChange && SimulateBones.Num() > 0)
		{
			RebuildSimulationForLOD(Output, BoneContainer);
			PrevComponentT = CurComponentT;
		}
		bPendingSimulationLODRebuild = false;
	}

	if (SimulateBones.Num() == 0)
	{
		InitializeSimulateBones(Output, BoneContainer);
		PrevComponentT = CurComponentT;
		bInitializedThisFrame = true;
	}

	const USkeletalMeshComponent* SkeletalMeshComponent = Output.AnimInstanceProxy->GetSkelMeshComponent();
	const UWorld* World = SkeletalMeshComponent->GetWorld();
	bool bAdvanceOutputBlend = false;
	float OutputBlendDeltaTime = 0.0f;
	if (bUseWarmup == false)
		bWarmupPending = false;

	/// Initialization and ResetPhysics only synchronize state on this frame.
	/// Warmup is intentionally deferred until the next evaluated frame.
	if (bInitializedThisFrame || bPendingDynamicsReset)
	{
		/// Prepare each SimulateBones
		PrepareSimulation(Output, BoneContainer, CurComponentT);
		ResetSimulation();

		PrevComponentT = CurComponentT;
		bPendingDynamicsReset = false;
		bWarmupPending = bUseWarmup;
		ResetOutputBlend();
	}
	else if (bWarmupPending && bPause == false)
	{
		/// The animation pose may have changed during the deferred frame. Start warmup from the pose prepared on this frame, with no inherited velocity.
		PrepareSimulation(Output, BoneContainer, CurComponentT);
		ResetSimulation();

		const int32 ClampedWarmupStepCount = FMath::Max(WarmupStepCount, 0);
		const float ClampedWarmupDeltaTime = FMath::Max(WarmupFixedDeltaTime, UE_SMALL_NUMBER);
		for (int32 WarmupStep = 0; WarmupStep < ClampedWarmupStepCount; ++WarmupStep)
		{
			if (WarmupStep > 0)
				PrepareSimulation(Output, BoneContainer, CurComponentT);
			/// Using the current component transform for both frames prevents component movement/rotation inertia from entering the warmup.
			SimulateVerlet(World, ClampedWarmupDeltaTime, CurComponentT, CurComponentT);
		}

		bWarmupPending = false;
		PrevComponentT = CurComponentT;
		bAdvanceOutputBlend = true;
		OutputBlendDeltaTime = DeltaTime;
	}
	/// Simulate verlet integration
	else if (DeltaTime > 0.0f && NumPendingSimulationSteps > 0 && bPause == false)
	{
		const int32 SimulationStepCount = NumPendingSimulationSteps;
		for (int32 SimulationStep = 0; SimulationStep < SimulationStepCount; ++SimulationStep)
		{
			const float PrevStepAlpha = static_cast<float>(SimulationStep) / static_cast<float>(SimulationStepCount);
			const float CurStepAlpha = static_cast<float>(SimulationStep + 1) / static_cast<float>(SimulationStepCount);
			FTransform PrevStepComponentT;
			FTransform CurStepComponentT;
			PrevStepComponentT.Blend(PrevComponentT, CurComponentT, PrevStepAlpha);
			CurStepComponentT.Blend(PrevComponentT, CurComponentT, CurStepAlpha);

			/// Prepare every fixed step so Verlet history advances correctly between substeps.
			PrepareSimulation(Output, BoneContainer, CurStepComponentT);
			SimulateVerlet(World, DeltaTime, CurStepComponentT, PrevStepComponentT);
		}
		FixedStepAccumulator = FMath::Max(FixedStepAccumulator - static_cast<float>(SimulationStepCount), 0.0f);
		NumPendingSimulationSteps = 0;
		PrevComponentT = CurComponentT;
		bAdvanceOutputBlend = true;
		OutputBlendDeltaTime = DeltaTime * static_cast<float>(SimulationStepCount);
	}

	if (bAdvanceOutputBlend)
		AdvanceOutputBlend(OutputBlendDeltaTime);

	/// Apply simulation to bone
	ApplyResult(OutBoneTransforms, Output, BoneContainer);

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

	for (FLKAnimVerletCustomDistanceConstraintSetting& CurConstraintSetting : CustomDistanceConstraints)
	{
		CurConstraintSetting.BoneA.Initialize(RequiredBones);
		CurConstraintSetting.BoneB.Initialize(RequiredBones);
	}

	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		SimulateBones[i].BoneReference.Initialize(RequiredBones);
	}

	for (int32 i = 0; i < ExcludedBones.Num(); ++i)
	{
		ExcludedBones[i].BoneReference.Initialize(RequiredBones);
	}

	for (FLKAnimVerletCollisionSphere& CurShape : SimulatingCollisionShapes.SphereCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}
	for (FLKAnimVerletCollisionCapsule& CurShape : SimulatingCollisionShapes.CapsuleCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}
	for (FLKAnimVerletCollisionBox& CurShape : SimulatingCollisionShapes.BoxCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}
	for (FLKAnimVerletCollisionPlane& CurShape : SimulatingCollisionShapes.PlaneCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}

	for (FLKAnimVerletCollisionSphere& CurShape : DynamicCollisionShapes.SphereCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}
	for (FLKAnimVerletCollisionCapsule& CurShape : DynamicCollisionShapes.CapsuleCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}
	for (FLKAnimVerletCollisionBox& CurShape : DynamicCollisionShapes.BoxCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
	}
	for (FLKAnimVerletCollisionPlane& CurShape : DynamicCollisionShapes.PlaneCollisionShapes)
	{
		InitializeAttachedShape(CurShape, RequiredBones);
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

	if (bActivate == false)
		return false;

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

	UpdateDeltaTime(Context.GetDeltaTime() * PlaySpeedRate, Context.AnimInstanceProxy != nullptr ? Context.AnimInstanceProxy->GetTimeDilation() * PlaySpeedRate : PlaySpeedRate);
}

void FLKAnimNode_AnimVerlet::InitializeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
	verify(SimulateBones.Num() == 0);

	USkeleton* Skeleton = BoneContainer.GetSkeletonAsset();
	const FReferenceSkeleton& ReferenceSkeleton = Skeleton->GetReferenceSkeleton();

	MaxThickness = Thickness;

	/// Create simulate bones
	for (const FLKAnimVerletBoneSetting& CurBoneSetting : VerletBones)
	{
		const int32 FoundBoneIndex = ReferenceSkeleton.FindBoneIndex(CurBoneSetting.RootBone.BoneName);
		if (FoundBoneIndex != INDEX_NONE)
			MakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, FoundBoneIndex, INDEX_NONE, INDEX_NONE, CurBoneSetting, false, INDEX_NONE);
	}

	/// Create constraints
	const bool bSingleChain = IsSingleChain();
	const double Compliance = static_cast<double>(1.0 / InvCompliance);
	const float BendingInvComplianceAtRest = bUseBendingComplianceRange ? FMath::Min(InvBendingComplianceMin, InvBendingComplianceMax) : InvBendingCompliance;
	const float BendingInvComplianceWhenFolded = bUseBendingComplianceRange ? FMath::Max(InvBendingComplianceMin, InvBendingComplianceMax) : InvBendingCompliance;
	const double BendingComplianceAtRest = 1.0 / static_cast<double>(FMath::Max(BendingInvComplianceAtRest, UE_SMALL_NUMBER));
	const double BendingComplianceWhenFolded = 1.0 / static_cast<double>(FMath::Max(BendingInvComplianceWhenFolded, UE_SMALL_NUMBER));
	const float BendingStiffnessAtRest = bUseBendingStiffnessRange ? FMath::Min(BendingStiffnessMin, BendingStiffnessMax) : BendingStiffness;
	const float BendingStiffnessWhenFolded = bUseBendingStiffnessRange ? FMath::Max(BendingStiffnessMin, BendingStiffnessMax) : BendingStiffness;
	const float BendingMaxAngle = bUseXPBDSolver ? BendingComplianceMaxAngle : BendingStiffnessMaxAngle;
	const float BendingMaxAngleRadians = FMath::DegreesToRadians(FMath::Max(BendingMaxAngle, 0.0f));
	const float FlatBendingInvComplianceAtRest = bUseFlatBendingComplianceRange ? FMath::Min(InvFlatBendingComplianceMin, InvFlatBendingComplianceMax) : InvFlatBendingCompliance;
	const float FlatBendingInvComplianceWhenFolded = bUseFlatBendingComplianceRange ? FMath::Max(InvFlatBendingComplianceMin, InvFlatBendingComplianceMax) : InvFlatBendingCompliance;
	const double FlatBendingComplianceAtRest = 1.0 / static_cast<double>(FMath::Max(FlatBendingInvComplianceAtRest, UE_SMALL_NUMBER));
	const double FlatBendingComplianceWhenFolded = 1.0 / static_cast<double>(FMath::Max(FlatBendingInvComplianceWhenFolded, UE_SMALL_NUMBER));
	const float FlatBendingStiffnessAtRest = bUseFlatBendingStiffnessRange ? FMath::Min(FlatBendingStiffnessMin, FlatBendingStiffnessMax) : FlatBendingStiffness;
	const float FlatBendingStiffnessWhenFolded = bUseFlatBendingStiffnessRange ? FMath::Max(FlatBendingStiffnessMin, FlatBendingStiffnessMax) : FlatBendingStiffness;
	const float FlatBendingMaxAngle = bUseXPBDSolver ? FlatBendingComplianceMaxAngle : FlatBendingStiffnessMaxAngle;
	const float FlatBendingMaxAngleRadians = FMath::DegreesToRadians(FMath::Max(FlatBendingMaxAngle, 0.0f));
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurSimulateBone = SimulateBones[i];
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

				if (bPreserveLengthFromParentBetweenRealBones && CurSimulateBone.bFakeBone == false && ParentSimulateBone.bSubDividedBone)
				{
					FLKAnimVerletBone* RealParentSimulateBone = &ParentSimulateBone;
					while (RealParentSimulateBone != nullptr && RealParentSimulateBone->bFakeBone)
					{
						if (RealParentSimulateBone->HasParentBone())
							RealParentSimulateBone = &SimulateBones[RealParentSimulateBone->ParentVerletBoneIndex];
						else
							RealParentSimulateBone = nullptr;
					}

					if (RealParentSimulateBone != nullptr)
					{
						const FLKAnimVerletConstraint_FixedDistance RealFixedDistanceConstraint(RealParentSimulateBone, &CurSimulateBone, bStretchEachBone, StretchStrength, false, LengthFromParentMargin);
						FixedDistanceConstraints.Emplace(RealFixedDistanceConstraint);
					}
				}
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
																			  CurSimulateBone.ConeAngleConstraint, CurSimulateBone.ConeAngleOffset, bUseXPBDSolver, Compliance);
				BallSocketConstraints.Emplace(BallSocketConstraint);
			}
			else if (ConeAngle > 0.0f)
			{
				FLKAnimVerletBone* GrandParentBoneNullable = nullptr;
				FLKAnimVerletBone* ParentBoneNullable = nullptr;
				if (CurSimulateBone.bConstrainConeAngleFromParent && ParentSimulateBone.HasParentBone())
				{
					GrandParentBoneNullable = &SimulateBones[ParentSimulateBone.ParentVerletBoneIndex];
					ParentBoneNullable = &ParentSimulateBone;
				}

				const FLKAnimVerletConstraint_BallSocket BallSocketConstraint(&ParentSimulateBone, &CurSimulateBone, GrandParentBoneNullable, ParentBoneNullable,
																			  ConeAngle, CurSimulateBone.ConeAngleOffset, bUseXPBDSolver, Compliance);
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

			if (ParentSimulateBone.HasParentBone())
			{
				FLKAnimVerletBone& GrandParentSimulateBone = SimulateBones[ParentSimulateBone.ParentVerletBoneIndex];
				if (bStraightenBendedBone)
				{
					const FLKAnimVerletConstraint_Straighten StraightenConstraint(&GrandParentSimulateBone, &ParentSimulateBone, &CurSimulateBone, StraightenBendedBoneStrength, false);
					StraightenConstraints.Emplace(StraightenConstraint);
				}

				if (bSingleChain && bUseIsometricBendingConstraint)
				{
					const FLKAnimVerletConstraint_Bending_1D BendingConstraint(&GrandParentSimulateBone, &ParentSimulateBone, &CurSimulateBone, bUseXPBDSolver,
																			  (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																			  BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
					BendingConstraints_1D.Emplace(BendingConstraint);
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
				if (LeftCurIndex != MidIndex)
				{
					const int32 LeftCurRightIndex = LeftCurIndex + 1;
					if (BoneChainIndexes.IsValidIndex(LeftIndex) && BoneChainIndexes.IsValidIndex(LeftCurRightIndex))
					{
						const TArray<int32>& CurBoneChain = BoneChainIndexes[LeftCurIndex];
						const TArray<int32>& LeftBoneChain = BoneChainIndexes[LeftIndex];
						const TArray<int32>& RightBoneChain = BoneChainIndexes[LeftCurRightIndex];
						if (i < CurBoneChain.Num() && i < LeftBoneChain.Num() && i < RightBoneChain.Num())
						{
							verify(SimulateBones.IsValidIndex(CurBoneChain[i]));
							verify(SimulateBones.IsValidIndex(LeftBoneChain[i]));
							verify(SimulateBones.IsValidIndex(RightBoneChain[i]));

							if (bStraightenBendedBone)
							{
								const FLKAnimVerletConstraint_Straighten StraightenConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], StraightenBendedBoneStrength, true);
								StraightenConstraints.Emplace(StraightenConstraint);
							}

							if (bUseIsometricBendingConstraint)
							{
								if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
								{
									const FLKAnimVerletConstraint_IsometricBending BendingConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i + 1]],
																									bUseXPBDSolver, (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																									BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
									BendingConstraints.Emplace(BendingConstraint);
								}
							}

							if (bUseFlatBendingConstraint)
							{
								if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
								{
									const FLKAnimVerletConstraint_FlatBending FlatBendingConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i + 1]],
																									bUseXPBDSolver, (bUseXPBDSolver ? FlatBendingComplianceAtRest : FlatBendingStiffnessAtRest), FlatBendingAlpha,
																									FlatBendingComplianceWhenFolded, FlatBendingStiffnessWhenFolded, FlatBendingMaxAngleRadians);
									FlatBendingConstraints.Emplace(FlatBendingConstraint);
								}
							}
						}
					}
				}

				const int32 RightCurLeftIndex = RightCurIndex - 1;
				if (BoneChainIndexes.IsValidIndex(RightCurLeftIndex) && BoneChainIndexes.IsValidIndex(RightIndex))
				{
					const TArray<int32>& CurBoneChain = BoneChainIndexes[RightCurIndex];
					const TArray<int32>& LeftBoneChain = BoneChainIndexes[RightCurLeftIndex];
					const TArray<int32>& RightBoneChain = BoneChainIndexes[RightIndex];
					if (i < CurBoneChain.Num() && i < LeftBoneChain.Num() && i < RightBoneChain.Num())
					{
						verify(SimulateBones.IsValidIndex(CurBoneChain[i]));
						verify(SimulateBones.IsValidIndex(LeftBoneChain[i]));
						verify(SimulateBones.IsValidIndex(RightBoneChain[i]));

						if (bStraightenBendedBone)
						{
							const FLKAnimVerletConstraint_Straighten StraightenConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], StraightenBendedBoneStrength, true);
							StraightenConstraints.Emplace(StraightenConstraint);
						}

						if (bUseIsometricBendingConstraint)
						{
							if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending BendingConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i + 1]],
																								bUseXPBDSolver, (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																								BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
								BendingConstraints.Emplace(BendingConstraint);
							}
						}

						if (bUseFlatBendingConstraint)
						{
							if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_FlatBending FlatBendingConstraint(&SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i + 1]],
																								bUseXPBDSolver, (bUseXPBDSolver ? FlatBendingComplianceAtRest : FlatBendingStiffnessAtRest), FlatBendingAlpha,
																								FlatBendingComplianceWhenFolded, FlatBendingStiffnessWhenFolded, FlatBendingMaxAngleRadians);
								FlatBendingConstraints.Emplace(FlatBendingConstraint);
							}
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

						if (bUseCapsuleCollisionForChain)
						{
							if (i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletBoneIndicatorTriangle IndicatorTriangle(FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(LeftBoneChain[i], false), FLKAnimVerletBoneIndicator(CurBoneChain[i + 1], false));
								SimulateBoneTriangleIndicators.Emplace(IndicatorTriangle);

								if (i + 1 < LeftBoneChain.Num())
								{
									const FLKAnimVerletBoneIndicatorTriangle IndicatorTriangle2(FLKAnimVerletBoneIndicator(LeftBoneChain[i], false), FLKAnimVerletBoneIndicator(CurBoneChain[i + 1], false), FLKAnimVerletBoneIndicator(LeftBoneChain[i + 1], false));
									SimulateBoneTriangleIndicators.Emplace(IndicatorTriangle2);
								}
							}
						}

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

							if (bPreserveSideLengthBetweenRealBones && SimulateBones[CurBoneChain[i]].bFakeBone == false && SimulateBones[LeftBoneChain[i]].bSubDividedBone)
							{
								int32 LeftIndexForRealBone = LeftIndex;
								FLKAnimVerletBone* RealLeftSimulateBone = &SimulateBones[LeftBoneChain[i]];
								while (RealLeftSimulateBone != nullptr && RealLeftSimulateBone->bFakeBone)
								{
									if (BoneChainIndexes.IsValidIndex(--LeftIndexForRealBone))
									{
										const TArray<int32>& LeftBoneChainForRealBone = BoneChainIndexes[LeftIndexForRealBone];
										RealLeftSimulateBone = &SimulateBones[LeftBoneChainForRealBone[i]];
									}
									else
										RealLeftSimulateBone = nullptr;
								}

								if (RealLeftSimulateBone != nullptr)
								{
									const FLKAnimVerletConstraint_FixedDistance RealFixedDistanceConstraint(&SimulateBones[CurBoneChain[i]], RealLeftSimulateBone, bStretchEachBone, StretchStrength, false, SideLengthMargin);
									FixedDistanceConstraints.Emplace(RealFixedDistanceConstraint);
								}
							}
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
							/// Hinge on the side edge shared by the triangles above and below this row.
							if (i > 0 && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending SideEdgeBendingConstraint(&SimulateBones[LeftBoneChain[i - 1]], &SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]],
																										bUseXPBDSolver, (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																										BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
								BendingConstraints.Emplace(SideEdgeBendingConstraint);
							}

							/// Hinge on the diagonal edge shared by the two triangles in this quad.
							if (i + 1 < LeftBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending BendingConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[LeftBoneChain[i + 1]],
																								bUseXPBDSolver, (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																								BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
								BendingConstraints.Emplace(BendingConstraint);
							}
						}

						if (bUseFlatBendingConstraint)
						{
							if (i + 1 < LeftBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_FlatBending FlatBendingConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], &SimulateBones[CurBoneChain[i + 1]], &SimulateBones[LeftBoneChain[i + 1]], 
																								bUseXPBDSolver, (bUseXPBDSolver ? FlatBendingComplianceAtRest : FlatBendingStiffnessAtRest), FlatBendingAlpha,
																								FlatBendingComplianceWhenFolded, FlatBendingStiffnessWhenFolded, FlatBendingMaxAngleRadians);
								FlatBendingConstraints.Emplace(FlatBendingConstraint);
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

						if (bUseCapsuleCollisionForChain)
						{
							if (i + 1 < RightBoneChain.Num())
							{
								const FLKAnimVerletBoneIndicatorTriangle IndicatorTriangle(FLKAnimVerletBoneIndicator(RightBoneChain[i], false), FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(RightBoneChain[i + 1], false));
								SimulateBoneTriangleIndicators.Emplace(IndicatorTriangle);

								if (i + 1 < CurBoneChain.Num())
								{
									const FLKAnimVerletBoneIndicatorTriangle IndicatorTriangle2(FLKAnimVerletBoneIndicator(CurBoneChain[i], false), FLKAnimVerletBoneIndicator(RightBoneChain[i + 1], false), FLKAnimVerletBoneIndicator(CurBoneChain[i + 1], false));
									SimulateBoneTriangleIndicators.Emplace(IndicatorTriangle2);
								}
							}
						}

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

							if (bPreserveSideLengthBetweenRealBones && SimulateBones[CurBoneChain[i]].bFakeBone == false && SimulateBones[RightBoneChain[i]].bSubDividedBone)
							{
								int32 RightIndexForRealBone = RightIndex;
								FLKAnimVerletBone* RealRightSimulateBone = &SimulateBones[RightBoneChain[i]];
								while (RealRightSimulateBone != nullptr && RealRightSimulateBone->bFakeBone)
								{
									if (BoneChainIndexes.IsValidIndex(++RightIndexForRealBone))
									{
										const TArray<int32>& RightBoneChainForRealBone = BoneChainIndexes[RightIndexForRealBone];
										RealRightSimulateBone = &SimulateBones[RightBoneChainForRealBone[i]];
									}
									else
										RealRightSimulateBone = nullptr;
								}

								if (RealRightSimulateBone != nullptr)
								{
									const FLKAnimVerletConstraint_FixedDistance RealFixedDistanceConstraint(&SimulateBones[CurBoneChain[i]], RealRightSimulateBone, bStretchEachBone, StretchStrength, false, SideLengthMargin);
									FixedDistanceConstraints.Emplace(RealFixedDistanceConstraint);
								}
							}
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
							/// Hinge on the side edge shared by the triangles above and below this row.
							if (i > 0 && i + 1 < RightBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending SideEdgeBendingConstraint(&SimulateBones[CurBoneChain[i - 1]], &SimulateBones[RightBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i + 1]],
																										bUseXPBDSolver, (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																										BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
								BendingConstraints.Emplace(SideEdgeBendingConstraint);
							}

							/// Hinge on the diagonal edge shared by the two triangles in this quad.
							if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_IsometricBending BendingConstraint(&SimulateBones[RightBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i + 1]], &SimulateBones[CurBoneChain[i + 1]],
																								bUseXPBDSolver, (bUseXPBDSolver ? BendingComplianceAtRest : BendingStiffnessAtRest),
																								BendingComplianceWhenFolded, BendingStiffnessWhenFolded, BendingMaxAngleRadians);
								BendingConstraints.Emplace(BendingConstraint);
							}
						}

						if (bUseFlatBendingConstraint)
						{
							if (i + 1 < RightBoneChain.Num() && i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_FlatBending FlatBendingConstraint(&SimulateBones[RightBoneChain[i]], &SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i + 1]], &SimulateBones[CurBoneChain[i + 1]],
																								bUseXPBDSolver, (bUseXPBDSolver ? FlatBendingComplianceAtRest : FlatBendingStiffnessAtRest), FlatBendingAlpha,
																								FlatBendingComplianceWhenFolded, FlatBendingStiffnessWhenFolded, FlatBendingMaxAngleRadians);
								FlatBendingConstraints.Emplace(FlatBendingConstraint);
							}
						}
					}
					RightCurIndex = RightIndex;
					++RightIndex;
				}
			}
		}
	}

	InitializeCustomDistanceConstraints(PoseContext, BoneContainer);

	if (bUseBroadphase)
	{
		InitializeBroadphase();
	}

	/// SelfCollision(Contact) constraints
	if (bUseSelfCollision)
	{
		FLKAnimVerletCollisionConstraintInput CollisionConstraintInput;
		{
			CollisionConstraintInput.Bones = &SimulateBones;
			CollisionConstraintInput.bUseBroadphase = bUseBroadphase;
			CollisionConstraintInput.bUseCapsuleCollisionForChain = bUseCapsuleCollisionForChain;
			CollisionConstraintInput.bSingleChain = bSingleChain;
			CollisionConstraintInput.SimulateBonePairIndicators = &SimulateBonePairIndicators;
			CollisionConstraintInput.SimulateBoneTriangleIndicators = &SimulateBoneTriangleIndicators;
			CollisionConstraintInput.BroadphaseContainer = &BroadphaseContainer;
			CollisionConstraintInput.bUseXPBDSolver = bUseXPBDSolver;
			CollisionConstraintInput.Compliance = Compliance;
		}
		const FLKAnimVerletConstraint_Self SelfCollisionConstraint(bUseTriangleSelfCollision, SelfCollisionAdditionalThickness, CollisionConstraintInput);
		SelfCollisionConstraints.Emplace(SelfCollisionConstraint);
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
					CollisionConstraintInput.bUseBroadphase = bUseBroadphase;
					CollisionConstraintInput.bUseCapsuleCollisionForChain = bUseCapsuleCollisionForChain;
					CollisionConstraintInput.bSingleChain = bSingleChain;
					CollisionConstraintInput.SimulateBonePairIndicators = &SimulateBonePairIndicators;
					CollisionConstraintInput.SimulateBoneTriangleIndicators = &SimulateBoneTriangleIndicators;
					CollisionConstraintInput.BroadphaseContainer = &BroadphaseContainer;
					CollisionConstraintInput.bUseXPBDSolver = bUseXPBDSolver;
					CollisionConstraintInput.Compliance = Compliance;
					CollisionConstraintInput.FrictionCoefficient = FrictionCoefficient;

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
}

void FLKAnimNode_AnimVerlet::RebuildSimulationForLOD(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
	struct FLKPreservedBoneState
	{
		FName BoneName = NAME_None;
		FName ParentBoneName = NAME_None;
		FVector Location = FVector::ZeroVector;
		FVector PrevLocation = FVector::ZeroVector;
		FQuat Rotation = FQuat::Identity;
		FQuat PrevRotation = FQuat::Identity;
		FVector MoveDelta = FVector::ZeroVector;
		FVector Velocity = FVector::ZeroVector;
		bool bFakeBone = false;
		bool bSubDividedBone = false;
		bool bTipBone = false;
		bool bSleep = false;
		float SleepTriggerElapsedTime = 0.0f;
		bool bUsed = false;
	};

	TArray<FLKPreservedBoneState, TInlineAllocator<64>> PreservedStates;
	TMultiMap<FName, int32, TInlineSetAllocator<64>> PreservedStateIndexesByBoneName;
	PreservedStates.Reserve(SimulateBones.Num());
	for (const FLKAnimVerletBone& Bone : SimulateBones)
	{
		if (Bone.Location.ContainsNaN() || Bone.PrevLocation.ContainsNaN() || Bone.Rotation.ContainsNaN() || Bone.PrevRotation.ContainsNaN())
			continue;

		const int32 StateIndex = PreservedStates.Emplace();
		FLKPreservedBoneState& State = PreservedStates[StateIndex];
		{
			State.BoneName = Bone.BoneReference.BoneName;
			State.ParentBoneName = (Bone.HasParentBone() && SimulateBones.IsValidIndex(Bone.ParentVerletBoneIndex) ? SimulateBones[Bone.ParentVerletBoneIndex].BoneReference.BoneName : NAME_None);
			State.Location = Bone.Location;
			State.PrevLocation = Bone.PrevLocation;
			State.Rotation = Bone.Rotation;
			State.PrevRotation = Bone.PrevRotation;
			State.MoveDelta = Bone.MoveDelta.ContainsNaN() ? FVector::ZeroVector : Bone.MoveDelta;
			State.Velocity = Bone.Velocity.ContainsNaN() ? FVector::ZeroVector : Bone.Velocity;
			State.bFakeBone = Bone.bFakeBone;
			State.bSubDividedBone = Bone.bSubDividedBone;
			State.bTipBone = Bone.bTipBone;
			State.bSleep = Bone.bSleep;
			State.SleepTriggerElapsedTime = Bone.SleepTriggerElapsedTime;
		}
		PreservedStateIndexesByBoneName.Emplace(State.BoneName, StateIndex);
	}

	ClearSimulateBones();
	InitializeSimulateBones(PoseContext, BoneContainer);

	for (FLKAnimVerletBone& Bone : SimulateBones)
	{
		const FName ParentBoneName = (Bone.HasParentBone() && SimulateBones.IsValidIndex(Bone.ParentVerletBoneIndex) ? SimulateBones[Bone.ParentVerletBoneIndex].BoneReference.BoneName : NAME_None);
		FLKPreservedBoneState* MatchingState = nullptr;
		for (auto StateIt = PreservedStateIndexesByBoneName.CreateKeyIterator(Bone.BoneReference.BoneName); StateIt; ++StateIt)
		{
			FLKPreservedBoneState& State = PreservedStates[StateIt.Value()];
			if (State.bUsed == false && State.ParentBoneName == ParentBoneName && State.bFakeBone == Bone.bFakeBone
				&& State.bSubDividedBone == Bone.bSubDividedBone && State.bTipBone == Bone.bTipBone)
			{
				MatchingState = &State;
				break;
			}
		}
		if (MatchingState == nullptr)
			continue;

		Bone.Location = MatchingState->Location;
		Bone.PrevLocation = MatchingState->PrevLocation;
		Bone.Rotation = MatchingState->Rotation;
		Bone.PrevRotation = MatchingState->PrevRotation;
		Bone.MoveDelta = MatchingState->MoveDelta;
		Bone.Velocity = MatchingState->Velocity;
		Bone.bSleep = MatchingState->bSleep;
		Bone.SleepTriggerElapsedTime = MatchingState->SleepTriggerElapsedTime;
		MatchingState->bUsed = true;
	}
}

void FLKAnimNode_AnimVerlet::InitializeCustomDistanceConstraints(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
	if (CustomDistanceConstraints.IsEmpty())
		return;

	/// Constraint endpoints store raw pointers. Reserving the maximum possible number of anchors keeps those pointers stable while the list is built.
	CustomDistanceConstraintBones.Reserve(CustomDistanceConstraints.Num() * 2);

	const double Compliance = static_cast<double>(1.0 / InvCompliance);
	const double SolverStiffness = bUseXPBDSolver ? Compliance : static_cast<double>(Stiffness);
	for (const FLKAnimVerletCustomDistanceConstraintSetting& CurConstraintSetting : CustomDistanceConstraints)
	{
		FLKAnimVerletBone* BoneA = FindOrAddCustomDistanceConstraintBone(CurConstraintSetting.BoneA, PoseContext, BoneContainer);
		FLKAnimVerletBone* BoneB = FindOrAddCustomDistanceConstraintBone(CurConstraintSetting.BoneB, PoseContext, BoneContainer);
		if (BoneA == nullptr || BoneB == nullptr || BoneA == BoneB)
			continue;

		const FLKAnimVerletConstraint_Distance NewDistanceConstraint(BoneA, BoneB, bUseXPBDSolver, SolverStiffness, bStretchEachBone, StretchStrength,
																	 CurConstraintSetting.MinDistance, CurConstraintSetting.MaxDistance);
		DistanceConstraints.Emplace(NewDistanceConstraint);
	}
}

FLKAnimVerletBone* FLKAnimNode_AnimVerlet::FindOrAddCustomDistanceConstraintBone(const FBoneReference& BoneReference, FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
	/// Subdivided particles inherit the real bone reference, so prefer the real skeletal particle when both share the same FBoneReference.
	int32 SimulateBoneIndex = SimulateBones.IndexOfByPredicate([&BoneReference] (const FLKAnimVerletBone& SimulateBone) {
		return SimulateBone.bFakeBone == false && SimulateBone.BoneReference == BoneReference;
	});

	if (SimulateBoneIndex == INDEX_NONE)
		SimulateBoneIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(BoneReference));
	if (SimulateBoneIndex != INDEX_NONE)
		return &SimulateBones[SimulateBoneIndex];

	const int32 ExistingAnchorIndex = CustomDistanceConstraintBones.IndexOfByKey(FLKAnimVerletBoneKey(BoneReference));
	if (ExistingAnchorIndex != INDEX_NONE)
		return &CustomDistanceConstraintBones[ExistingAnchorIndex];

	FBoneReference InitializedBoneReference = BoneReference;
	InitializedBoneReference.Initialize(BoneContainer);
	const FCompactPoseBoneIndex PoseBoneIndex = InitializedBoneReference.GetCompactPoseIndex(BoneContainer);
	if (PoseBoneIndex == INDEX_NONE)
		return nullptr;

	FLKAnimVerletBone NewAnchorBone;
	{
		NewAnchorBone.BoneReference = InitializedBoneReference;
		NewAnchorBone.bPinned = true;
		NewAnchorBone.InvMass = 0.0f;
		NewAnchorBone.bUseXPBDSolver = bUseXPBDSolver;
		NewAnchorBone.InitializeTransform(PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex));
	}
	return &CustomDistanceConstraintBones.Emplace_GetRef(MoveTemp(NewAnchorBone));
}

void FLKAnimNode_AnimVerlet::InitializeBroadphase()
{
	verify(bUseBroadphase);

	if (bUseCapsuleCollisionForChain)
	{
		if (IsSingleChain())
		{
			BroadphaseContainer.InitializeFromPairs(&SimulateBones, &SimulateBonePairIndicators, MaxThickness);
		}
		else
		{
			BroadphaseContainer.InitializeFromTriangles(&SimulateBones, &SimulateBoneTriangleIndicators, MaxThickness);
		}
	}
	else
	{
		BroadphaseContainer.InitializeFromBones(&SimulateBones, MaxThickness);
	}
}

void FLKAnimNode_AnimVerlet::InitializeLocalCollisionConstraints(const FBoneContainer& BoneContainer)
{
	SimulatingCollisionShapes.SphereCollisionShapes = SphereCollisionShapes;
	SimulatingCollisionShapes.CapsuleCollisionShapes = CapsuleCollisionShapes;
	SimulatingCollisionShapes.BoxCollisionShapes = BoxCollisionShapes;
	SimulatingCollisionShapes.PlaneCollisionShapes = PlaneCollisionShapes;
	if (CollisionDataAsset != nullptr)
		CollisionDataAsset->ConvertToShape(OUT SimulatingCollisionShapes);
	if (CollisionPhysicsAsset != nullptr)
		ConvertPhysicsAssetToShape(OUT SimulatingCollisionShapes, *CollisionPhysicsAsset, &BoneContainer);

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
		NewSimulateBone.bConstrainConeAngleFromParent = bConstrainConeAngleFromParent;
		NewSimulateBone.ConeAngleOffset = ConeAngleOffset;
		NewSimulateBone.InvMass = 1.0f / FMath::Max(BoneSetting.Mass, 0.01f);
		NewSimulateBone.Thickness = Thickness;
		if (FoundBoneUnitSettingNullable != nullptr)
		{
			if (FoundBoneUnitSettingNullable->bOverrideConstrainConeAngleFromParent)
				NewSimulateBone.bConstrainConeAngleFromParent = FoundBoneUnitSettingNullable->bConstrainConeAngleFromParent;
			if (FoundBoneUnitSettingNullable->bOverrideConeAngle)
				NewSimulateBone.ConeAngleConstraint = FoundBoneUnitSettingNullable->ConeAngle;
			if (FoundBoneUnitSettingNullable->bOverrideConeAngleOffset)
				NewSimulateBone.ConeAngleOffset = FoundBoneUnitSettingNullable->ConeAngleOffset;
			if (FoundBoneUnitSettingNullable->bOverrideMass)
				NewSimulateBone.InvMass = 1.0f / FMath::Max(FoundBoneUnitSettingNullable->Mass, 0.01f);
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

		/// The parent bone's unit setting controls the segment from that parent to this bone.
		bool bSubDivideCurrentSegment = bSubDivideBones;
		uint8 NumSubDividedBoneForCurrentSegment = NumSubDividedBone;
		if (ParentSimulateBoneIndex != INDEX_NONE)
		{
			const FLKAnimVerletBoneUnitSetting* ParentBoneUnitSettingNullable = BoneSetting.BoneUnitSettingOverride.FindByKey(SimulateBones[ParentSimulateBoneIndex].BoneReference);
			if (ParentBoneUnitSettingNullable != nullptr && ParentBoneUnitSettingNullable->bOverrideSubDivideBones)
			{
				bSubDivideCurrentSegment = ParentBoneUnitSettingNullable->bSubDivideBones;
				NumSubDividedBoneForCurrentSegment = ParentBoneUnitSettingNullable->NumSubDividedBone;
			}
		}

		/// Make subdivided simulate bones
		if (bSubDivideCurrentSegment && NumSubDividedBoneForCurrentSegment > 0 && ParentSimulateBoneIndex != INDEX_NONE)
		{
			int32 SubDividedParentSimulateBoneIndex = ParentSimulateBoneIndex;
			const FLKAnimVerletBone& ParentSimulateBone = SimulateBones[ParentSimulateBoneIndex];
			FVector DirToParent = FVector::ZeroVector;
			float DirToParentSize = 0.0f;
			(ParentSimulateBone.PoseLocation - NewSimulateBone.PoseLocation).ToDirectionAndLength(OUT DirToParent, OUT DirToParentSize);
			const float SubDivideSize = DirToParentSize / static_cast<float>(NumSubDividedBoneForCurrentSegment + 1);

			for (uint8 SubDivideCount = NumSubDividedBoneForCurrentSegment; SubDivideCount > 0; --SubDivideCount)
			{
				FLKAnimVerletBone SubDividedSimulateBone = NewSimulateBone;
				SubDividedSimulateBone.ParentVerletBoneIndex = SubDividedParentSimulateBoneIndex;
				SubDividedSimulateBone.bFakeBone = true;
				SubDividedSimulateBone.bSubDividedBone = true;
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

		MaxThickness = FMath::Max(MaxThickness, NewSimulateBone.Thickness);
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
			FakeSimulateBone.bConstrainConeAngleFromParent = bConstrainConeAngleFromParent;
			FakeSimulateBone.ConeAngleOffset = ConeAngleOffset;
			FakeSimulateBone.InvMass = 1.0f / FMath::Max(BoneSetting.Mass, 0.01f);
			FakeSimulateBone.Thickness = Thickness;
			if (FoundBoneUnitSettingNullable != nullptr)
			{
				if (FoundBoneUnitSettingNullable->bOverrideConstrainConeAngleFromParent)
					FakeSimulateBone.bConstrainConeAngleFromParent = FoundBoneUnitSettingNullable->bConstrainConeAngleFromParent;
				if (FoundBoneUnitSettingNullable->bOverrideConeAngle)
					FakeSimulateBone.ConeAngleConstraint = FoundBoneUnitSettingNullable->ConeAngle;
				if (FoundBoneUnitSettingNullable->bOverrideConeAngleOffset)
					FakeSimulateBone.ConeAngleOffset = FoundBoneUnitSettingNullable->ConeAngleOffset;
				if (FoundBoneUnitSettingNullable->bOverrideMass)
					FakeSimulateBone.InvMass = 1.0f / FMath::Max(FoundBoneUnitSettingNullable->Mass, 0.01f);
				if (FoundBoneUnitSettingNullable->bOverrideThickness)
					FakeSimulateBone.Thickness = FoundBoneUnitSettingNullable->Thickness;
				FakeSimulateBone.bOverrideToUseSphereCollisionForChain = FoundBoneUnitSettingNullable->bOverrideToUseSphereCollisionForChain;

				if (FoundBoneUnitSettingNullable->bLockBone)
				{
					FakeSimulateBone.bPinned = true;	///Reserve
					FakeSimulateBone.PinMargin = FoundBoneUnitSettingNullable->LockMargin;
				}
			}
			/// Keep the legacy fake-tip behavior unless this terminal bone explicitly overrides subdivision.
			bool bSubDivideFakeTipSegment = false;
			uint8 NumSubDividedBoneForFakeTipSegment = 0;
			if (FoundBoneUnitSettingNullable != nullptr && FoundBoneUnitSettingNullable->bOverrideSubDivideBones)
			{
				bSubDivideFakeTipSegment = FoundBoneUnitSettingNullable->bSubDivideBones;
				NumSubDividedBoneForFakeTipSegment = FoundBoneUnitSettingNullable->NumSubDividedBone;
			}

			const int32 NumFakeTipSegments = bSubDivideFakeTipSegment ? static_cast<int32>(NumSubDividedBoneForFakeTipSegment) + 1 : 1;
			const float FakeTipSegmentLength = FakeTipBoneLength / static_cast<float>(NumFakeTipSegments);
			int32 FakeTipParentSimulateBoneIndex = CurSimulateBoneIndex;
			for (int32 SubDivideCount = 0; SubDivideCount + 1 < NumFakeTipSegments; ++SubDivideCount)
			{
				FLKAnimVerletBone SubDividedSimulateBone = FakeSimulateBone;
				SubDividedSimulateBone.bTipBone = false;
				SubDividedSimulateBone.bSubDividedBone = true;
				SubDividedSimulateBone.ParentVerletBoneIndex = FakeTipParentSimulateBoneIndex;
				SubDividedSimulateBone.FakeBoneLengthFromParent = FakeTipSegmentLength;

				FTransform SubDividedBoneT = FTransform::Identity;
				MakeFakeBoneTransform(OUT SubDividedBoneT, FakeTipParentSimulateBoneIndex, FakeTipSegmentLength);
				SubDividedSimulateBone.InitializeTransform(SubDividedBoneT);

				const int32 SubDividedSimulateBoneIndex = SimulateBones.Emplace(SubDividedSimulateBone);
				FLKAnimVerletBone& CurParentVerletBone = SimulateBones[FakeTipParentSimulateBoneIndex];
				bool bAlreadyInSet = false;
				CurParentVerletBone.ChildVerletBoneIndexes.Add(SubDividedSimulateBoneIndex, &bAlreadyInSet);

				const int32 AddedSubDividedBoneChainLength = BoneChainIndexes[RootSimulateBoneIndex].Add(SubDividedSimulateBoneIndex) + 1;
				MaxBoneChainLength = FMath::Max(MaxBoneChainLength, AddedSubDividedBoneChainLength);
				FakeTipParentSimulateBoneIndex = SubDividedSimulateBoneIndex;
			}

			FakeSimulateBone.ParentVerletBoneIndex = FakeTipParentSimulateBoneIndex;
			FakeSimulateBone.FakeBoneLengthFromParent = FakeTipSegmentLength;

			FTransform FakeBoneT = FTransform::Identity;
			MakeFakeBoneTransform(OUT FakeBoneT, FakeTipParentSimulateBoneIndex, FakeTipSegmentLength);
			FakeSimulateBone.InitializeTransform(FakeBoneT);

			MaxThickness = FMath::Max(MaxThickness, FakeSimulateBone.Thickness);
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

void FLKAnimNode_AnimVerlet::MakeFakeBoneTransform(OUT FTransform& OutTransform, int32 ParentSimulateBoneIndex, float InFakeBoneLength) const
{
	verify(SimulateBones.IsValidIndex(ParentSimulateBoneIndex));
	const FLKAnimVerletBone& ParentBone = SimulateBones[ParentSimulateBoneIndex];

	const FVector DirToParent = ParentBone.HasParentBone() ? (SimulateBones[ParentBone.ParentVerletBoneIndex].PoseLocation - ParentBone.PoseLocation).GetSafeNormal() : ParentBone.PoseRotation.GetUpVector();
	OutTransform = FTransform(ParentBone.Rotation, ParentBone.PoseLocation - DirToParent * InFakeBoneLength);
}

void FLKAnimNode_AnimVerlet::UpdateDeltaTime(float InDeltaTime, float InTimeDilation)
{
	const float PositiveFrameDeltaTime = FMath::Max(InDeltaTime, 0.0f);
	const float ClampedFrameDeltaTime = FMath::Clamp(InDeltaTime, MinDeltaTime, MaxDeltaTime);
	const bool bUseFixedDeltaTime = FMath::IsNearlyZero(FixedDeltaTime, KINDA_SMALL_NUMBER) == false;
	NumPendingSimulationSteps = 0;

	if (bPause)
	{
		FixedStepAccumulator = 0.0f;
		DeltaTime = bUseFixedDeltaTime ? FMath::Clamp(FixedDeltaTime * InTimeDilation, MinDeltaTime, MaxDeltaTime) : ClampedFrameDeltaTime;
		return;
	}

	if (bUseFixedDeltaTime == false)
	{
		FixedStepAccumulator = 0.0f;
		DeltaTime = ClampedFrameDeltaTime;
		NumPendingSimulationSteps = DeltaTime > 0.0f ? 1 : 0;
		return;
	}

	DeltaTime = FMath::Clamp(FixedDeltaTime * InTimeDilation, MinDeltaTime, MaxDeltaTime);
	if (bApplyDeltaTimeCorrection == false)
	{
		FixedStepAccumulator = 0.0f;
		NumPendingSimulationSteps = DeltaTime > 0.0f ? 1 : 0;
		return;
	}

	const int32 ClampedMaxSubStep = FMath::Max(MaxSubStep, 1);
	const float TargetFrameRate = FMath::Max(DeltaTimeCorrectionTargetFrameRate, 1.0f);
	FixedStepAccumulator = FMath::Min(FixedStepAccumulator + PositiveFrameDeltaTime * TargetFrameRate, static_cast<float>(ClampedMaxSubStep));
	NumPendingSimulationSteps = FMath::Min(FMath::FloorToInt(FixedStepAccumulator + UE_SMALL_NUMBER), ClampedMaxSubStep);
}

void FLKAnimNode_AnimVerlet::PrepareSimulation(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FTransform& ComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PrepareSimulation);
#endif

	for (int32 SimulateBoneIndex = 0; SimulateBoneIndex < SimulateBones.Num(); ++SimulateBoneIndex)
	{
		FLKAnimVerletBone& CurSimulateBone = SimulateBones[SimulateBoneIndex];
		FTransform CurBonePoseT = FTransform::Identity;
		if (CurSimulateBone.bFakeBone && CurSimulateBone.HasBoneSetup() == false)
		{
			/// Virtual TipBone case
			MakeFakeBoneTransform(OUT CurBonePoseT, CurSimulateBone.ParentVerletBoneIndex, CurSimulateBone.FakeBoneLengthFromParent);
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

	const FQuat GravityAlignmentRotation = CalculateGravityAlignmentRotation(ComponentTransform);
	for (FLKAnimVerletBone& CurSimulateBone : SimulateBones)
	{
		CurSimulateBone.GravityAlignedPoseDiff = GravityAlignmentRotation.RotateVector(CurSimulateBone.PoseLocation - CurSimulateBone.PrevPoseLocation);
		if (CurSimulateBone.HasParentBone())
		{
			const FLKAnimVerletBone& ParentSimulateBone = SimulateBones[CurSimulateBone.ParentVerletBoneIndex];
			const FVector PoseVectorFromParent = CurSimulateBone.PoseLocation - ParentSimulateBone.PoseLocation;
			const FVector GravityAlignedPoseVectorFromParent = GravityAlignmentRotation.RotateVector(PoseVectorFromParent);
			CurSimulateBone.GravityAlignedPoseLocation = ParentSimulateBone.GravityAlignedPoseLocation + GravityAlignedPoseVectorFromParent;
			CurSimulateBone.GravityAlignedPoseDirFromParent = GravityAlignedPoseVectorFromParent.GetSafeNormal();
		}
		else
		{
			/// Keep each simulated chain anchored at its original animation-pose root.
			CurSimulateBone.GravityAlignedPoseLocation = CurSimulateBone.PoseLocation;
			CurSimulateBone.GravityAlignedPoseDirFromParent = FVector::ZeroVector;
		}
	}

	for (int32 ExcludedBoneIndex = 0; ExcludedBoneIndex < ExcludedBones.Num(); ++ExcludedBoneIndex)
	{
		FLKAnimVerletExcludedBone& CurExcludedBone = ExcludedBones[ExcludedBoneIndex];
		const FCompactPoseBoneIndex PoseBoneIndex = CurExcludedBone.BoneReference.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		const FTransform CurBonePoseT = PoseBoneIndex != INDEX_NONE ? PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex) : FTransform::Identity;
		CurExcludedBone.PrepareSimulation(CurBonePoseT);
	}

	for (FLKAnimVerletBone& CurAnchorBone : CustomDistanceConstraintBones)
	{
		const FCompactPoseBoneIndex PoseBoneIndex = CurAnchorBone.BoneReference.GetCompactPoseIndex(BoneContainer);
		if (PoseBoneIndex == INDEX_NONE)
			continue;

		const FTransform CurBonePoseT = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
		CurAnchorBone.PrepareSimulation(CurBonePoseT, FVector::ZeroVector);
		CurAnchorBone.Location = CurAnchorBone.PoseLocation;
		CurAnchorBone.PrevLocation = CurAnchorBone.PoseLocation;
		CurAnchorBone.Rotation = CurAnchorBone.PoseRotation;
		CurAnchorBone.PrevRotation = CurAnchorBone.PoseRotation;
	}

	PrepareLocalCollisionConstraints(PoseContext, BoneContainer, ComponentTransform);
}

FQuat FLKAnimNode_AnimVerlet::CalculateGravityAlignmentRotation(const FTransform& ComponentTransform) const
{
	const FVector ReferenceDirection = AnimationPoseReferenceDirection.GetSafeNormal();
	if (ReferenceDirection.IsNearlyZero(KINDA_SMALL_NUMBER))
		return FQuat::Identity;

	const FVector GravityVector = bGravityInWorldSpace ? ComponentTransform.InverseTransformVectorNoScale(Gravity) : Gravity;
	if (GravityVector.IsNearlyZero(KINDA_SMALL_NUMBER))
		return FQuat::Identity;

	const FVector GravityDirection = GravityVector.GetSafeNormal();
	return FQuat::FindBetweenNormals(ReferenceDirection, GravityDirection).GetNormalized();
}

void FLKAnimNode_AnimVerlet::PrepareLocalCollisionConstraints(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FTransform& ComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PrepareLocalCollisionConstraints);
#endif

	const bool bSingleChain = IsSingleChain();
	const double Compliance = static_cast<double>(1.0 / InvCompliance);
	if (bLocalColliderDirty)
	{
		SimulatingCollisionShapes.ResetCollisionShapeList();
		InitializeLocalCollisionConstraints(BoneContainer);
	}

	FLKAnimVerletCollisionConstraintInput CollisionConstraintInput;
	{
		CollisionConstraintInput.Bones = &SimulateBones;
		CollisionConstraintInput.bUseBroadphase = bUseBroadphase;
		CollisionConstraintInput.bUseCapsuleCollisionForChain = bUseCapsuleCollisionForChain;
		CollisionConstraintInput.bSingleChain = bSingleChain;
		CollisionConstraintInput.SimulateBonePairIndicators = &SimulateBonePairIndicators;
		CollisionConstraintInput.SimulateBoneTriangleIndicators = &SimulateBoneTriangleIndicators;
		CollisionConstraintInput.BroadphaseContainer = &BroadphaseContainer;
		CollisionConstraintInput.bUseXPBDSolver = bUseXPBDSolver;
		CollisionConstraintInput.Compliance = Compliance;
		CollisionConstraintInput.FrictionCoefficient = FrictionCoefficient;
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
			SphereCollisionConstraints.Emplace(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
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
				SphereCollisionConstraints.Emplace(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
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
			SphereCollisionConstraints.Emplace(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
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
				SphereCollisionConstraints.Emplace(BoneLocation, CurShapeSphere.Radius, CollisionConstraintInput);
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
			CapsuleCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeCapsule.Radius,
												CurShapeCapsule.HalfHeight, CollisionConstraintInput);
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
				CapsuleCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeCapsule.Radius,
													CurShapeCapsule.HalfHeight, CollisionConstraintInput);
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
			CapsuleCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeCapsule.Radius,
												CurShapeCapsule.HalfHeight, CollisionConstraintInput);
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
				CapsuleCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeCapsule.Radius,
													CurShapeCapsule.HalfHeight, CollisionConstraintInput);
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

			BoxCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
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

				BoxCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
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

			BoxCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
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

				BoxCollisionConstraints.Emplace(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, CollisionConstraintInput);
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
			PlaneCollisionConstraints.Emplace(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
											  CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
											  CollisionConstraintInput);
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

				PlaneCollisionConstraints.Emplace(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
												  CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
												  CollisionConstraintInput);
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
			PlaneCollisionConstraints.Emplace(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
											  CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
											  CollisionConstraintInput);
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
				PlaneCollisionConstraints.Emplace(BoneLocation, BoneRotation.GetUpVector(), BoneRotation,
												  CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector,
												  CollisionConstraintInput);
			}
		}
	}
}

void FLKAnimNode_AnimVerlet::ConvertPhysicsAssetToShape(OUT FLKAnimVerletCollisionShapeList& OutShapeList, const UPhysicsAsset& InPhysicsAsset, const FBoneContainer* BoneContainerNullable) const
{
	for (const TObjectPtr<USkeletalBodySetup>& BodySetup : InPhysicsAsset.SkeletalBodySetups)
	{
		FBoneReference AttachedBone = BodySetup->BoneName;
		if (BoneContainerNullable != nullptr)
		{
			AttachedBone.Initialize(*BoneContainerNullable);
			if (AttachedBone.IsValidToEvaluate(*BoneContainerNullable) == false)
				continue;
		}

		const FKAggregateGeom& CurGeometry = BodySetup->AggGeom;
		for (const FKSphereElem& SphereElem : CurGeometry.SphereElems)
		{
			FLKAnimVerletCollisionSphere& SphereShape = OutShapeList.SphereCollisionShapes.Emplace_GetRef();
			{
				FLKAnimVerletCollisionDataSphere SphereData;
				SphereData.ConvertFromPhysicsAsset(SphereElem, AttachedBone.BoneName);
				SphereData.ConvertToShape(SphereShape);
			}
		}
		for (const FKSphylElem& CapsuleElem : CurGeometry.SphylElems)
		{
			FLKAnimVerletCollisionCapsule& CapsuleShape = OutShapeList.CapsuleCollisionShapes.Emplace_GetRef();
			{
				FLKAnimVerletCollisionDataCapsule CapsuleData;
				CapsuleData.ConvertFromPhysicsAsset(CapsuleElem, AttachedBone.BoneName);
				CapsuleData.ConvertToShape(CapsuleShape);
			}
		}
		for (const FKBoxElem& BoxElem : CurGeometry.BoxElems)
		{
			FLKAnimVerletCollisionBox& BoxShape = OutShapeList.BoxCollisionShapes.Emplace_GetRef();
			{
				FLKAnimVerletCollisionDataBox BoxData;
				BoxData.ConvertFromPhysicsAsset(BoxElem, AttachedBone.BoneName);
				BoxData.ConvertToShape(BoxShape);
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

	const bool bComponentInertiaApplied = PreUpdateBones(World, InDeltaTime, ComponentTransform, PrevComponentTransform);

	if (bUseBroadphase)
		UpdateBroadphase(World, InDeltaTime, ComponentTransform);

	/// Solve
	SolveConstraints(InDeltaTime);

	if (bComponentInertiaApplied)
		ApplyComponentInertiaTangentialDamping(InDeltaTime);
	
	if (bUseSleep)
		UpdateSleep(InDeltaTime);

	PostUpdateBones(InDeltaTime);
}

bool FLKAnimNode_AnimVerlet::PreUpdateBones(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_PreUpdateBones);
#endif

	extern ENGINE_API float GAverageFPS;
	const bool bUseCorrectedFixedStep = FMath::IsNearlyZero(FixedDeltaTime, KINDA_SMALL_NUMBER) == false && bApplyDeltaTimeCorrection;
	const float CorrectionFrameRate = bUseCorrectedFixedStep ? FMath::Max(DeltaTimeCorrectionTargetFrameRate, 1.0f) : FMath::Clamp(GAverageFPS, LKG_MINFPS, LKG_MAXFPS);
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

			const bool bIgnoreMoveInertia = bIgnoreSuddenMoveInertia && MoveDiffDist > FMath::Max(MoveInertiaIgnoreThreshold, 0.0f);
			if (bIgnoreMoveInertia)
			{
				VerletUpdateParam.ComponentMoveDiff = FVector::ZeroVector;
			}
			else
			{
				MoveDiffDist = bClampMoveInertia ? FMath::Clamp(MoveDiffDist, 0.0f, MoveInertiaClampMaxDistance) : MoveDiffDist;
				VerletUpdateParam.ComponentMoveDiff = MoveDiffDir * MoveDiffDist * MoveInertiaScale;
			}
		}

		/// Clamp Rotation Intertia
		{
			const FQuat PrevComponentRotation = PrevComponentTransform.GetRotation();
			VerletUpdateParam.ComponentRotDiff = ComponentTransform.InverseTransformRotation(PrevComponentRotation);
			FVector RotDiffAxis = FVector::ZeroVector;
			float RotDiffAngle = 0.0f;
			VerletUpdateParam.ComponentRotDiff.ToAxisAndAngle(OUT RotDiffAxis, OUT RotDiffAngle);

			float DiffAngeDegrees = FMath::RadiansToDegrees(RotDiffAngle);
			const bool bIgnoreRotationInertia = bIgnoreSuddenRotationInertia && FMath::Abs(DiffAngeDegrees) > FMath::Max(RotationInertiaIgnoreDegrees, 0.0f);
			if (bIgnoreRotationInertia)
			{
				VerletUpdateParam.ComponentRotDiff = FQuat::Identity;
			}
			else
			{
				if (bClampRotationInertia && FMath::Abs(DiffAngeDegrees) > RotationInertiaClampDegrees)
					DiffAngeDegrees = FMath::Sign(DiffAngeDegrees) * RotationInertiaClampDegrees;

				VerletUpdateParam.ComponentRotDiff = FQuat(RotDiffAxis, FMath::DegreesToRadians(DiffAngeDegrees * RotationInertiaScale)).GetNormalized();
			}
		}

		VerletUpdateParam.bUseSquaredDeltaTime = bUseSquaredDeltaTime;
		VerletUpdateParam.StretchForce = StretchForce;
		VerletUpdateParam.bAlignStretchForceToGravity = bAlignStretchForceToGravity;
		VerletUpdateParam.SideStraightenForce = SideStraightenForce;
		VerletUpdateParam.ShapeMemoryForce = ShapeMemoryForce;
		VerletUpdateParam.bAlignShapeMemoryForceToGravity = bAlignShapeMemoryForceToGravity;
		VerletUpdateParam.Gravity = (bGravityInWorldSpace == false || Gravity.IsNearlyZero(KINDA_SMALL_NUMBER)) ? Gravity : ComponentTransform.InverseTransformVector(Gravity);
		VerletUpdateParam.ExternalForce = (bExternalForceInWorldSpace == false || ExternalForce.IsNearlyZero(KINDA_SMALL_NUMBER)) ? ExternalForce : ComponentTransform.InverseTransformVector(ExternalForce);

		const bool bUseRandomWind = (RandomWindDirection.IsNearlyZero(KINDA_SMALL_NUMBER) == false);
		VerletUpdateParam.RandomWind.RandomForceDirection = (bRandomWindDirectionInWorldSpace == false && bUseRandomWind) ? RandomWindDirection : ComponentTransform.InverseTransformVector(RandomWindDirection);
		VerletUpdateParam.RandomWind.RandomForceSizeMin = RandomWindSizeMin;
		VerletUpdateParam.RandomWind.RandomForceSizeMax = RandomWindSizeMax;
		VerletUpdateParam.RandomWind.bRandomForceDirectionInWorldSpace = bRandomWindDirectionInWorldSpace;
		for (const FLKAnimVerletRandomForceSetting& CurWind : AdditionalRandomWinds)
		{
			FLKAnimVerletRandomForceSetting& NewWind = VerletUpdateParam.AdditionalRandomWinds.Emplace_GetRef();
			NewWind.RandomForceDirection = (CurWind.bRandomForceDirectionInWorldSpace == false) ? CurWind.RandomForceDirection : ComponentTransform.InverseTransformVector(CurWind.RandomForceDirection);
			NewWind.RandomForceSizeMin = CurWind.RandomForceSizeMin;
			NewWind.RandomForceSizeMax = CurWind.RandomForceSizeMax;
			NewWind.bRandomForceDirectionInWorldSpace = CurWind.bRandomForceDirectionInWorldSpace;
		}
		VerletUpdateParam.Damping = bApplyDampingCorrection ? FMath::Clamp(FMath::Pow(Damping, (DampingCorrectionTargetFrameRate / CorrectionFrameRate)), 0.0f, 1.0f) : Damping;
	}
	const bool bComponentFrameMoved = VerletUpdateParam.ComponentMoveDiff.IsNearlyZero(KINDA_SMALL_NUMBER) == false || VerletUpdateParam.ComponentRotDiff.Equals(FQuat::Identity, KINDA_SMALL_NUMBER) == false;

	/// Simulate each bones	
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		/// A moving component is an external kinematic input. Keeping a bone asleep here would
		/// discard the rebased displacement in UpdateSleep and make slow component motion vanish.
		if (bUseSleep && bComponentFrameMoved)
			CurVerletBone.WakeUp();

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
			CurVerletBone.Location += WindVelocity * (InDeltaTime * CurVerletBone.InvMass);
		}

		/// Adjust animation pose transform
		if (bIgnoreAnimationPose == false && CurVerletBone.HasParentBone())
		{
			FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurVerletBone.ParentVerletBoneIndex];
			///const float AnimPoseDeltaInertiaScaled = bApplyAnimationPoseInertiaCorrection ? (AnimationPoseDeltaInertia * AnimationPoseDeltaInertiaScale * AnimationPoseInertiaTargetFrameRate / CorrectionFrameRate) : AnimationPoseDeltaInertia * AnimationPoseDeltaInertiaScale;
			const float AnimPoseDeltaInertiaScaled = AnimationPoseDeltaInertia * AnimationPoseDeltaInertiaScale;
			const float TargetAnimationPoseInertia = bApplyAnimationPoseInertiaCorrection ? (AnimationPoseInertia * AnimationPoseInertiaTargetFrameRate / CorrectionFrameRate) : AnimationPoseInertia;
			CurVerletBone.AdjustPoseTransform(InDeltaTime, ParentVerletBone.Location, ParentVerletBone.PoseLocation, ParentVerletBone.GravityAlignedPoseLocation,
											 bAlignAnimationPoseToGravity, TargetAnimationPoseInertia, AnimPoseDeltaInertiaScaled,
											 bClampAnimationPoseDeltaInertia, AnimationPoseDeltaInertiaClampMax);
		}
	}

	return bComponentFrameMoved;
}

void FLKAnimNode_AnimVerlet::UpdateBroadphase(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform)
{
#if LK_ENABLE_STAT
	SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_UpdateBroadphase);
#endif

	BroadphaseContainer.Update();
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
		const bool bInitialUpdate = (Iteration == 0);
		const bool bFinalizeUpdate = (Iteration == SolveIteration - 1);

		/// Simulate each constraints
		/// Solve order is important. Make a heuristic order by constraint priority.
		/*for (int32 i = 0; i < Constraints.Num(); ++i)
		{
			verify(Constraints[i] != nullptr);
			Constraints[i]->Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}*/
		for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, false);
		}
		for (int32 i = 0; i < DistanceConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_DistanceConstraints);
		#endif
			DistanceConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BendingConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BendingConstraints);
		#endif
			BendingConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BendingConstraints_1D.Num(); ++i)
		{
			#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BendingConstraints_1D);
			#endif
			BendingConstraints_1D[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < FlatBendingConstraints.Num(); ++i)
		{
			#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FlatBendingConstraints);
			#endif
			FlatBendingConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < StraightenConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_StraightenConstraints);
		#endif
			StraightenConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BallSocketConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BallSocketConstraints);
		#endif
			BallSocketConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}

		///-------------------------------------------------------------------------------------
		/// Local collision constratins
		for (int32 i = 0; i < PlaneCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PlaneCollisionConstraints);
		#endif
			PlaneCollisionConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < SphereCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_SphereCollisionConstraints);
		#endif
			SphereCollisionConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < CapsuleCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_CapsuleCollisionConstraints);
		#endif
			CapsuleCollisionConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < BoxCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_BoxCollisionConstraints);
		#endif
			BoxCollisionConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		///-------------------------------------------------------------------------------------

		///-------------------------------------------------------------------------------------
		/// Self collision constratins
		for (int32 i = 0; i < SelfCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_SelfCollisionConstraints);
		#endif
			SelfCollisionConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		///-------------------------------------------------------------------------------------

		/*for (int32 i = 0; i < WorldCollisionConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_WorldCollisionConstraints);
		#endif
			WorldCollisionConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}*/

		/*for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}
		for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
		#endif
			FixedDistanceConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
		}*/

		/*if (bLockTipBone)
		{
			/// Backward Update
			for (int32 i = PinConstraints.Num() - 1; i >= 0; --i)
			{
			#if LK_ENABLE_STAT
				SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
			#endif
				PinConstraints[i].BackwardUpdate(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
			}
			for (int32 i = FixedDistanceConstraints.Num() - 1; i >= 0; --i)
			{
			#if LK_ENABLE_STAT
				SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
			#endif
				FixedDistanceConstraints[i].BackwardUpdate(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
			}

			for (int32 i = 0; i < PinConstraints.Num(); ++i)
			{
				#if LK_ENABLE_STAT
					SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
				#endif
				PinConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
			}
			for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
			{
			#if LK_ENABLE_STAT
				SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
			#endif
				FixedDistanceConstraints[i].Update(SubStepDeltaTime, bInitialUpdate, bFinalizeUpdate);
			}
		}*/
	}

	/// Finalize special constraints
	for (int32 i = 0; i < WorldCollisionConstraints.Num(); ++i)
	{
	#if LK_ENABLE_STAT
		SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_WorldCollisionConstraints);
	#endif
		WorldCollisionConstraints[i].Update(InDeltaTime, false, true);
	}

	for (int32 i = 0; i < PinConstraints.Num(); ++i)
	{
	#if LK_ENABLE_STAT
		SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
	#endif
		PinConstraints[i].Update(InDeltaTime, false, true);
	}
	for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
	{
	#if LK_ENABLE_STAT
		SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
	#endif
		FixedDistanceConstraints[i].Update(InDeltaTime, false, true);
	}

	if (bLockTipBone)
	{
		/// Backward Update
		for (int32 i = PinConstraints.Num() - 1; i >= 0; --i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].BackwardUpdate(InDeltaTime, false, true);
		}
		for (int32 i = FixedDistanceConstraints.Num() - 1; i >= 0; --i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
		#endif
			FixedDistanceConstraints[i].BackwardUpdate(InDeltaTime, false, true);
		}

		for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_PinConstraints);
		#endif
			PinConstraints[i].Update(InDeltaTime, false, true);
		}
		for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
		{
		#if LK_ENABLE_STAT
			SCOPE_CYCLE_COUNTER(STAT_AnimVerlet_SolveConstraints_FixedDistanceConstraints);
		#endif
			FixedDistanceConstraints[i].Update(InDeltaTime, false, true);
		}
	}

	/// PostUpdate each bones
	ForEachConstraints([InDeltaTime](FLKAnimVerletConstraint& CurConstraint) {
		CurConstraint.PostUpdate(InDeltaTime);
	});
}

void FLKAnimNode_AnimVerlet::ApplyComponentInertiaTangentialDamping(float InDeltaTime)
{
	const float BaseRetention = FMath::Clamp(ComponentInertiaTangentialDamping, 0.0f, 1.0f);
	if (BaseRetention >= 1.0f - KINDA_SMALL_NUMBER)
		return;

	/// Treat the setting as the retained fraction per 60 Hz step so the damping rate remains
	/// approximately consistent when the simulation frame rate changes.
	const float Retention = FMath::IsNearlyZero(BaseRetention)
		? 0.0f
		: FMath::Pow(BaseRetention, FMath::Max(InDeltaTime, 0.0f) * 60.0f);

	for (FLKAnimVerletBone& ChildBone : SimulateBones)
	{
		if (ChildBone.HasParentBone() == false)
			continue;

		FLKAnimVerletBone& ParentBone = SimulateBones[ChildBone.ParentVerletBoneIndex];
		const float ParentInvMass = ParentBone.IsPinned() ? 0.0f : ParentBone.InvMass;
		const float ChildInvMass = ChildBone.IsPinned() ? 0.0f : ChildBone.InvMass;
		const float InvMassSum = ParentInvMass + ChildInvMass;
		if (InvMassSum <= KINDA_SMALL_NUMBER)
			continue;

		const FVector SegmentAxis = (ChildBone.Location - ParentBone.Location).GetSafeNormal();
		if (SegmentAxis.IsNearlyZero(KINDA_SMALL_NUMBER))
			continue;

		const FVector ParentMoveDelta = ParentBone.Location - ParentBone.PrevLocation;
		const FVector ChildMoveDelta = ChildBone.Location - ChildBone.PrevLocation;
		const FVector RelativeMoveDelta = ChildMoveDelta - ParentMoveDelta;
		const FVector TangentialMoveDelta = RelativeMoveDelta - SegmentAxis * FVector::DotProduct(RelativeMoveDelta, SegmentAxis);
		const FVector DampingDelta = TangentialMoveDelta * (1.0f - Retention);

		/// Adjust the Verlet history instead of the solved positions so distance and collision
		/// constraints remain satisfied. The mass-weighted split preserves pair momentum.
		ParentBone.PrevLocation -= DampingDelta * (ParentInvMass / InvMassSum);
		ChildBone.PrevLocation += DampingDelta * (ChildInvMass / InvMassSum);
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

		/// Calculate rotation. Transport the previous simulated orientation to the new segment direction so roll stays continuous even when the segment points opposite its pose.
		FQuat ReferenceRotation = ParentBone->PoseRotation;
		FVector ReferenceDirection = ParentToCurPoseDir;
		if (CurBoneIndicator.HasParentSimulateBone())
		{
			const FLKAnimVerletBone& ParentSimulateBone = SimulateBones[CurBoneIndicator.ParentAnimVerletBoneIndex];
			ReferenceRotation = ParentSimulateBone.PrevRotation;

			/// The segment's local aim axis is stable across animation poses. Recreate the previous frame's aim direction from it instead of comparing against the pose.
			const FVector LocalAimDirection = ParentBone->PoseRotation.UnrotateVector(ParentToCurPoseDir).GetSafeNormal();
			ReferenceDirection = ReferenceRotation.RotateVector(LocalAimDirection).GetSafeNormal();
		}

		if (ReferenceDirection.IsNearlyZero() || ParentToCurVerletDir.IsNearlyZero())
		{
			ParentBone->Rotation = ReferenceRotation;
			continue;
		}

		FQuat DeltaRotation = FQuat::Identity;
		const float DirectionDot = FMath::Clamp(FVector::DotProduct(ReferenceDirection, ParentToCurVerletDir), -1.0f, 1.0f);
		if (DirectionDot < -1.0f + KINDA_SMALL_NUMBER)
		{
			/// Exactly opposite directions have no unique rotation axis. Select the axis of the previous orientation that is most perpendicular to the aim direction.
			/// This makes the fallback deterministic and preserves roll continuity.
			FVector StableRotationAxis = ReferenceRotation.GetAxisX();
			float StableAxisAlignment = FMath::Abs(FVector::DotProduct(StableRotationAxis, ReferenceDirection));

			const FVector CandidateAxisY = ReferenceRotation.GetAxisY();
			const float CandidateAxisYAlignment = FMath::Abs(FVector::DotProduct(CandidateAxisY, ReferenceDirection));
			if (CandidateAxisYAlignment < StableAxisAlignment)
			{
				StableRotationAxis = CandidateAxisY;
				StableAxisAlignment = CandidateAxisYAlignment;
			}

			const FVector CandidateAxisZ = ReferenceRotation.GetAxisZ();
			if (FMath::Abs(FVector::DotProduct(CandidateAxisZ, ReferenceDirection)) < StableAxisAlignment)
				StableRotationAxis = CandidateAxisZ;

			StableRotationAxis = FVector::VectorPlaneProject(StableRotationAxis, ReferenceDirection).GetSafeNormal();
			DeltaRotation = FQuat(StableRotationAxis, PI);
		}
		else
		{
			DeltaRotation = FQuat::FindBetweenNormals(ReferenceDirection, ParentToCurVerletDir);
		}

		const FQuat TransportedRotation = (DeltaRotation * ReferenceRotation).GetNormalized();
		ParentBone->Rotation = TransportedRotation;

		/// Parallel transport preserves roll continuity, but its roll is path-dependent and can drift away from the animation pose. 
		/// Away from the pose-opposite singularity, blend toward the pose-based solution. Both rotations align the same local aim axis to ParentToCurVerletDir, so this interpolation acts as a twist-only correction.
		constexpr float NoRollRecoveryDot = -0.98480775f;	/// cos(170 degrees)
		constexpr float FullRollRecoveryDot = -0.86602540f;	/// cos(150 degrees)
		const float PoseDirectionDot = FMath::Clamp(FVector::DotProduct(ParentToCurPoseDir, ParentToCurVerletDir), -1.0f, 1.0f);
		const float RollRecoveryAlpha = FMath::SmoothStep(NoRollRecoveryDot, FullRollRecoveryDot, PoseDirectionDot);
		if (RollRecoveryAlpha > KINDA_SMALL_NUMBER)
		{
			const FQuat PoseDeltaRotation = FQuat::FindBetweenNormals(ParentToCurPoseDir, ParentToCurVerletDir);
			const FQuat PoseBasedRotation = (PoseDeltaRotation * ParentBone->PoseRotation).GetNormalized();
			ParentBone->Rotation = FQuat::Slerp(TransportedRotation, PoseBasedRotation, RollRecoveryAlpha).GetNormalized();
		}
	}
}

void FLKAnimNode_AnimVerlet::ApplyResult(OUT TArray<FBoneTransform>& OutBoneTransforms, FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
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
			const FTransform& PoseBoneT = PoseContext.Pose.GetComponentSpaceTransform(BonePoseIndex);
			const FTransform SimulatedBoneT(CurBone->Rotation, CurBone->Location, PoseBoneT.GetScale3D());
			FTransform ResultBoneT;
			ResultBoneT.Blend(PoseBoneT, SimulatedBoneT, FMath::Clamp(OutputBlendAlpha, 0.0f, 1.0f));
			OutBoneTransforms.Emplace(FBoneTransform(BonePoseIndex, ResultBoneT));
		}
	}

	/// Need to sort by UE4 rules (SimulateBones != OutBoneTransforms)
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}

void FLKAnimNode_AnimVerlet::ResetOutputBlend()
{
	OutputBlendAlpha = 0.0f;
}

void FLKAnimNode_AnimVerlet::AdvanceOutputBlend(float InDeltaTime)
{
	if (OutputBlendDuration <= UE_SMALL_NUMBER)
	{
		OutputBlendAlpha = 1.0f;
		return;
	}

	OutputBlendAlpha = FMath::Clamp(OutputBlendAlpha + FMath::Max(InDeltaTime, 0.0f) / OutputBlendDuration, 0.0f, 1.0f);
}

void FLKAnimNode_AnimVerlet::ClearSimulateBones()
{
	SimulatingCollisionShapes.ResetCollisionShapeList();

	///Constraints.Reset();
	PinConstraints.Reset();
	DistanceConstraints.Reset();
	BendingConstraints.Reset();
	BendingConstraints_1D.Reset();
	FlatBendingConstraints.Reset();
	StraightenConstraints.Reset();
	FixedDistanceConstraints.Reset();
	BallSocketConstraints.Reset();
	SphereCollisionConstraints.Reset();
	CapsuleCollisionConstraints.Reset();
	BoxCollisionConstraints.Reset();
	PlaneCollisionConstraints.Reset();
	WorldCollisionConstraints.Reset();
	SelfCollisionConstraints.Reset();
	CustomDistanceConstraintBones.Reset();

	BroadphaseContainer.Destroy();
	BoneChainIndexes.Reset();
	MaxBoneChainLength = 0;
	RelevantBoneIndicators.Reset();
	SimulateBonePairIndicators.Reset();
	SimulateBoneTriangleIndicators.Reset();
	ExcludedBones.Reset();
	SimulateBones.Reset();
}

void FLKAnimNode_AnimVerlet::ResetSimulation()
{
	FixedStepAccumulator = 0.0f;
	NumPendingSimulationSteps = 0;

	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		CurVerletBone.ResetSimulation();
	}

	for (FLKAnimVerletBone& CurAnchorBone : CustomDistanceConstraintBones)
	{
		CurAnchorBone.ResetSimulation();
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
	for (FLKAnimVerletConstraint_Bending_1D& CurConstraint : BendingConstraints_1D)
	{
		Pred(CurConstraint);
	}
	for (FLKAnimVerletConstraint_FlatBending& CurConstraint : FlatBendingConstraints)
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
	for (FLKAnimVerletConstraint_Self& CurConstraint : SelfCollisionConstraints)
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
	CollisionDataAsset->MarkPackageDirty();

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

bool FLKAnimNode_AnimVerlet::ConvertPhysicsAssetToDataAsset()
{
	if (CollisionPhysicsAsset == nullptr)
		return false;

	if (CollisionDataAsset == nullptr)
		return false;

	FLKAnimVerletCollisionShapeList ShapeList;
	ConvertPhysicsAssetToShape(OUT ShapeList, *CollisionPhysicsAsset, nullptr);

	CollisionDataAsset->Reset();
	CollisionDataAsset->ConvertFromShape(ShapeList);
	CollisionDataAsset->MarkPackageDirty();

	return true;
}

bool FLKAnimNode_AnimVerlet::ConvertCollisionShapesFromPhysicsAsset()
{
	if (CollisionPhysicsAsset == nullptr)
		return false;

	ResetCollisionShapes();

	FLKAnimVerletCollisionShapeList ShapeList;
	ConvertPhysicsAssetToShape(OUT ShapeList, *CollisionPhysicsAsset, nullptr);
	CollisionShapesFromCollisionShapeList(ShapeList);

	return true;
}

void FLKAnimNode_AnimVerlet::SyncFromOtherAnimVerletNode(const FLKAnimNode_AnimVerlet& Other)
{
	///VerletBones = Other.VerletBones;

	bSubDivideBones = Other.bSubDivideBones;
	NumSubDividedBone = Other.NumSubDividedBone;
	bRebuildSimulationOnLODChange = Other.bRebuildSimulationOnLODChange;
	bActivate = Other.bActivate;
	bSkipUpdateOnDedicatedServer = Other.bSkipUpdateOnDedicatedServer;
	bPause = Other.bPause;
	PlaySpeedRate = Other.PlaySpeedRate;
	bUseWarmup = Other.bUseWarmup;
	WarmupStepCount = Other.WarmupStepCount;
	WarmupFixedDeltaTime = Other.WarmupFixedDeltaTime;
	OutputBlendDuration = Other.OutputBlendDuration;

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
	bAlignAnimationPoseToGravity = Other.bAlignAnimationPoseToGravity;
	AnimationPoseInertia = Other.AnimationPoseInertia;
	bApplyAnimationPoseInertiaCorrection = Other.bApplyAnimationPoseInertiaCorrection;
	AnimationPoseInertiaTargetFrameRate = Other.AnimationPoseInertiaTargetFrameRate;

	Damping = Other.Damping;
	bApplyDampingCorrection = Other.bApplyDampingCorrection;
	DampingCorrectionTargetFrameRate = Other.DampingCorrectionTargetFrameRate;

	bUseXPBDSolver = Other.bUseXPBDSolver;
	InvCompliance = Other.InvCompliance;
	Stiffness = Other.Stiffness;
	CustomDistanceConstraints = Other.CustomDistanceConstraints;

	bUseSleep = Other.bUseSleep;
	bIgnoreSleepWhenParentWakedUp = Other.bIgnoreSleepWhenParentWakedUp;
	SleepDeltaThreshold = Other.SleepDeltaThreshold;
	SleepTriggerDuration = Other.SleepTriggerDuration;
	WakeUpDeltaThreshold = Other.WakeUpDeltaThreshold;

	bConstrainRightDiagonalDistance = Other.bConstrainRightDiagonalDistance;
	bConstrainLeftDiagonalDistance = Other.bConstrainLeftDiagonalDistance;
	bUseIsometricBendingConstraint = Other.bUseIsometricBendingConstraint;
	InvBendingCompliance = Other.InvBendingCompliance;
	bUseBendingComplianceRange = Other.bUseBendingComplianceRange;
	InvBendingComplianceMin = Other.InvBendingComplianceMin;
	InvBendingComplianceMax = Other.InvBendingComplianceMax;
	BendingComplianceMaxAngle = Other.BendingComplianceMaxAngle;
	BendingStiffness = Other.BendingStiffness;
	bUseBendingStiffnessRange = Other.bUseBendingStiffnessRange;
	BendingStiffnessMin = Other.BendingStiffnessMin;
	BendingStiffnessMax = Other.BendingStiffnessMax;
	BendingStiffnessMaxAngle = Other.BendingStiffnessMaxAngle;

	bPreserveLengthFromParent = Other.bPreserveLengthFromParent;
	bPreserveLengthFromParentBetweenRealBones = Other.bPreserveLengthFromParentBetweenRealBones;
	LengthFromParentMargin = Other.LengthFromParentMargin;
	bPreserveSideLength = Other.bPreserveSideLength;
	bPreserveSideLengthBetweenRealBones = Other.bPreserveSideLengthBetweenRealBones;
	SideLengthMargin = Other.SideLengthMargin;

	bStretchEachBone = Other.bStretchEachBone;
	StretchStrength = Other.StretchStrength;

	bStraightenBendedBone = Other.bStraightenBendedBone;
	StraightenBendedBoneStrength = Other.StraightenBendedBoneStrength;

	bUseFlatBendingConstraint = Other.bUseFlatBendingConstraint;
	InvFlatBendingCompliance = Other.InvFlatBendingCompliance;
	bUseFlatBendingComplianceRange = Other.bUseFlatBendingComplianceRange;
	InvFlatBendingComplianceMin = Other.InvFlatBendingComplianceMin;
	InvFlatBendingComplianceMax = Other.InvFlatBendingComplianceMax;
	FlatBendingComplianceMaxAngle = Other.FlatBendingComplianceMaxAngle;
	FlatBendingStiffness = Other.FlatBendingStiffness;
	bUseFlatBendingStiffnessRange = Other.bUseFlatBendingStiffnessRange;
	FlatBendingStiffnessMin = Other.FlatBendingStiffnessMin;
	FlatBendingStiffnessMax = Other.FlatBendingStiffnessMax;
	FlatBendingStiffnessMaxAngle = Other.FlatBendingStiffnessMaxAngle;
	FlatBendingAlpha = Other.FlatBendingAlpha;

	SolveIteration = Other.SolveIteration;

	FixedDeltaTime = Other.FixedDeltaTime;
	bApplyDeltaTimeCorrection = Other.bApplyDeltaTimeCorrection;
	DeltaTimeCorrectionTargetFrameRate = Other.DeltaTimeCorrectionTargetFrameRate;
	MaxSubStep = Other.MaxSubStep;
	MinDeltaTime = Other.MinDeltaTime;
	MaxDeltaTime = Other.MaxDeltaTime;
	bUseSquaredDeltaTime = Other.bUseSquaredDeltaTime;

	bConstrainConeAngleFromParent = Other.bConstrainConeAngleFromParent;
	ConeAngle = Other.ConeAngle;
	ConeAngleOffset = Other.ConeAngleOffset;
	bUseBroadphase = Other.bUseBroadphase;
	Thickness = Other.Thickness;
	FrictionCoefficient = Other.FrictionCoefficient;
	bUseCapsuleCollisionForChain = Other.bUseCapsuleCollisionForChain;

	bUseSelfCollision = Other.bUseSelfCollision;
	bUseTriangleSelfCollision = Other.bUseTriangleSelfCollision;
	SelfCollisionAdditionalThickness = Other.SelfCollisionAdditionalThickness;

	WorldCollisionProfile = Other.WorldCollisionProfile;
	WorldCollisionExcludeBones = Other.WorldCollisionExcludeBones;

	SphereCollisionShapes = Other.SphereCollisionShapes;
	CapsuleCollisionShapes = Other.CapsuleCollisionShapes;
	BoxCollisionShapes = Other.BoxCollisionShapes;
	PlaneCollisionShapes = Other.PlaneCollisionShapes;
	CollisionDataAsset = Other.CollisionDataAsset;
	CollisionPhysicsAsset = Other.CollisionPhysicsAsset;
	DynamicCollisionShapes = Other.DynamicCollisionShapes;

	Gravity = Other.Gravity;
	bGravityInWorldSpace = Other.bGravityInWorldSpace;
	AnimationPoseReferenceDirection = Other.AnimationPoseReferenceDirection;

	StretchForce = Other.StretchForce;
	bAlignStretchForceToGravity = Other.bAlignStretchForceToGravity;
	SideStraightenForce = Other.SideStraightenForce;
	ShapeMemoryForce = Other.ShapeMemoryForce;
	bAlignShapeMemoryForceToGravity = Other.bAlignShapeMemoryForceToGravity;
	ExternalForce = Other.ExternalForce;
	bExternalForceInWorldSpace = Other.bExternalForceInWorldSpace;

	RandomWindDirection = Other.RandomWindDirection;
	RandomWindSizeMin = Other.RandomWindSizeMin;
	RandomWindSizeMax = Other.RandomWindSizeMax;
	bRandomWindDirectionInWorldSpace = Other.bRandomWindDirectionInWorldSpace;
	AdditionalRandomWinds = Other.AdditionalRandomWinds;

	bAdjustWindComponent = Other.bAdjustWindComponent;
	WindComponentScale = Other.WindComponentScale;

	MoveInertiaScale = Other.MoveInertiaScale;
	bIgnoreSuddenMoveInertia = Other.bIgnoreSuddenMoveInertia;
	MoveInertiaIgnoreThreshold = Other.MoveInertiaIgnoreThreshold;
	bClampMoveInertia = Other.bClampMoveInertia;
	MoveInertiaClampMaxDistance = Other.MoveInertiaClampMaxDistance;

	RotationInertiaScale = Other.RotationInertiaScale;
	bIgnoreSuddenRotationInertia = Other.bIgnoreSuddenRotationInertia;
	RotationInertiaIgnoreDegrees = Other.RotationInertiaIgnoreDegrees;
	bClampRotationInertia = Other.bClampRotationInertia;
	RotationInertiaClampDegrees = Other.RotationInertiaClampDegrees;
	ComponentInertiaTangentialDamping = Other.ComponentInertiaTangentialDamping;
}

void FLKAnimNode_AnimVerlet::ApplyPresetType(ELKAnimVerletPreset InPresetType)
{
	switch (InPresetType)
	{
		case ELKAnimVerletPreset::AnimationPose:
		{
			bIgnoreAnimationPose = false;
			bUseXPBDSolver = false;
			bUseSquaredDeltaTime = false;

			bUseIsometricBendingConstraint = false;
			Damping = 0.8f;
			SolveIteration = 2;
			Gravity = FVector(0.0f, 0.0f, -9.8f);
			break;
		}

		case ELKAnimVerletPreset::Physics_XPBD:
		{
			bIgnoreAnimationPose = true;
			bUseXPBDSolver = true;
			bUseSquaredDeltaTime = true;

			bUseIsometricBendingConstraint = true;
			Damping = 0.99f;
			SolveIteration = 4;
			Gravity = FVector(0.0f, 0.0f, -980.0f);
			break;
		}

		case ELKAnimVerletPreset::Physics_PBD:
		{
			bIgnoreAnimationPose = true;
			bUseXPBDSolver = false;
			bUseSquaredDeltaTime = true;

			bUseIsometricBendingConstraint = false;
			Damping = 0.9f;
			SolveIteration = 4;
			Gravity = FVector(0.0f, 0.0f, -980.0f);
			break;
		}

		default:
			break;
	}
}

bool FLKAnimNode_AnimVerlet::IsPresetTypeRelatedProperty(const FName& InPropertyName) const
{
	#if WITH_EDITOR
	if (InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, bIgnoreAnimationPose)
		|| InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, bUseXPBDSolver)
		|| InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, bUseSquaredDeltaTime)
		|| InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, bUseIsometricBendingConstraint)
		|| InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, SolveIteration)
		///|| InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, Gravity)
		///|| InPropertyName == GET_MEMBER_NAME_CHECKED(FLKAnimNode_AnimVerlet, Damping)
		)
	{
		return true;
	}
	#endif
	return false;
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
	const bool bDrawTriangle = bUseCapsuleCollisionForChain && (IsSingleChain() == false);
	if (bDrawTriangle)
	{
		for (const FLKAnimVerletBoneIndicatorTriangle& CurTriangle : SimulateBoneTriangleIndicators)
		{
			if (CurTriangle.BoneA.IsValidBoneIndicator() == false || SimulateBones.IsValidIndex(CurTriangle.BoneA.AnimVerletBoneIndex) == false)
				continue;
			if (CurTriangle.BoneB.IsValidBoneIndicator() == false || SimulateBones.IsValidIndex(CurTriangle.BoneB.AnimVerletBoneIndex) == false)
				continue;
			if (CurTriangle.BoneC.IsValidBoneIndicator() == false || SimulateBones.IsValidIndex(CurTriangle.BoneC.AnimVerletBoneIndex) == false)
				continue;

			const FLKAnimVerletBone& AVerletBone = SimulateBones[CurTriangle.BoneA.AnimVerletBoneIndex];
			if (AVerletBone.bOverrideToUseSphereCollisionForChain)
				continue;
			const FLKAnimVerletBone& BVerletBone = SimulateBones[CurTriangle.BoneB.AnimVerletBoneIndex];
			if (BVerletBone.bOverrideToUseSphereCollisionForChain)
				continue;
			const FLKAnimVerletBone& CVerletBone = SimulateBones[CurTriangle.BoneC.AnimVerletBoneIndex];
			if (CVerletBone.bOverrideToUseSphereCollisionForChain)
				continue;

			const float TriThickness = FMath::Max3(AVerletBone.Thickness, BVerletBone.Thickness, CVerletBone.Thickness);
			{
				const FVector WorldLocation = ComponentToWorld.TransformPosition((BVerletBone.Location + AVerletBone.Location) * 0.5f);
				const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(BVerletBone.Location - AVerletBone.Location).ToQuat();
				const FQuat WorldRotation = ComponentToWorld.TransformRotation(CapsuleRotation);
				AnimInstanceProxy->AnimDrawDebugCapsule(WorldLocation, (BVerletBone.Location - AVerletBone.Location).Size() * 0.5f + TriThickness, TriThickness, WorldRotation.Rotator(), FColor::Blue, false, -1.0f, SDPG_Foreground);
			}
			{
				const FVector WorldLocation = ComponentToWorld.TransformPosition((CVerletBone.Location + AVerletBone.Location) * 0.5f);
				const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(CVerletBone.Location - AVerletBone.Location).ToQuat();
				const FQuat WorldRotation = ComponentToWorld.TransformRotation(CapsuleRotation);
				AnimInstanceProxy->AnimDrawDebugCapsule(WorldLocation, (CVerletBone.Location - AVerletBone.Location).Size() * 0.5f + TriThickness, TriThickness, WorldRotation.Rotator(), FColor::Blue, false, -1.0f, SDPG_Foreground);
			}
			{
				const FVector WorldLocation = ComponentToWorld.TransformPosition((CVerletBone.Location + BVerletBone.Location) * 0.5f);
				const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(CVerletBone.Location - BVerletBone.Location).ToQuat();
				const FQuat WorldRotation = ComponentToWorld.TransformRotation(CapsuleRotation);
				AnimInstanceProxy->AnimDrawDebugCapsule(WorldLocation, (CVerletBone.Location - BVerletBone.Location).Size() * 0.5f + TriThickness, TriThickness, WorldRotation.Rotator(), FColor::Blue, false, -1.0f, SDPG_Foreground);
			}
		}
	}
	else
	{
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
			const FVector ConstraintDirection = CurConstraint.GetConstraintDirection();
			const FVector Dir = ComponentToWorld.TransformVectorNoScale(ConstraintDirection).GetSafeNormal();
			const float Length = (CurConstraint.BoneB->PoseLocation - CurConstraint.BoneA->PoseLocation).Size();
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