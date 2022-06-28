#include "LKAnimNode_AnimVerlet.h"

#include <AnimationRuntime.h>
#include <Animation/AnimInstanceProxy.h>
#include <Animation/AnimTypes.h>
#include <DrawDebugHelpers.h>

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
	if (DeltaTime <= 0.0f)
		return;
	if (Output.AnimInstanceProxy == nullptr || Output.AnimInstanceProxy->GetSkelMeshComponent() == nullptr)
		return;
	if (Output.AnimInstanceProxy->GetSkelMeshComponent()->GetWorld() == nullptr)
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
	PrepareSimulation(Output, BoneContainer);

	/// Simulate verlet integration
	const USkeletalMeshComponent* SkeletalMeshComponent = Output.AnimInstanceProxy->GetSkelMeshComponent();
	const UWorld* World = SkeletalMeshComponent->GetWorld();
	SimulateVerlet(World, DeltaTime, CurComponentT, PrevComponentT);

	/// Apply simulation to bone
	ApplyResult(OutBoneTransforms, BoneContainer);

	PrevComponentT = CurComponentT;
}

void FLKAnimNode_AnimVerlet::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	for (FLKAnimVerletBoneSetting& CurBoneSetting : VerletBones)
	{
		CurBoneSetting.RootBone.Initialize(RequiredBones);
	}

	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		SimulateBones[i].BoneReference.Initialize(RequiredBones);
	}
}

bool FLKAnimNode_AnimVerlet::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
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

	DeltaTime = FMath::IsNearlyZero(FixedDeltaTime, KINDA_SMALL_NUMBER) ? FMath::Clamp(Context.GetDeltaTime(), 0.0f, MaxDeltaTime) : FixedDeltaTime;
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
			MakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, FoundBoneIndex, INDEX_NONE, INDEX_NONE, CurBoneSetting);
	}

	/// Create constraints
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurSimulateBone = SimulateBones[i];
		if (CurSimulateBone.HasParentBone() == false)
		{
			const FLKAnimVerletConstraint_Pin PinConstraint(&CurSimulateBone, StartBoneLockMargin);
			PinConstraints.Emplace(PinConstraint);
		}
		else
		{
			FLKAnimVerletBone& ParentSimulateBone = SimulateBones[CurSimulateBone.ParentVerletBoneIndex];
			const FLKAnimVerletConstraint_Distance DistanceConstraint(&ParentSimulateBone, &CurSimulateBone, Stiffness);
			DistanceConstraints.Emplace(DistanceConstraint);

			if (bPreserveLengthFromParent)
			{
				const FLKAnimVerletConstraint_FixedDistance FixedDistanceConstraint(&ParentSimulateBone, &CurSimulateBone, false, LengthFromParentMargin);
				FixedDistanceConstraints.Emplace(FixedDistanceConstraint);
			}

			if (ConeAngle > 0.0f)
			{
				const FLKAnimVerletConstraint_BallSocket BallSocketConstraint(&ParentSimulateBone, &CurSimulateBone, ConeAngle);
				BallSocketConstraints.Emplace(BallSocketConstraint);
			}

			if (bLockTipBone)
			{
				if (CurSimulateBone.IsTipBone())
				{
					const FLKAnimVerletConstraint_Pin PinConstraint(&CurSimulateBone, TipBoneLockMargin);
					PinConstraints.Emplace(PinConstraint);
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
				if (BoneChainIndexes.IsValidIndex(LeftIndex))
				{
					const TArray<int32>& CurBoneChain = BoneChainIndexes[LeftCurIndex];
					const TArray<int32>& LeftBoneChain = BoneChainIndexes[LeftIndex];
					if (i < CurBoneChain.Num() && i < LeftBoneChain.Num())
					{
						verify(SimulateBones.IsValidIndex(CurBoneChain[i]));
						verify(SimulateBones.IsValidIndex(LeftBoneChain[i]));

						const FLKAnimVerletConstraint_Distance DistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], Stiffness);
						DistanceConstraints.Emplace(DistanceConstraint);

						if (bPreserveSideLength)
						{
							const FLKAnimVerletConstraint_FixedDistance FixedDistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i]], false, SideLengthMargin);
							FixedDistanceConstraints.Emplace(FixedDistanceConstraint);
						}

						if (bConstrainRightDiagonalDistance)
						{
							if (i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_Distance LeftDiagonalConstraint(&SimulateBones[CurBoneChain[i + 1]], &SimulateBones[LeftBoneChain[i]], Stiffness);
								DistanceConstraints.Emplace(LeftDiagonalConstraint);
							}
						}
						if (bConstrainLeftDiagonalDistance)
						{
							if (i + 1 < LeftBoneChain.Num())
							{
								const FLKAnimVerletConstraint_Distance RightDiagonalConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[LeftBoneChain[i + 1]], Stiffness);
								DistanceConstraints.Emplace(RightDiagonalConstraint);
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

						const FLKAnimVerletConstraint_Distance DistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], Stiffness);
						DistanceConstraints.Emplace(DistanceConstraint);

						if (bPreserveSideLength)
						{
							const FLKAnimVerletConstraint_FixedDistance FixedDistanceConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i]], false, SideLengthMargin);
							FixedDistanceConstraints.Emplace(FixedDistanceConstraint);
						}

						if (bConstrainRightDiagonalDistance)
						{
							if (i + 1 < RightBoneChain.Num())
							{
								const FLKAnimVerletConstraint_Distance RightDiagonalConstraint(&SimulateBones[CurBoneChain[i]], &SimulateBones[RightBoneChain[i + 1]], Stiffness);
								DistanceConstraints.Emplace(RightDiagonalConstraint);
							}
						}
						if (bConstrainLeftDiagonalDistance)
						{
							if (i + 1 < CurBoneChain.Num())
							{
								const FLKAnimVerletConstraint_Distance LeftDiagonalConstraint(&SimulateBones[CurBoneChain[i + 1]], &SimulateBones[RightBoneChain[i]], Stiffness);
								DistanceConstraints.Emplace(LeftDiagonalConstraint);
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
				const UWorld* World = SkeletalMeshComponent->GetWorld();

				const FLKAnimVerletConstraint_World WorldCollisionConstraint(World, SkeletalMeshComponent, WorldCollisionProfile, Thickness, &SimulateBones, TExcludeBoneBits());
				WorldCollisionConstraints.Emplace(WorldCollisionConstraint);
			}
		}
	}

	/// LocalCollision(Contact) constraints
	for (FLKAnimVerletCollisionSphere& CurShape : SphereCollisionShapes)
	{
		CurShape.AttachedBone.Initialize(BoneContainer);

		CurShape.ExcludeBoneBits.Init(false, SimulateBones.Num());
		for (int32 i = 0; i < CurShape.ExcludeBones.Num(); ++i)
		{
			CurShape.ExcludeBones[i].Initialize(BoneContainer);
			const int32 FoundIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(CurShape.ExcludeBones[i]));
			if (FoundIndex != INDEX_NONE)
				CurShape.ExcludeBoneBits[FoundIndex] = true;
		}
	}
	for (FLKAnimVerletCollisionCapsule& CurShape : CapsuleCollisionShapes)
	{
		CurShape.AttachedBone.Initialize(BoneContainer);
		
		CurShape.ExcludeBoneBits.Init(false, SimulateBones.Num());
		for (int32 i = 0; i < CurShape.ExcludeBones.Num(); ++i)
		{
			CurShape.ExcludeBones[i].Initialize(BoneContainer);
			const int32 FoundIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(CurShape.ExcludeBones[i]));
			if (FoundIndex != INDEX_NONE)
				CurShape.ExcludeBoneBits[FoundIndex] = true;
		}
	}
	for (FLKAnimVerletCollisionBox& CurShape : BoxCollisionShapes)
	{
		CurShape.AttachedBone.Initialize(BoneContainer);
		
		CurShape.ExcludeBoneBits.Init(false, SimulateBones.Num());
		for (int32 i = 0; i < CurShape.ExcludeBones.Num(); ++i)
		{
			CurShape.ExcludeBones[i].Initialize(BoneContainer);
			const int32 FoundIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(CurShape.ExcludeBones[i]));
			if (FoundIndex != INDEX_NONE)
				CurShape.ExcludeBoneBits[FoundIndex] = true;
		}
	}
	for (FLKAnimVerletCollisionPlane& CurShape : PlaneCollisionShapes)
	{
		CurShape.AttachedBone.Initialize(BoneContainer);
		
		CurShape.ExcludeBoneBits.Init(false, SimulateBones.Num());
		for (int32 i = 0; i < CurShape.ExcludeBones.Num(); ++i)
		{
			CurShape.ExcludeBones[i].Initialize(BoneContainer);
			const int32 FoundIndex = SimulateBones.IndexOfByKey(FLKAnimVerletBoneKey(CurShape.ExcludeBones[i]));
			if (FoundIndex != INDEX_NONE)
				CurShape.ExcludeBoneBits[FoundIndex] = true;
		}
	}
}

bool FLKAnimNode_AnimVerlet::MakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton,
											   int32 BoneIndex, int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting)
{
	verify(BoneIndex >= 0 && BoneIndex < ReferenceSkeleton.GetNum());

	FBoneReference CurBoneRef;
	CurBoneRef.BoneName = ReferenceSkeleton.GetBoneName(BoneIndex);
	
	int32 CurSimulateBoneIndex = ParentSimulateBoneIndex;
	const bool bExcludedBone = (BoneSetting.ExcludeBones.Find(CurBoneRef) != INDEX_NONE);
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
				const int32 CurAddedBoneChainLength = BoneChainIndexes[RootSimulateBoneIndex].Add(SubDividedSimulateBoneIndex) + 1;
				MaxBoneChainLength = FMath::Max(MaxBoneChainLength, CurAddedBoneChainLength);

				SubDividedParentSimulateBoneIndex = SubDividedSimulateBoneIndex;
			}
			NewSimulateBone.ParentVerletBoneIndex = SubDividedParentSimulateBoneIndex;
		}

		CurSimulateBoneIndex = SimulateBones.Emplace(NewSimulateBone);
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
	const bool bTipBone = (WalkChildsAndMakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, BoneIndex, CurSimulateBoneIndex, RootSimulateBoneIndex, BoneSetting) == false);

	if (bTipBone && bExcludedBone == false)
	{
		if (bLockTipBone == false && bMakeFakeTipBone && FakeTipBoneLength > 0.0f)
		{
			FLKAnimVerletBone FakeSimulateBone;
			FakeSimulateBone.bFakeBone = true;
			FakeSimulateBone.bTipBone = true;
			FakeSimulateBone.ParentVerletBoneIndex = CurSimulateBoneIndex;

			FTransform FakeBoneT = FTransform::Identity;
			MakeFakeBoneTransform(OUT FakeBoneT, CurSimulateBoneIndex);
			FakeSimulateBone.InitializeTransform(FakeBoneT);

			const int32 FakeBoneIndex = SimulateBones.Emplace(FakeSimulateBone);
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

bool FLKAnimNode_AnimVerlet::WalkChildsAndMakeSimulateBones(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer, const FReferenceSkeleton& ReferenceSkeleton,
															int32 BoneIndex, int32 ParentSimulateBoneIndex, int32 RootSimulateBoneIndex, const FLKAnimVerletBoneSetting& BoneSetting)
{
	verify(RootSimulateBoneIndex != INDEX_NONE);

	bool bWalked = false;
	const int32 NumBones = ReferenceSkeleton.GetNum();
	for (int32 ChildIndex = BoneIndex + 1; ChildIndex < NumBones; ++ChildIndex)
	{
		if (BoneIndex == ReferenceSkeleton.GetParentIndex(ChildIndex))
		{
			MakeSimulateBones(PoseContext, BoneContainer, ReferenceSkeleton, ChildIndex, ParentSimulateBoneIndex, RootSimulateBoneIndex, BoneSetting);
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

void FLKAnimNode_AnimVerlet::PrepareSimulation(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
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
		CurSimulateBone.PrepareSimulation(CurBonePoseT);
	}

	PrepareLocalCollisionConstraints(PoseContext, BoneContainer);
}

void FLKAnimNode_AnimVerlet::PrepareLocalCollisionConstraints(FComponentSpacePoseContext& PoseContext, const FBoneContainer& BoneContainer)
{
	SphereCollisionConstraints.Reset();
	for (int32 i = 0; i < SphereCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionSphere& CurShapeSphere = SphereCollisionShapes[i];
		
		const FCompactPoseBoneIndex PoseBoneIndex = CurShapeSphere.AttachedBone.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		if (PoseBoneIndex != INDEX_NONE)
		{
			FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
			const FTransform OffsetT(FQuat::Identity, CurShapeSphere.LocationOffset);

			BoneTInCS = OffsetT * BoneTInCS;
			const FVector BoneLocation = BoneTInCS.GetLocation();

			const FLKAnimVerletConstraint_Sphere SphereConstraint(BoneLocation, CurShapeSphere.Radius, Thickness, 
																  &SimulateBones, CurShapeSphere.ExcludeBoneBits);
			SphereCollisionConstraints.Emplace(SphereConstraint);
		}
	}

	CapsuleCollisionConstraints.Reset();
	for (int32 i = 0; i < CapsuleCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionCapsule& CurShapeCapsule = CapsuleCollisionShapes[i];
		
		const FCompactPoseBoneIndex PoseBoneIndex = CurShapeCapsule.AttachedBone.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		if (PoseBoneIndex != INDEX_NONE)
		{
			FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
			const FTransform OffsetT(CurShapeCapsule.RotationOffset.Quaternion(), CurShapeCapsule.LocationOffset);

			BoneTInCS = OffsetT * BoneTInCS;
			const FVector BoneLocation = BoneTInCS.GetLocation();
			const FQuat BoneRotation = BoneTInCS.GetRotation();

			const FLKAnimVerletConstraint_Capsule CapsuleConstraint(BoneLocation, BoneRotation, CurShapeCapsule.Radius, CurShapeCapsule.HalfHeight, Thickness, 
																	&SimulateBones, CurShapeCapsule.ExcludeBoneBits);
			CapsuleCollisionConstraints.Emplace(CapsuleConstraint);
		}
	}

	BoxCollisionConstraints.Reset();
	for (int32 i = 0; i < BoxCollisionShapes.Num(); ++i)
	{
		const FLKAnimVerletCollisionBox& CurShapeBox = BoxCollisionShapes[i];

		const FCompactPoseBoneIndex PoseBoneIndex = CurShapeBox.AttachedBone.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		if (PoseBoneIndex != INDEX_NONE)
		{
			FTransform BoneTInCS = PoseContext.Pose.GetComponentSpaceTransform(PoseBoneIndex);
			const FTransform OffsetT(CurShapeBox.RotationOffset.Quaternion(), CurShapeBox.LocationOffset);

			BoneTInCS = OffsetT * BoneTInCS;
			const FVector BoneLocation = BoneTInCS.GetLocation();
			const FQuat BoneRotation = BoneTInCS.GetRotation();

			const FLKAnimVerletConstraint_Box BoxConstraint(BoneLocation, BoneRotation, CurShapeBox.HalfExtents, Thickness, 
															&SimulateBones, CurShapeBox.ExcludeBoneBits);
			BoxCollisionConstraints.Emplace(BoxConstraint);
		}
	}

	PlaneCollisionConstraints.Reset();
	{
		for (int32 i = 0; i < PlaneCollisionShapes.Num(); ++i)
		{
			const FLKAnimVerletCollisionPlane& CurShapePlane = PlaneCollisionShapes[i];

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
																	CurShapePlane.bFinitePlane ? CurShapePlane.FinitePlaneHalfExtents : FVector2D::ZeroVector, Thickness, 
																	&SimulateBones, CurShapePlane.ExcludeBoneBits);
				PlaneCollisionConstraints.Emplace(PlaneConstraint);
			}
		}
	}
}

void FLKAnimNode_AnimVerlet::SimulateVerlet(const UWorld* World, float InDeltaTime, const FTransform& ComponentTransform, const FTransform& PrevComponentTransform)
{
	verify(World != nullptr);

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

	const float SubStepDeltaTime = InDeltaTime / SolveIteration;
	for (int32 Iteration = 0; Iteration < SolveIteration; ++Iteration)
	{
		/// Simulate each constraints
		/// Solve order is important. Make a heuristic order by constraint priority.
		/*for (int32 i = 0; i < Constraints.Num(); ++i)
		{
			verify(Constraints[i] != nullptr);
			Constraints[i]->Update(SubStepDeltaTime);
		}*/
		for (int32 i = 0; i < DistanceConstraints.Num(); ++i)
		{
			DistanceConstraints[i].Update(SubStepDeltaTime);
		}
		for (int32 i = 0; i < BallSocketConstraints.Num(); ++i)
		{
			BallSocketConstraints[i].Update(SubStepDeltaTime);
		}
		for (int32 i = 0; i < SphereCollisionConstraints.Num(); ++i)
		{
			SphereCollisionConstraints[i].Update(SubStepDeltaTime);
		}
		for (int32 i = 0; i < CapsuleCollisionConstraints.Num(); ++i)
		{
			CapsuleCollisionConstraints[i].Update(SubStepDeltaTime);
		}
		for (int32 i = 0; i < BoxCollisionConstraints.Num(); ++i)
		{
			BoxCollisionConstraints[i].Update(SubStepDeltaTime);
		}
		for (int32 i = 0; i < PlaneCollisionConstraints.Num(); ++i)
		{
			PlaneCollisionConstraints[i].Update(SubStepDeltaTime);
		}
		for (int32 i = 0; i < WorldCollisionConstraints.Num(); ++i)
		{
			WorldCollisionConstraints[i].Update(SubStepDeltaTime);
		}

		for (int32 i = 0; i < PinConstraints.Num(); ++i)
		{
			PinConstraints[i].Update(SubStepDeltaTime);
		}		
		for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
		{
			FixedDistanceConstraints[i].Update(SubStepDeltaTime);
		}

		if (bLockTipBone)
		{
			/// Backward Update
			for (int32 i = PinConstraints.Num() - 1; i >= 0; --i)
			{
				PinConstraints[i].BackwardUpdate(SubStepDeltaTime);
			}
			for (int32 i = FixedDistanceConstraints.Num() - 1; i >= 0; --i)
			{
				FixedDistanceConstraints[i].BackwardUpdate(SubStepDeltaTime);
			}

			for (int32 i = 0; i < PinConstraints.Num(); ++i)
			{
				PinConstraints[i].Update(SubStepDeltaTime);
			}
			for (int32 i = 0; i < FixedDistanceConstraints.Num(); ++i)
			{
				FixedDistanceConstraints[i].Update(SubStepDeltaTime);
			}
		}
	}

	/// Calculate rotations and fix length
	for (int32 i = SimulateBones.Num() - 1; i >= 0; --i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		if (CurVerletBone.HasParentBone() == false)
			continue;

		FLKAnimVerletBone& ParentVerletBone = SimulateBones[CurVerletBone.ParentVerletBoneIndex];	
		const FVector ParentToCurPose = CurVerletBone.PoseLocation - ParentVerletBone.PoseLocation;
		const FVector ParentToCurVerlet = CurVerletBone.Location - ParentVerletBone.Location;
		
		FVector ParentToCurVerletDir = FVector::ZeroVector;
		float ParentToCurVerletSize = 0.0f;
		ParentToCurVerlet.ToDirectionAndLength(OUT ParentToCurVerletDir, OUT ParentToCurVerletSize);

		FVector ParentToCurPoseDir = FVector::ZeroVector;
		float ParentToCurPoseSize = 0.0f;
		ParentToCurPose.ToDirectionAndLength(OUT ParentToCurPoseDir, OUT ParentToCurPoseSize);

		/// Calculate rotation
		const FVector RotationAxis = FVector::CrossProduct(ParentToCurPoseDir, ParentToCurVerletDir).GetSafeNormal();
		const float RotationAngle = FMath::Acos(FVector::DotProduct(ParentToCurPoseDir, ParentToCurVerletDir));
		const FQuat DeltaRotation = FQuat(RotationAxis, RotationAngle);
		ParentVerletBone.Rotation = DeltaRotation * ParentVerletBone.PoseRotation;
		ParentVerletBone.Rotation.Normalize();
	}
}

void FLKAnimNode_AnimVerlet::ApplyResult(OUT TArray<FBoneTransform>& OutBoneTransforms, const FBoneContainer& BoneContainer)
{
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		const FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		if (CurVerletBone.bFakeBone)
			continue;

		const FCompactPoseBoneIndex BonePoseIndex = CurVerletBone.BoneReference.GetCompactPoseIndex(BoneContainer);
		/// LOD case?
		if (BonePoseIndex != INDEX_NONE)
		{
			const FTransform ResultBoneT(CurVerletBone.Rotation, CurVerletBone.Location, CurVerletBone.PoseScale);
			OutBoneTransforms.Emplace(FBoneTransform(BonePoseIndex, ResultBoneT));
		}
	}

	/// Need to sort by UE4 rules (SimulateBones != OutBoneTransforms)
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}

void FLKAnimNode_AnimVerlet::ClearSimulateBones()
{
	///Constraints.Reset();
	PinConstraints.Reset();
	DistanceConstraints.Reset();
	FixedDistanceConstraints.Reset();
	BallSocketConstraints.Reset();
	SphereCollisionConstraints.Reset();
	CapsuleCollisionConstraints.Reset();
	BoxCollisionConstraints.Reset();
	PlaneCollisionConstraints.Reset();
	WorldCollisionConstraints.Reset();

	BoneChainIndexes.Reset();
	MaxBoneChainLength = 0;
	SimulateBones.Reset();
}

void FLKAnimNode_AnimVerlet::ResetSimulation()
{
	for (int32 i = 0; i < SimulateBones.Num(); ++i)
	{
		FLKAnimVerletBone& CurVerletBone = SimulateBones[i];
		CurVerletBone.ResetSimulation();
	}
}