#include "LKAnimVerletConstraint.h"

#include "LKAnimVerletBone.h"

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Pin
///=========================================================================================================================================
void FLKAnimVerletConstraint_Pin::Update(float DeltaTime)
{
	verify(Bone != nullptr);

	if (FMath::IsNearlyZero(PinMargin))
	{
		Bone->Location = Bone->PoseLocation;
	}
	else
	{
		/// Calculate the distance
		FVector Direction = FVector::ZeroVector;
		float Distance = 0.0f;
		(Bone->Location - Bone->PoseLocation).ToDirectionAndLength(OUT Direction, OUT Distance);

		/// Adjust distance constraint
		if (Distance > PinMargin)
		{
			Bone->Location = Bone->PoseLocation + Direction * PinMargin;
		}
	}
}

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Distance
///=========================================================================================================================================
FLKAnimVerletConstraint_Distance::FLKAnimVerletConstraint_Distance(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, bool bInUseXPBDSolver, double InStiffness, bool bInStretchEachBone, float InStretchStrength)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, bStretchEachBone(bInStretchEachBone)
	, StretchStrength(InStretchStrength)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	Length = (BoneB->PoseLocation - BoneA->PoseLocation).Size();
	Lambda = 0.0f;

	bUseXPBDSolver = bInUseXPBDSolver;
	if (bUseXPBDSolver)
		Compliance = InStiffness;
	else
		Stiffness = static_cast<float>(InStiffness);
}

void FLKAnimVerletConstraint_Distance::Update(float DeltaTime)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	/// Update length
	FVector PoseDirection = FVector::ZeroVector;
	(BoneB->PoseLocation - BoneA->PoseLocation).ToDirectionAndLength(OUT PoseDirection, OUT Length);

	/// Calculate the distance
	FVector Direction = FVector::ZeroVector;
	float Distance = 0.0f;
	(BoneB->Location - BoneA->Location).ToDirectionAndLength(OUT Direction, OUT Distance);

	/// XPBD
	if (bUseXPBDSolver)
	{
		const float C = Distance - Length;
		const double Alpha = Compliance / (DeltaTime * DeltaTime);
		const double DeltaLambda = -(C + Alpha * Lambda) / (BoneA->InvMass + BoneB->InvMass + Alpha);
		Lambda += DeltaLambda;

		if (bStretchEachBone)
			Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

		/// Adjust distance constraint
		const FVector DiffDir = (Direction * DeltaLambda);
		BoneA->Location -= (DiffDir * BoneA->InvMass);
		BoneB->Location += (DiffDir * BoneB->InvMass);
	}
	/// PBD
	else
	{
		/// Calculate the resting distance
		const float Diff = ((Length - Distance) / Distance) * Stiffness;
		
		if (bStretchEachBone)
			Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();
		
		/// Adjust distance constraint
		const FVector DiffDir = Direction * Diff * 0.5f;
		BoneA->Location -= (DiffDir * BoneA->InvMass);
		BoneB->Location += (DiffDir * BoneB->InvMass);
	}
}

void FLKAnimVerletConstraint_Distance::PostUpdate(float DeltaTime)
{
	Lambda = 0.0f;
}

void FLKAnimVerletConstraint_Distance::ResetSimulation()
{
	Lambda = 0.0f;
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_FixedDistance
///=========================================================================================================================================
FLKAnimVerletConstraint_FixedDistance::FLKAnimVerletConstraint_FixedDistance(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, bool bInStretchEachBone, float InStretchStrength, bool bInAwayFromEachOther, float InLengthMargin)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, bStretchEachBone(bInStretchEachBone)
	, bAwayFromEachOther(bInAwayFromEachOther)
	, StretchStrength(InStretchStrength)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
	verify(InLengthMargin >= 0.0f);

	Length = (BoneB->PoseLocation - BoneA->PoseLocation).Size();
	LengthMargin = InLengthMargin;
}

void FLKAnimVerletConstraint_FixedDistance::Update(float DeltaTime)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	/// Update length
	FVector PoseDirection = FVector::ZeroVector;
	(BoneB->PoseLocation - BoneA->PoseLocation).ToDirectionAndLength(OUT PoseDirection, OUT Length);

	/// Calculate the distance
	FVector Direction = FVector::ZeroVector;
	float Distance = 0.0f;
	(BoneB->Location - BoneA->Location).ToDirectionAndLength(OUT Direction, OUT Distance);

	///if (bStretchEachBone)
	///	Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

	/// Adjust distance constraint
	if (Distance > Length + LengthMargin)
	{
		const float LengthWithMargin = Length + LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneA->Location + Direction * Distance * 0.5f;
			BoneB->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneA->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneB->Location = BoneA->Location + Direction * LengthWithMargin;
		}
	}
	else if (Distance < Length - LengthMargin)
	{
		const float LengthWithMargin = Length - LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneA->Location + Direction * Distance * 0.5f;
			BoneB->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneA->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneB->Location = BoneA->Location + Direction * LengthWithMargin;
		}
	}
}

void FLKAnimVerletConstraint_FixedDistance::BackwardUpdate(float DeltaTime)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	/// Update length
	FVector PoseDirection = FVector::ZeroVector;
	(BoneA->PoseLocation - BoneB->PoseLocation).ToDirectionAndLength(OUT PoseDirection, OUT Length);


	/// Calculate the distance
	FVector Direction = FVector::ZeroVector;
	float Distance = 0.0f;
	(BoneA->Location - BoneB->Location).ToDirectionAndLength(OUT Direction, OUT Distance);

	///if (bStretchEachBone)
	///	Direction = (Direction + PoseDirection * StretchStrength).GetSafeNormal();

	/// Adjust distance constraint
	if (Distance > Length + LengthMargin)
	{
		const float LengthWithMargin = Length + LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneB->Location + Direction * Distance * 0.5f;
			BoneA->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneB->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneA->Location = BoneB->Location + Direction * LengthWithMargin;
		}
	}
	else if (Distance < Length - LengthMargin)
	{
		const float LengthWithMargin = Length - LengthMargin;
		if (bAwayFromEachOther)
		{
			const FVector Center = BoneB->Location + Direction * Distance * 0.5f;
			BoneA->Location = Center + Direction * LengthWithMargin * 0.5f;
			BoneB->Location = Center - Direction * LengthWithMargin * 0.5f;
		}
		else
		{
			BoneA->Location = BoneB->Location + Direction * LengthWithMargin;
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_BallSocket
///=========================================================================================================================================
FLKAnimVerletConstraint_BallSocket::FLKAnimVerletConstraint_BallSocket(FLKAnimVerletBone* InBoneA, FLKAnimVerletBone* InBoneB, float InAngleDegrees)
	: BoneA(InBoneA)
	, BoneB(InBoneB)
	, AngleDegrees(InAngleDegrees)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);
}

void FLKAnimVerletConstraint_BallSocket::Update(float DeltaTime)
{
	verify(BoneA != nullptr);
	verify(BoneB != nullptr);

	FVector BoneAToBoneB = FVector::ZeroVector;
	float BoneAToBoneBSize = 0.0f;
	(BoneB->Location - BoneA->Location).ToDirectionAndLength(OUT BoneAToBoneB, OUT BoneAToBoneBSize);
	const FVector PoseAToPoseB = (BoneB->PoseLocation - BoneA->PoseLocation).GetSafeNormal();

	const FVector RotationAxis = FVector::CrossProduct(PoseAToPoseB, BoneAToBoneB);
	const float RotationAngle = FMath::Acos(FVector::DotProduct(PoseAToPoseB, BoneAToBoneB));
	const float AngleDiff = FMath::RadiansToDegrees(RotationAngle) - AngleDegrees;

	if (AngleDiff > 0.0f)
	{
		const FVector ConstraintDir = BoneAToBoneB.RotateAngleAxis(-AngleDiff, RotationAxis);
		BoneB->Location = BoneA->Location + (ConstraintDir * BoneAToBoneBSize);
	}
}
///=========================================================================================================================================


///=========================================================================================================================================
/// FLKAnimVerletConstraint_Sphere
///=========================================================================================================================================
FLKAnimVerletConstraint_Sphere::FLKAnimVerletConstraint_Sphere(const FVector& InLocation, float InRadius, float InThickness, 
															   TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones)
	: Location(InLocation)
	, Radius(InRadius)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
{
	verify(Bones != nullptr);
}

void FLKAnimVerletConstraint_Sphere::Update(float DeltaTime)
{
	verify(Bones != nullptr);

	const float ConstraintDistance = Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector SphereToBone = (CurVerletBone.Location - Location);
		const float SphereToBoneSQ = SphereToBone.SizeSquared();
		if (SphereToBoneSQ < ConstraintDistanceSQ)
		{
			const float SphereToBoneSize = FMath::Sqrt(SphereToBoneSQ);
			const FVector SphereToBoneDir = SphereToBoneSize > KINDA_SMALL_NUMBER ? (SphereToBone / SphereToBoneSize) : FVector::ZeroVector;
			CurVerletBone.Location = Location + (SphereToBoneDir * ConstraintDistance);
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Capsule
///=========================================================================================================================================
FLKAnimVerletConstraint_Capsule::FLKAnimVerletConstraint_Capsule(const FVector& InLocation, const FQuat& InRot, float InRadius, float InHalfHeight, float InThickness, 
																 TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones)
	: Location(InLocation)
	, Rotation(InRot)
	, Radius(InRadius)
	, HalfHeight(InHalfHeight)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
{
	verify(Bones != nullptr);
}

void FLKAnimVerletConstraint_Capsule::Update(float DeltaTime)
{
	verify(Bones != nullptr);

	const float ConstraintDistance = Thickness + Radius;
	const float ConstraintDistanceSQ = FMath::Square(ConstraintDistance);

	const FVector CapsuleHeightDir = Rotation.GetUpVector();
	const FVector CapsuleStart = Location - CapsuleHeightDir * HalfHeight;
	const FVector CapsuleEnd = Location + CapsuleHeightDir * HalfHeight;
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector ClosestOnCapsule = FMath::ClosestPointOnSegment(CurVerletBone.Location, CapsuleStart, CapsuleEnd);
		const float CapsuleToBoneSQ = (CurVerletBone.Location - ClosestOnCapsule).SizeSquared();
		if (CapsuleToBoneSQ < ConstraintDistanceSQ)
		{
			const FVector CapsuleToBoneDir = (CurVerletBone.Location - ClosestOnCapsule).GetSafeNormal();
			CurVerletBone.Location = ClosestOnCapsule + (CapsuleToBoneDir * ConstraintDistance);
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Box
///=========================================================================================================================================
FLKAnimVerletConstraint_Box::FLKAnimVerletConstraint_Box(const FVector& InLocation, const FQuat& InRot, const FVector& InHalfExtents, float InThickness, 
														 TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones)
	: Location(InLocation)
	, Rotation(InRot)
	, HalfExtents(InHalfExtents)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
{
	verify(Bones != nullptr);
}

void FLKAnimVerletConstraint_Box::Update(float DeltaTime)
{
	verify(Bones != nullptr);

	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector BoneLocationInBoxLocal = InvRotation.RotateVector(CurVerletBone.Location - Location);
		if (FMath::Abs(BoneLocationInBoxLocal.X) < HalfExtents.X + Thickness &&
			FMath::Abs(BoneLocationInBoxLocal.Y) < HalfExtents.Y + Thickness &&
			FMath::Abs(BoneLocationInBoxLocal.Z) < HalfExtents.Z + Thickness)
		{
			const FVector MaxDistToSurface = BoneLocationInBoxLocal - HalfExtents;
			const FVector MinDistsToSurface = -HalfExtents - BoneLocationInBoxLocal;

			/// Determine closest distance and normal to box surface(PenetrationDepth is negative in this overlap case)
			float ClosestPenetrationDepthToSurface = 0.0f;
			FVector NormalToSurface = FVector::ZeroVector;
			const FVector ComponentMax = MaxDistToSurface.ComponentMax(MinDistsToSurface);
			if (ComponentMax.X > ComponentMax.Y)
			{
				if (ComponentMax.X > ComponentMax.Z)
				{
					ClosestPenetrationDepthToSurface = ComponentMax.X;
					NormalToSurface.X = MaxDistToSurface.X > MinDistsToSurface.X ? 1.0f : -1.0f;
				}
				else
				{
					ClosestPenetrationDepthToSurface = ComponentMax.Z;
					NormalToSurface.Z = MaxDistToSurface.Z > MinDistsToSurface.Z ? 1.0f : -1.0f;
				}
			}
			else
			{
				if (ComponentMax.Y > ComponentMax.Z)
				{
					ClosestPenetrationDepthToSurface = ComponentMax.Y;
					NormalToSurface.Y = MaxDistToSurface.Y > MinDistsToSurface.Y ? 1.0f : -1.0f;
				}
				else
				{
					ClosestPenetrationDepthToSurface = ComponentMax.Z;
					NormalToSurface.Z = MaxDistToSurface.Z > MinDistsToSurface.Z ? 1.0f : -1.0f;
				}
			}
			verify(NormalToSurface.IsNormalized());

			CurVerletBone.Location = CurVerletBone.Location - Rotation.RotateVector(NormalToSurface) * (ClosestPenetrationDepthToSurface - Thickness);
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_Plane
///=========================================================================================================================================
FLKAnimVerletConstraint_Plane::FLKAnimVerletConstraint_Plane(const FVector& InPlaneBase, const FVector& InPlaneNormal, const FQuat& InRotation, const FVector2D& InPlaneHalfExtents, 
															 float InThickness, TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones)
	: PlaneBase(InPlaneBase)
	, PlaneNormal(InPlaneNormal)
	, Rotation(InRotation)
	, PlaneHalfExtents(InPlaneHalfExtents)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
{
	verify(Bones != nullptr);
}

void FLKAnimVerletConstraint_Plane::Update(float DeltaTime)
{
	verify(Bones != nullptr);

	const bool bFinitePlane = (PlaneHalfExtents.IsNearlyZero() == false);
	const FQuat InvRotation = Rotation.Inverse();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const float DistToPlane = FVector::PointPlaneDist(CurVerletBone.Location, PlaneBase, PlaneNormal);
		if (DistToPlane < Thickness)
		{
			if (bFinitePlane)
			{
				const FVector ProjectedLocationOnPlane = CurVerletBone.Location - (DistToPlane * PlaneNormal);
				const FVector BoneLocationInPlaneLocal = InvRotation.RotateVector(ProjectedLocationOnPlane - PlaneBase);
				if (BoneLocationInPlaneLocal.X > PlaneHalfExtents.X + Thickness || BoneLocationInPlaneLocal.X < -PlaneHalfExtents.X + Thickness ||
					BoneLocationInPlaneLocal.Y > PlaneHalfExtents.Y + Thickness || BoneLocationInPlaneLocal.Y < -PlaneHalfExtents.Y + Thickness)
					continue;
			}
			CurVerletBone.Location += (PlaneNormal * (Thickness - DistToPlane));
		}
	}
}
///=========================================================================================================================================

///=========================================================================================================================================
/// FLKAnimVerletConstraint_World
///=========================================================================================================================================
FLKAnimVerletConstraint_World::FLKAnimVerletConstraint_World(const UWorld* InWorld, UPrimitiveComponent* InSelfComponent, const FName& InCollisionProfileName,
															 float InThickness, TArray<FLKAnimVerletBone>* InBones, const TExcludeBoneBits& InExcludeBones)
	: WorldPtr(InWorld)
	, SelfComponentPtr(InSelfComponent)
	, WorldCollisionProfileName(InCollisionProfileName)
	, Thickness(InThickness)
	, Bones(InBones)
	, ExcludeBones(InExcludeBones)
{
	verify(WorldPtr.IsValid());
	verify(SelfComponentPtr.IsValid());
	verify(WorldCollisionProfileName != NAME_None);
	verify(Bones != nullptr);
}

void FLKAnimVerletConstraint_World::Update(float DeltaTime)
{
	verify(Bones != nullptr);

	if (WorldPtr.IsValid() == false || SelfComponentPtr.IsValid() == false)
		return;

	const UWorld* World = WorldPtr.Get();
	UPrimitiveComponent* SelfComponent = SelfComponentPtr.Get();

	FCollisionQueryParams CollisionQueryParams(SCENE_QUERY_STAT(LKAnimVerlet));
	CollisionQueryParams.AddIgnoredComponent(SelfComponent);
	if (SelfComponent->GetOwner() != nullptr)
		CollisionQueryParams.AddIgnoredActor(SelfComponent->GetOwner());

	const FTransform ComponentTransform = SelfComponent->GetComponentTransform();
	for (int32 i = 0; i < Bones->Num(); ++i)
	{
		if (ExcludeBones.IsValidIndex(i) && ExcludeBones[i])
			continue;

		FLKAnimVerletBone& CurVerletBone = (*Bones)[i];
		const FVector PrevWorldLoc = ComponentTransform.TransformPosition(CurVerletBone.PrevLocation);
		const FVector CurWorldLoc = ComponentTransform.TransformPosition(CurVerletBone.Location);

		FHitResult HitResult;
		const bool bHit = World->SweepSingleByProfile(OUT HitResult, PrevWorldLoc, CurWorldLoc, FQuat::Identity, WorldCollisionProfileName, FCollisionShape::MakeSphere(Thickness), CollisionQueryParams);
		if (bHit)
		{
			if (HitResult.bStartPenetrating && HitResult.PenetrationDepth > 0.0f)
			{
				const FVector ResolvedLocationInWorld = HitResult.Location + (HitResult.Normal * HitResult.PenetrationDepth);
				CurVerletBone.Location = ComponentTransform.InverseTransformPosition(ResolvedLocationInWorld);
			}
			else
			{
				const FVector ResolvedLocationInWorld = HitResult.Location;
				CurVerletBone.Location = ComponentTransform.InverseTransformPosition(ResolvedLocationInWorld);
			}
		}
	}
}
///=========================================================================================================================================