#pragma once
#include <CoreMinimal.h>

#if	(ENGINE_MAJOR_VERSION == 5)
	#define LK_UEWIDGET UE::Widget
#else
	#define LK_UEWIDGET FWidget
#endif


UENUM(BlueprintType)
enum class ELKAnimVerletPreset : uint8
{
	Custom,

	/**
		bIgnoreAnimationPose = false;
		bUseXPBDSolver = false;
		bUseSquaredDeltaTime = false;
		bUseIsometricBendingConstraint = false;
		Damping = 0.8f;
		SolveIteration = 1;
		Gravity = FVector(0.0f, 0.0f, -9.8f);
	*/
	AnimationPose,

	/**
		bIgnoreAnimationPose = true;
		bUseXPBDSolver = true;
		bUseSquaredDeltaTime = true;
		bUseIsometricBendingConstraint = true;
		Damping = 0.99f;
		SolveIteration = 4;
		Gravity = FVector(0.0f, 0.0f, -980.0f);
	*/
	Physics_XPBD,

	/**
		bIgnoreAnimationPose = true;
		bUseXPBDSolver = false;
		bUseSquaredDeltaTime = true;
		bUseIsometricBendingConstraint = false;
		Damping = 0.9f;
		SolveIteration = 4;
		Gravity = FVector(0.0f, 0.0f, -980.0f);
	*/
	Physics_PBD
};