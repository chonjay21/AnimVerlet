#pragma once
#include <CoreMinimal.h>
#include <Kismet/BlueprintFunctionLibrary.h>
#include "LKAnimVerletCollisionShape.h"
#include "LKAnimVerletBlueprintFunctionLibrary.generated.h"

UCLASS()
class ANIMVERLET_API ULKAnimVerletBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Creates absolute-world AnimVerlet collision shapes from the primitive component's body instance. */
	UFUNCTION(BlueprintCallable, Category = "AnimVerlet|Collision")
	static void MakeCollisionShapeListFromPrimitiveComponent(FLKAnimVerletCollisionShapeList& OutShapeList, class UPrimitiveComponent* PrimitiveComponent);

	/** Creates absolute-world AnimVerlet collision shapes from every body instance owned by the skeletal mesh component. */
	UFUNCTION(BlueprintCallable, Category = "AnimVerlet|Collision")
	static void MakeCollisionShapeListFromSkeletalMeshComponent(FLKAnimVerletCollisionShapeList& OutShapeList, const class USkeletalMeshComponent* SkeletalMeshComponent);

	/** Converts a collision data asset to absolute-world shapes using the skeletal mesh component's current bone transforms. */
	UFUNCTION(BlueprintCallable, Category = "AnimVerlet|Collision")
	static void MakeCollisionShapeListFromCollisionDataAsset(FLKAnimVerletCollisionShapeList& OutShapeList, const class USkeletalMeshComponent* SkeletalMeshComponent, const class ULKAnimVerletCollisionDataAsset* CollisionDataAsset);

	/** Converts a physics asset to absolute-world shapes using the skeletal mesh component's current bone transforms. */
	UFUNCTION(BlueprintCallable, Category = "AnimVerlet|Collision")
	static void MakeCollisionShapeListFromPhysicsAsset(FLKAnimVerletCollisionShapeList& OutShapeList, const class USkeletalMeshComponent* SkeletalMeshComponent, const class UPhysicsAsset* PhysicsAsset);

	/** Converts bone-attached shapes to absolute-world shapes using the skeletal mesh component's current bone transforms. */
	UFUNCTION(BlueprintCallable, Category = "AnimVerlet|Collision")
	static void MakeCollisionShapeListFromCollisionShapeList(FLKAnimVerletCollisionShapeList& OutShapeList, const class USkeletalMeshComponent* SkeletalMeshComponent, const FLKAnimVerletCollisionShapeList& CollisionShapeList);
};
