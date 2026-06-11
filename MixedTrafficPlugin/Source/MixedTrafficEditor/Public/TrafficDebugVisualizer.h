// Copyright QLwin. All Rights Reserved.
// TrafficDebugVisualizer.h
//
// Editor / PIE debug component.  Attach to any Actor that has a
// UTrafficVehicleComponent to visualise:
//   - Acceleration vectors (like the "Display Forces" toggle in the JS sim)
//   - Neighbour search range box
//   - Road boundary proximity indicator
//
// Rendered via UPrimitiveComponent::DrawDebugComponents (M5 implementation).
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TrafficDebugVisualizer.generated.h"

UCLASS(ClassGroup="MixedTraffic|Debug",
       meta=(BlueprintSpawnableComponent),
       HideCategories=(Object, LOD))
class MIXEDTRAFFICEDITOR_API UTrafficDebugVisualizer : public UActorComponent
{
	GENERATED_BODY()

public:
	UTrafficDebugVisualizer();

	/** Draw acceleration arrows in PIE */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
	bool bShowAccelerationVectors = true;

	/** Draw the longitudinal neighbour search range */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
	bool bShowNeighbourRange = false;

	/** Scale factor for acceleration arrow length [cm per m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug", meta=(ClampMin="1"))
	float ArrowScale = 50.0f;

protected:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;
};
