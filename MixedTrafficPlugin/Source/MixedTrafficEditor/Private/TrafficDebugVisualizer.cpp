// Copyright QLwin. All Rights Reserved.
#include "TrafficDebugVisualizer.h"
#include "Components/TrafficVehicleComponent.h"
#include "DrawDebugHelpers.h"

UTrafficDebugVisualizer::UTrafficDebugVisualizer()
{
	PrimaryComponentTick.bCanEverTick = true;
	// Only tick in editor and PIE, not in shipping builds
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UTrafficDebugVisualizer::TickComponent(float DeltaTime, ELevelTick TickType,
                                             FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

#if !UE_BUILD_SHIPPING
	AActor* Owner = GetOwner();
	if (!Owner) { return; }

	UTrafficVehicleComponent* TrafficComp = Owner->FindComponentByClass<UTrafficVehicleComponent>();
	if (!TrafficComp) { return; }

	const FVector Origin = Owner->GetActorLocation();
	UWorld* World = GetWorld();

	if (bShowAccelerationVectors)
	{
		const FTrafficVehicleState& S = TrafficComp->State;

		// Longitudinal acceleration arrow (actor forward direction)
		const FVector FwdDir = Owner->GetActorForwardVector();
		const FVector LatDir = Owner->GetActorRightVector();

		const FVector AccLongEnd = Origin + FwdDir * (S.AccLong * ArrowScale);
		const FVector AccLatEnd  = Origin + LatDir  * (S.AccLat  * ArrowScale);

		DrawDebugDirectionalArrow(World, Origin, AccLongEnd, 20.0f, FColor::Green, false, -1.0f, 0, 3.0f);
		DrawDebugDirectionalArrow(World, Origin, AccLatEnd,  20.0f, FColor::Blue,  false, -1.0f, 0, 3.0f);
	}
#endif
}
