// Copyright QLwin. All Rights Reserved.
// TrafficRoadSubsystem.h
//
// World subsystem that coordinates all registered UTrafficVehicleComponents.
// Each Tick:
//   1. Sort vehicles by longitudinal position (u) – ascending arc-length
//   2. For each vehicle, collect neighbours within [−dumaxLag, +dumaxLead]
//   3. Compute longitudinal + lateral accelerations via FMTMModel
//   4. Apply ballistic integration to update positions and speeds
//   5. Push new world-space positions back to Actor transforms
//
// Road geometry is provided via TFunction delegates (widthLeft/widthRight).
// For M2 a straight road is used; M3 will wire in USplineComponent.
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Models/MTMTypes.h"
#include "TrafficRoadSubsystem.generated.h"

class UTrafficVehicleComponent;

UCLASS()
class MIXEDTRAFFIC_API UTrafficRoadSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	// ── Vehicle registration ──────────────────────────────────────────────

	void RegisterVehicle(UTrafficVehicleComponent* Vehicle);
	void UnregisterVehicle(UTrafficVehicleComponent* Vehicle);

	// ── Road geometry configuration ───────────────────────────────────────

	/**
	 * Set constant half-widths for a straight road (M2 prototype).
	 * Replace with SetRoadSplineFunctions() in M3.
	 *
	 * @param LeftWidth   distance from road centre to left boundary [m]
	 * @param RightWidth  distance from road centre to right boundary [m]
	 */
	UFUNCTION(BlueprintCallable, Category="Traffic|Road")
	void SetStraightRoad(float LeftWidth, float RightWidth);

	/**
	 * Provide arbitrary road boundary functions (M3+).
	 * Both functions map arc-length u [m] to half-width [m].
	 */
	void SetRoadBoundaryFunctions(TFunction<float(float)> InWidthLeft,
	                              TFunction<float(float)> InWidthRight);

	/** Total road width at position U [m] */
	UFUNCTION(BlueprintPure, Category="Traffic|Road")
	float GetRoadWidth(float U) const;

	// ── Simulation parameters ─────────────────────────────────────────────

	/** Maximum desired speed across all vehicle types [m/s]; used in boundary scaling */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Simulation")
	float V0Max = 25.0f;

	/** Push factor for following-vehicle lateral influence [0,1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Simulation",
	          meta=(ClampMin="0", ClampMax="1"))
	float PushLong = 1.0f;

	/** Push factor for lateral influence from following vehicles [0,1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Simulation",
	          meta=(ClampMin="0", ClampMax="1"))
	float PushLat = 0.0f;

	// ── UTickableWorldSubsystem interface ─────────────────────────────────

	virtual void Tick(float DeltaTime) override;
	virtual bool IsTickable() const override;
	virtual TStatId GetStatId() const override;

protected:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

private:
	/** All currently active vehicles, kept sorted by State.U descending */
	TArray<TWeakObjectPtr<UTrafficVehicleComponent>> Vehicles;

	TFunction<float(float)> WidthLeftFn;
	TFunction<float(float)> WidthRightFn;

	// Longitudinal neighbour search ranges (computed from V0Max)
	float DumaxLead = 0.0f;
	float DumaxLag  = 0.0f;

	void UpdateSearchRanges();
	void SortVehicles();

	// Returns [imin, imax] index range of neighbours of vehicle at index I
	// (same algorithm as road.get_neighborIndexRange in the JS source)
	void GetNeighbourRange(int32 I, int32& OutIMin, int32& OutIMax) const;

	// Calculate and set accLong/accLat for the vehicle at index I
	void CalcAccelerationsForVehicle(int32 I);

	// Apply ballistic update + push world transform back to Actor
	void ApplyIntegration(UTrafficVehicleComponent* Veh, float Dt) const;
};
