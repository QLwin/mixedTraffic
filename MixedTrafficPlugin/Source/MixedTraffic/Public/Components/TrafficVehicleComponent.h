// Copyright QLwin. All Rights Reserved.
// TrafficVehicleComponent.h
//
// ActorComponent that makes any Actor participate in the Mixed Traffic system.
// The component holds an FMTMModel instance and exposes vehicle parameters to
// Blueprints.  The actual per-frame acceleration calculation is driven by
// UTrafficRoadSubsystem; this component stores state and provides the interface.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Models/MTMTypes.h"
#include "Models/MTMModel.h"
#include "TrafficVehicleComponent.generated.h"

class UTrafficVehiclePreset;

UCLASS(ClassGroup="MixedTraffic", meta=(BlueprintSpawnableComponent),
       HideCategories=(Object, LOD, Lighting, TextureStreaming))
class MIXEDTRAFFIC_API UTrafficVehicleComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UTrafficVehicleComponent();

	// ── Configuration (set before BeginPlay) ─────────────────────────────

	/** Optional preset asset; if set, overrides all inline parameters below */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Preset")
	TObjectPtr<UTrafficVehiclePreset> Preset;

	/** Vehicle type; also used to choose mesh/color if no Preset given */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Vehicle",
	          meta=(EditCondition="Preset==nullptr"))
	ETrafficVehicleType VehicleType = ETrafficVehicleType::Car;

	/** Vehicle length [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Vehicle",
	          meta=(EditCondition="Preset==nullptr", ClampMin="0.5"))
	float VehicleLength = 5.0f;

	/** Vehicle width [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Vehicle",
	          meta=(EditCondition="Preset==nullptr", ClampMin="0.3"))
	float VehicleWidth = 2.0f;

	/** Longitudinal model parameters (used when Preset is null) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Model",
	          meta=(EditCondition="Preset==nullptr"))
	FLongModelParams LongParams;

	/** MTM lateral / boundary parameters (used when Preset is null) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Traffic|Model",
	          meta=(EditCondition="Preset==nullptr"))
	FMTMParams MTMParams;

	// ── Runtime state (read-only from Blueprints) ─────────────────────────

	/** Live vehicle state (positions and velocities in logical road coordinates) */
	UPROPERTY(BlueprintReadOnly, Transient, Category="Traffic|State")
	FTrafficVehicleState State;

	// ── Blueprint-callable helpers ────────────────────────────────────────

	/** Returns the current longitudinal speed in m/s */
	UFUNCTION(BlueprintPure, Category="Traffic")
	float GetSpeedLong() const { return State.SpeedLong; }

	/** Returns the current lateral speed in m/s */
	UFUNCTION(BlueprintPure, Category="Traffic")
	float GetSpeedLat() const { return State.SpeedLat; }

	/** Returns the last computed longitudinal acceleration [m/s²] */
	UFUNCTION(BlueprintPure, Category="Traffic")
	float GetAccLong() const { return State.AccLong; }

	/** Returns the last computed lateral acceleration [m/s²] */
	UFUNCTION(BlueprintPure, Category="Traffic")
	float GetAccLat() const { return State.AccLat; }

	/** Sets logical road position (u = arc-length, v = lateral offset) [m] */
	UFUNCTION(BlueprintCallable, Category="Traffic")
	void SetLogicalPosition(float U, float V);

	/** Sets logical velocity [m/s] */
	UFUNCTION(BlueprintCallable, Category="Traffic")
	void SetLogicalVelocity(float SpeedLong, float SpeedLat);

	// ── Internal (used by UTrafficRoadSubsystem) ──────────────────────────

	/** Returns a reference to the MTM model (built from params or preset) */
	const FMTMModel& GetMTMModel() const { return CachedModel; }

	/** Rebuilds CachedModel from Preset or inline params; call after any param change */
	void RebuildModel();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** The resolved MTM model used each tick */
	FMTMModel CachedModel;
};
