// Copyright QLwin. All Rights Reserved.
// TrafficVehiclePreset.h – DataAsset holding all parameters for one vehicle type.
// Create via Content Browser → Miscellaneous → Data Asset → TrafficVehiclePreset.
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Models/MTMTypes.h"
#include "TrafficVehiclePreset.generated.h"

UCLASS(BlueprintType)
class MIXEDTRAFFIC_API UTrafficVehiclePreset : public UDataAsset
{
	GENERATED_BODY()

public:
	/** Vehicle type this preset represents */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Vehicle")
	ETrafficVehicleType VehicleType = ETrafficVehicleType::Car;

	/** Vehicle length [m] */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Vehicle", meta=(ClampMin="0.5", ClampMax="30"))
	float Length = 5.0f;

	/** Vehicle width [m] */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Vehicle", meta=(ClampMin="0.3", ClampMax="5"))
	float Width = 2.0f;

	/** Longitudinal model parameters (ACC / IDM) */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Longitudinal Model")
	FLongModelParams LongParams;

	/** MTM lateral and boundary parameters */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="MTM Parameters")
	FMTMParams MTMParams;

	// ── Built-in default presets ──────────────────────────────────────────

	/** Fill this asset with Car defaults */
	UFUNCTION(CallInEditor, Category="Defaults")
	void ApplyCarDefaults();

	/** Fill this asset with Truck defaults */
	UFUNCTION(CallInEditor, Category="Defaults")
	void ApplyTruckDefaults();

	/** Fill this asset with Bike / Motorbike defaults */
	UFUNCTION(CallInEditor, Category="Defaults")
	void ApplyBikeDefaults();
};
