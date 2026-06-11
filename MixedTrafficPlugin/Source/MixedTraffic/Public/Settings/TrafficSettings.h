// Copyright QLwin. All Rights Reserved.
// TrafficSettings.h – project-wide global settings for the Mixed Traffic plugin.
// Accessible via Project Settings → Mixed Traffic.
#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Models/MTMTypes.h"
#include "TrafficSettings.generated.h"

UCLASS(Config=MixedTraffic, DefaultConfig, meta=(DisplayName="Mixed Traffic"))
class MIXEDTRAFFIC_API UTrafficSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UTrafficSettings();

	// ── Boundary response defaults ────────────────────────────────────────
	// These populate FMTMParams for every vehicle unless overridden per-preset.

	UPROPERTY(Config, EditAnywhere, Category="MTM Boundary Defaults")
	float AccLatBMax = 20.0f;

	UPROPERTY(Config, EditAnywhere, Category="MTM Boundary Defaults")
	float AccLatBRef = 15.0f;

	UPROPERTY(Config, EditAnywhere, Category="MTM Boundary Defaults")
	float AccLongBRef = 0.2f;

	UPROPERTY(Config, EditAnywhere, Category="MTM Boundary Defaults")
	float AnticFactorB = 2.0f;

	UPROPERTY(Config, EditAnywhere, Category="MTM Boundary Defaults")
	float S0yB = 0.15f;

	UPROPERTY(Config, EditAnywhere, Category="MTM Boundary Defaults")
	float S0yLatB = 0.20f;

	// ── LOD distances ─────────────────────────────────────────────────────

	/** Enter full MTM 2-D mode when vehicle is within this distance of the player [m] */
	UPROPERTY(Config, EditAnywhere, Category="LOD", meta=(ClampMin="10", ClampMax="500"))
	float LodFullRadius = 100.0f;

	/** Drop back to background mode beyond this distance (> LodFullRadius) [m] */
	UPROPERTY(Config, EditAnywhere, Category="LOD", meta=(ClampMin="10", ClampMax="600"))
	float LodBackgroundRadius = 150.0f;

	// UDeveloperSettings interface
	virtual FName GetCategoryName() const override { return FName(TEXT("Plugins")); }

	/** Build FMTMParams with boundary values from project settings */
	FMTMParams MakeDefaultMTMParams() const;
};
