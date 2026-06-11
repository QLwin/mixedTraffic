// Copyright QLwin. All Rights Reserved.
// TrafficVehiclePresetFactory.h
// Registers the "Traffic Vehicle Preset" asset type so it appears in the
// Content Browser → Add → Miscellaneous menu.
#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "TrafficVehiclePresetFactory.generated.h"

UCLASS()
class UTrafficVehiclePresetFactory : public UFactory
{
	GENERATED_BODY()

public:
	UTrafficVehiclePresetFactory();

	virtual FText GetDisplayName() const override;

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent,
	                                   FName InName, EObjectFlags Flags,
	                                   UObject* Context, FFeedbackContext* Warn) override;
};
