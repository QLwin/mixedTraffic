// Copyright QLwin. All Rights Reserved.
#include "TrafficVehiclePresetFactory.h"
#include "Data/TrafficVehiclePreset.h"

UTrafficVehiclePresetFactory::UTrafficVehiclePresetFactory()
{
	bCreateNew     = true;
	bEditAfterNew  = true;
	SupportedClass = UTrafficVehiclePreset::StaticClass();
}

FText UTrafficVehiclePresetFactory::GetDisplayName() const
{
	return NSLOCTEXT("MixedTrafficEditor", "PresetFactory", "Traffic Vehicle Preset");
}

UObject* UTrafficVehiclePresetFactory::FactoryCreateNew(UClass* InClass, UObject* InParent,
                                                         FName InName, EObjectFlags Flags,
                                                         UObject* Context, FFeedbackContext* Warn)
{
	UTrafficVehiclePreset* Asset = NewObject<UTrafficVehiclePreset>(InParent, InClass, InName, Flags);
	Asset->ApplyCarDefaults();  // sensible starting point
	return Asset;
}
