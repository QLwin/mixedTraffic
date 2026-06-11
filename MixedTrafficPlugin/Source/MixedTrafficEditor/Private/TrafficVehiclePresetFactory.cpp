// Copyright QLwin. All Rights Reserved.
// TrafficVehiclePresetFactory.cpp
// Registers the "Traffic Vehicle Preset" asset type so it appears in the
// Content Browser → Add → Miscellaneous menu.
#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "Data/TrafficVehiclePreset.h"
#include "AssetTypeCategories.h"
#include "TrafficVehiclePresetFactory.generated.h"

UCLASS()
class UTrafficVehiclePresetFactory : public UFactory
{
	GENERATED_BODY()

public:
	UTrafficVehiclePresetFactory()
	{
		bCreateNew    = true;
		bEditAfterNew = true;
		SupportedClass = UTrafficVehiclePreset::StaticClass();
	}

	virtual FText GetDisplayName() const override
	{
		return NSLOCTEXT("MixedTrafficEditor", "PresetFactory", "Traffic Vehicle Preset");
	}

	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent,
	                                   FName InName, EObjectFlags Flags,
	                                   UObject* Context, FFeedbackContext* Warn) override
	{
		UTrafficVehiclePreset* Asset = NewObject<UTrafficVehiclePreset>(InParent, InClass, InName, Flags);
		Asset->ApplyCarDefaults();  // sensible starting point
		return Asset;
	}
};
