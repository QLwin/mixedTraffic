// Copyright QLwin. All Rights Reserved.
#include "Data/TrafficVehiclePreset.h"

void UTrafficVehiclePreset::ApplyCarDefaults()
{
	VehicleType = ETrafficVehicleType::Car;
	Length      = 5.0f;
	Width       = 2.0f;
	LongParams  = FLongModelParams::CarDefaults();
}

void UTrafficVehiclePreset::ApplyTruckDefaults()
{
	VehicleType = ETrafficVehicleType::Truck;
	Length      = 10.0f;
	Width       = 2.5f;
	LongParams  = FLongModelParams::TruckDefaults();
}

void UTrafficVehiclePreset::ApplyBikeDefaults()
{
	VehicleType = ETrafficVehicleType::Bike;
	Length      = 2.0f;
	Width       = 0.7f;
	LongParams  = FLongModelParams::BikeDefaults();
}
