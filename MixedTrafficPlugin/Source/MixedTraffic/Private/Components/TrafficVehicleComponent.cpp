// Copyright QLwin. All Rights Reserved.
#include "Components/TrafficVehicleComponent.h"
#include "Data/TrafficVehiclePreset.h"
#include "Subsystems/TrafficRoadSubsystem.h"
#include "Engine/World.h"

UTrafficVehicleComponent::UTrafficVehicleComponent()
{
	PrimaryComponentTick.bCanEverTick = false; // subsystem drives updates
}

void UTrafficVehicleComponent::BeginPlay()
{
	Super::BeginPlay();

	RebuildModel();

	// Initialise state from current Actor transform (straight road approximation)
	// Full spline-based projection is handled by UTrafficRoadSubsystem.
	const FVector WorldPos = GetOwner()->GetActorLocation();
	State.U = WorldPos.X * 0.01f;  // cm → m
	State.V = WorldPos.Y * 0.01f;

	// Register with the world subsystem
	if (UWorld* World = GetWorld())
	{
		if (UTrafficRoadSubsystem* Sub = World->GetSubsystem<UTrafficRoadSubsystem>())
		{
			Sub->RegisterVehicle(this);
		}
	}
}

void UTrafficVehicleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld())
	{
		if (UTrafficRoadSubsystem* Sub = World->GetSubsystem<UTrafficRoadSubsystem>())
		{
			Sub->UnregisterVehicle(this);
		}
	}

	Super::EndPlay(EndPlayReason);
}

void UTrafficVehicleComponent::RebuildModel()
{
	// Resolve params: prefer Preset over inline values
	FLongModelParams ResolvedLong = LongParams;
	FMTMParams       ResolvedMTM  = MTMParams;
	ETrafficVehicleType ResolvedType = VehicleType;
	float ResolvedLen = VehicleLength;
	float ResolvedWid = VehicleWidth;

	if (IsValid(Preset))
	{
		ResolvedLong = Preset->LongParams;
		ResolvedMTM  = Preset->MTMParams;
		ResolvedType = Preset->VehicleType;
		ResolvedLen  = Preset->Length;
		ResolvedWid  = Preset->Width;
	}

	CachedModel.LongModel.Params = ResolvedLong;
	CachedModel.MTMParam         = ResolvedMTM;

	State.Type   = ResolvedType;
	State.Length = ResolvedLen;
	State.Width  = ResolvedWid;
}

void UTrafficVehicleComponent::SetLogicalPosition(float U, float V)
{
	State.U = U;
	State.V = V;
}

void UTrafficVehicleComponent::SetLogicalVelocity(float SpeedLong, float SpeedLat)
{
	State.SpeedLong = SpeedLong;
	State.SpeedLat  = SpeedLat;
}
