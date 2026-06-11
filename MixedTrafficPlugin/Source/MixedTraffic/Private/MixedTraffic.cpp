// Copyright QLwin. All Rights Reserved.
#include "MixedTraffic.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_MODULE(FMixedTrafficModule, MixedTraffic)

void FMixedTrafficModule::StartupModule()
{
	// Nothing to initialize at module startup.
	// Vehicle models are value types (FIDMModel etc.) created per-vehicle.
}

void FMixedTrafficModule::ShutdownModule()
{
}
