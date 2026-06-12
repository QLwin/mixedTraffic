// Copyright QLwin. All Rights Reserved.
#include "MixedTrafficEditor.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_MODULE(FMixedTrafficEditorModule, MixedTrafficEditor)

void FMixedTrafficEditorModule::StartupModule()
{
	// Editor-only initialisation:
	// - register custom asset type actions (TrafficVehiclePreset)
	// - register detail customisations
	// Implemented in M5.
}

void FMixedTrafficEditorModule::ShutdownModule()
{
}
