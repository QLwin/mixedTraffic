// Copyright QLwin. All Rights Reserved.
#include "Settings/TrafficSettings.h"

UTrafficSettings::UTrafficSettings()
{
	// Defaults are set in the header; ini file can override them.
}

FMTMParams UTrafficSettings::MakeDefaultMTMParams() const
{
	FMTMParams P;
	P.AccLatBMax    = AccLatBMax;
	P.AccLatBRef    = AccLatBRef;
	P.AccLongBRef   = AccLongBRef;
	P.AnticFactorB  = AnticFactorB;
	P.S0yB          = S0yB;
	P.S0yLatB       = S0yLatB;
	// Lateral interaction defaults (not exposed at project level; use presets)
	return P;
}
