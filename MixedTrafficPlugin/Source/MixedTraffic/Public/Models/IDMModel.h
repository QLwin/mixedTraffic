// Copyright QLwin. All Rights Reserved.
// IDMModel.h – Intelligent Driver Model (IDM)
//
// Longitudinal car-following model.
// Reference: Treiber, Hennecke & Helbing, Phys. Rev. E 62 (2000) 1805.
//
// Ported from IDM class in models.js (Martin Treiber).
#pragma once

#include "CoreMinimal.h"
#include "Models/MTMTypes.h"

/**
 * Intelligent Driver Model (IDM) – pure C++, no UObject overhead.
 *
 * Usage:
 *   FIDMModel Model;
 *   Model.Params = FLongModelParams::CarDefaults();
 *   float Acc = Model.CalcAcc(Gap, OwnSpeed, LeaderSpeed, LeaderAcc);
 */
struct MIXEDTRAFFIC_API FIDMModel
{
	FLongModelParams Params;

	// ── Free acceleration (no leader) ─────────────────────────────────────
	//
	// accFree = A * (1 - (v/v0)^4)   if v < v0
	//         = A * (1 - v/v0)       if v >= v0
	//
	// @param V   own longitudinal speed [m/s]
	// @return    free acceleration [m/s²]
	FORCEINLINE float CalcAccFree(float V) const;

	// ── Interaction (following) acceleration ──────────────────────────────
	//
	// Returns only the interaction term (negative or zero), NOT including
	// free acceleration.  Total acc = CalcAccFree(V) + CalcAccInt(...)
	//
	// @param S   bumper-to-bumper gap to leader [m]
	// @param V   own longitudinal speed [m/s]
	// @param Vl  leader's longitudinal speed [m/s]
	// @param Al  leader's longitudinal acceleration [m/s²] (unused in IDM,
	//            present for interface compatibility with ACC)
	// @return    interaction acceleration [m/s²]  (≤ 0)
	FORCEINLINE float CalcAccInt(float S, float V, float Vl, float Al) const;

	// ── Combined total longitudinal acceleration ──────────────────────────
	//
	// Convenience wrapper: CalcAccFree(V) + CalcAccInt(S,V,Vl,Al)
	//
	// @param S   bumper-to-bumper gap to leader [m]  (pass large value for
	//            free flow, e.g. 1e6f)
	// @param V   own speed [m/s]
	// @param Vl  leader speed [m/s]
	// @param Al  leader acceleration [m/s²]
	// @return    total acceleration [m/s²], clamped to [-BMax, +A]
	FORCEINLINE float CalcAcc(float S, float V, float Vl, float Al) const;
};

// ---------------------------------------------------------------------------
// Inline implementations
// ---------------------------------------------------------------------------

FORCEINLINE float FIDMModel::CalcAccFree(float V) const
{
	const float V0Eff = FMath::Max(0.01f, FMath::Min(Params.V0, Params.SpeedLimit));
	if (V < V0Eff)
	{
		// Avoid FMath::Pow for the common case: (v/v0)^4 = r*r where r=(v/v0)^2
		const float R = V / V0Eff;
		const float R2 = R * R;
		return Params.A * (1.0f - R2 * R2);
	}
	return Params.A * (1.0f - V / V0Eff);
}

FORCEINLINE float FIDMModel::CalcAccInt(float S, float V, float Vl, float Al) const
{
	// Desired gap: s*(v, Δv) = s0 + max(0, v·T + v·Δv / (2·sqrt(a·b)))
	const float DeltaV = V - Vl;
	const float SStar = Params.S0 + FMath::Max(0.0f,
		V * Params.T + 0.5f * V * DeltaV / FMath::Sqrt(Params.A * Params.B));

	// Gap floor prevents division by zero; 0.1*s0 matches JS original
	const float SFrac = SStar / FMath::Max(S, 0.1f * Params.S0);
	return FMath::Max(-Params.BMax, -Params.A * SFrac * SFrac);
}

FORCEINLINE float FIDMModel::CalcAcc(float S, float V, float Vl, float Al) const
{
	return CalcAccFree(V) + CalcAccInt(S, V, Vl, Al);
}
