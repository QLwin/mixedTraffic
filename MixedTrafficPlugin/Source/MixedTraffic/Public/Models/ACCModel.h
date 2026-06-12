// Copyright QLwin. All Rights Reserved.
// ACCModel.h – Adaptive Cruise Control model (ACC / IDM+)
//
// Same interface as FIDMModel but uses the Constant Acceleration Heuristic
// (CAH) to produce smoother reactions to sudden braking.
//
// Reference: Kesting, Treiber & Helbing, Phil. Trans. R. Soc. A (2010) 4585.
//
// Ported from ACC class in models.js (Martin Treiber).
#pragma once

#include "CoreMinimal.h"
#include "Models/MTMTypes.h"

/**
 * ACC longitudinal model with CAH smoothing.
 *
 * Identical parameter set to IDM; adds a "Cool" blending factor.
 *
 * Usage:
 *   FACCModel Model;
 *   Model.Params  = FLongModelParams::CarDefaults();
 *   Model.Cool    = 0.99f;  // 1=pure CAH-blend, 0=pure IDM
 *   float Acc = Model.CalcAcc(Gap, OwnSpeed, LeaderSpeed, LeaderAcc);
 */
struct MIXEDTRAFFIC_API FACCModel
{
	FLongModelParams Params;

	/** Blending coefficient between CAH and IDM (default 0.99 → almost pure blend) */
	float Cool = 0.99f;

	// ── Free acceleration (identical to IDM) ──────────────────────────────
	FORCEINLINE float CalcAccFree(float V) const;

	// ── Interaction acceleration ───────────────────────────────────────────
	//
	// Uses IDM desired-gap formula, then blends with the CAH prediction to
	// reduce unnecessary braking.
	//
	// @param S   bumper-to-bumper gap [m]
	// @param V   own speed [m/s]
	// @param Vl  leader speed [m/s]
	// @param Al  leader acceleration [m/s²]  ← actively used here (unlike IDM)
	// @return    interaction acceleration [m/s²]  (≤ 0)
	FORCEINLINE float CalcAccInt(float S, float V, float Vl, float Al) const;

	// ── Combined total longitudinal acceleration ──────────────────────────
	FORCEINLINE float CalcAcc(float S, float V, float Vl, float Al) const;
};

// ---------------------------------------------------------------------------
// Inline implementations
// ---------------------------------------------------------------------------

FORCEINLINE float FACCModel::CalcAccFree(float V) const
{
	const float V0Eff = FMath::Max(0.01f, FMath::Min(Params.V0, Params.SpeedLimit));
	if (V < V0Eff)
	{
		const float R = V / V0Eff;
		const float R2 = R * R;
		return Params.A * (1.0f - R2 * R2);
	}
	return Params.A * (1.0f - V / V0Eff);
}

FORCEINLINE float FACCModel::CalcAccInt(float S, float V, float Vl, float Al) const
{
	// ── IDM desired gap ───────────────────────────────────────────────────
	const float DeltaV = V - Vl;
	const float SStar = Params.S0 + FMath::Max(0.0f,
		V * Params.T + 0.5f * V * DeltaV / FMath::Sqrt(Params.A * Params.B));

	const float AccFree = CalcAccFree(V);
	const float SFrac   = SStar / FMath::Max(S, 0.1f * Params.S0);
	const float AccIDM  = AccFree - Params.A * SFrac * SFrac;

	// ── CAH (Constant Acceleration Heuristic) ─────────────────────────────
	// Gives a more anticipatory response to the leader's acceleration.
	float AccCAH;
	if (Vl * (-DeltaV) < 2.0f * S * Al)  // Vl*(V-Vl) < -2*S*Al => branch 1
	{
		// Leader is decelerating fast; mirror kinematics
		const float Denom = Vl * Vl - 2.0f * S * Al;
		AccCAH = (FMath::Abs(Denom) > KINDA_SMALL_NUMBER)
			? V * V * Al / Denom
			: -Params.BMax;
	}
	else
	{
		AccCAH = Al - (DeltaV > 0.0f
			? DeltaV * DeltaV / (2.0f * FMath::Max(S, 0.1f))
			: 0.0f);
	}
	AccCAH = FMath::Min(AccCAH, Params.A);

	// ── Blend IDM and CAH ─────────────────────────────────────────────────
	const float AccMix = (AccIDM > AccCAH)
		? AccIDM
		: AccCAH + Params.B * FMath::Tanh((AccIDM - AccCAH) / Params.B);

	const float AccACC = Cool * AccMix + (1.0f - Cool) * AccIDM;
	return FMath::Max(-Params.BMax, AccACC - AccFree);
}

FORCEINLINE float FACCModel::CalcAcc(float S, float V, float Vl, float Al) const
{
	return CalcAccFree(V) + CalcAccInt(S, V, Vl, Al);
}
