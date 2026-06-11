// Copyright QLwin. All Rights Reserved.
// MTMModel.cpp – non-trivial method implementations for FMTMModel.
//
// Ported from MTM.prototype.calcAccLong / calcAccLatInt / calcAccB
// in models.js (Martin Treiber).
#include "Models/MTMModel.h"

// ---------------------------------------------------------------------------
// CalcAccLong
// ---------------------------------------------------------------------------

float FMTMModel::CalcAccLong(float Dx, float Dy,
                              float Vx, float Vxl, float Axl,
                              float Ll, float Wavg) const
{
	// Bumper-to-bumper gap and signed lateral gap to leader edge
	const float Sx = FMath::Max(0.0f, Dx - Ll);
	const float Sy = FMath::Abs(Dy) - Wavg;  // >0 if lanes are separate

	const float AccFree    = LongModel.CalcAccFree(Vx);
	const float AccCFInt   = LongModel.CalcAccInt(Sx, Vx, Vxl, Axl);

	// Lateral attenuation: interaction decays when leader is to the side
	float Alpha = FMath::Min(FMath::Exp(-Sy / MTMParam.S0y), 1.0f);

	// Suppress longitudinal interaction when vehicles are side-by-side
	// and NOT in collision (both conditions: dx < Ll means overlap in length,
	// sy > 0 means no lateral overlap)
	if ((Dx < Ll) && (Sy > 0.0f))
	{
		Alpha = 0.0f;  // longParReductFactor = 0 in JS original
	}

	return AccFree + Alpha * AccCFInt;
}

// ---------------------------------------------------------------------------
// CalcLeaderInteraction (interaction term only, no free acc)
// ---------------------------------------------------------------------------

float FMTMModel::CalcLeaderInteraction(float Dx, float Dy,
                                        float Vx, float Vxl, float Axl,
                                        float Ll, float Wavg) const
{
	const float Sx    = FMath::Max(0.0f, Dx - Ll);
	const float Sy    = FMath::Abs(Dy) - Wavg;
	const float S0Max = FMath::Max(MTMParam.S0y, MTMParam.S0yLat);
	const float Alpha = FMath::Min(FMath::Exp(-Sy / S0Max), 1.0f);
	return Alpha * LongModel.CalcAccInt(Sx, Vx, Vxl, Axl);
}

// ---------------------------------------------------------------------------
// CalcAccLatInt
// ---------------------------------------------------------------------------

float FMTMModel::CalcAccLatInt(float X,  float Xl,
                                float Y,  float Yl,
                                float Vx, float Vxl,
                                float Vy, float Vyl,
                                float Axl,
                                float Lveh, float Ll,
                                float Wveh, float Wl,
                                float Wroad) const
{
	const float Dx    = Xl - X;
	const float Sx    = FMath::Max(0.0f, Dx - Ll);
	const float Dy    = Yl - Y;
	const float SignDy = (Dy < 0.0f) ? -1.0f : 1.0f;
	const float Wavg  = 0.5f * (Wveh + Wl);

	const bool bOverlap = FMath::Abs(Dy) < Wavg;

	// Longitudinal interaction term (reused for desired lateral speed)
	const float AccCFInt = LongModel.CalcAccInt(Sx, Vx, Vxl, Axl);

	// ── Normalised lateral desire alpha ∈ [-1, 1] ─────────────────────────
	// Points away from the other vehicle (negative = move left, positive = right)
	float Alpha;
	if (bOverlap)
	{
		// Linear/sqrt attenuation within lateral overlap zone
		const float RelDy = FMath::Abs(Dy) / Wavg;
		Alpha = -SignDy * FMath::Sqrt(RelDy);
	}
	else
	{
		Alpha = -SignDy * FMath::Exp(-(FMath::Abs(Dy) - Wavg) / MTMParam.S0yLat);
	}

	// ── Override alpha if leader is trapped near a boundary ───────────────
	// Prevents the ego vehicle from being stuck behind a slow vehicle that
	// hugs one side of the road.
	if (bOverlap)
	{
		const float SylbRight = 0.5f * Wroad - Yl - 0.5f * Wl; // leader→right-bd gap
	const float SylbLeft  = Wroad - SylbRight - Wl;          // leader→left-bd gap
		const bool bTooNarrowRight = SylbRight < Wveh + MTMParam.S0yLatB;
		const bool bTooNarrowLeft  = SylbLeft  < Wveh + MTMParam.S0yLatB;

		if (!(bTooNarrowRight && bTooNarrowLeft))  // skip on fully narrow roads
		{
			if (bTooNarrowRight && (Y > Yl)) { Alpha = -1.0f; }
			if (bTooNarrowLeft  && (Y < Yl)) { Alpha =  1.0f; }
		}
	}

	// ── Desired lateral speed ─────────────────────────────────────────────
	// v0LatInt = -SensLat * alpha * accCFint   (accCFint ≤ 0, so v0LatInt has
	// sign that points away from the other vehicle when they're too close)
	const float V0LatInt = -MTMParam.SensLat * Alpha * AccCFInt;

	// FVDM-like multiplicative factor: reduces lateral push when already
	// moving away from the other vehicle
	float MultDvFactor;
	if (bOverlap)
	{
		MultDvFactor = 1.0f;
	}
	else
	{
		MultDvFactor = FMath::Max(0.0f, 1.0f - MTMParam.SensDvy * SignDy * (Vyl - Vy));
	}

	// Lateral acceleration toward desired lateral speed
	float AccLatInt = V0LatInt / MTMParam.TauLatOVM * MultDvFactor;

	// Clamp to ±accLatIntMax
	const float Cap = GetAccLatIntMax();
	AccLatInt = FMath::Clamp(AccLatInt, -Cap, Cap);

	return AccLatInt;
}

// ---------------------------------------------------------------------------
// CalcAccB
// ---------------------------------------------------------------------------

FVector2f FMTMModel::CalcAccB(TFunctionRef<float(float)> WidthLeftFn,
                               TFunctionRef<float(float)> WidthRightFn,
                               float X, float Y,
                               float Vx, float Vy,
                               float Wveh,
                               float V0Max) const
{
	const float Tantic  = MTMParam.AnticFactorB * LongModel.Params.T;
	const float DTantic = (MTMParam.Nj > 1) ? 2.0f * Tantic / MTMParam.Nj : Tantic;

	float AlphaLongLeftMax  = 0.0f;
	float AlphaLongRightMax = 0.0f;
	float AlphaLatLeftMax   = 0.0f;
	float AlphaLatRightMax  = 0.0f;
	float V0yBLeft  = 0.0f;
	float V0yBRight = 0.0f;

	for (int32 J = 0; J < MTMParam.Nj; ++J)
	{
		const float TTC    = J * DTantic;
		const float Weight = FMath::Exp(-TTC / FMath::Max(Tantic, KINDA_SMALL_NUMBER));
		const float Uantic = X + Vx * TTC;

		// Signed gap from vehicle edge to boundary (positive = safe clearance)
		const float SyLeft  = WidthLeftFn(Uantic)  + Y - 0.5f * Wveh;  // v positive→right
		const float SyRight = WidthRightFn(Uantic) - Y - 0.5f * Wveh;

		// Derive required lateral speed to avoid boundary violation
		if (J > 0 && TTC > KINDA_SMALL_NUMBER)
		{
			V0yBLeft  = FMath::Max(V0yBLeft,  -SyLeft  / TTC);
			V0yBRight = FMath::Min(V0yBRight, +SyRight / TTC);
		}

		AlphaLongLeftMax  = FMath::Max(AlphaLongB(SyLeft)  * Weight, AlphaLongLeftMax);
		AlphaLongRightMax = FMath::Max(AlphaLongB(SyRight) * Weight, AlphaLongRightMax);
		AlphaLatLeftMax   = FMath::Max(AlphaLatB(SyLeft)   * Weight, AlphaLatLeftMax);
		AlphaLatRightMax  = FMath::Max(AlphaLatB(SyRight)  * Weight, AlphaLatRightMax);
	}

	// Pick the more demanding lateral evasion requirement
	const float V0y = (FMath::Abs(V0yBLeft) > FMath::Abs(V0yBRight)) ? V0yBLeft : V0yBRight;

	// Longitudinal deceleration (symmetric: both boundaries slow vehicle)
	float AccLongB = MTMParam.AccLongBRef * (-AlphaLongLeftMax - AlphaLongRightMax);
	// Scale with normalised speed so effect weakens at low speeds
	if (V0Max > KINDA_SMALL_NUMBER) { AccLongB *= Vx / V0Max; }

	// Lateral: direct alpha term plus OVM component from evasion velocity
	const float AccLatB0 = MTMParam.AccLatBRef * (AlphaLatLeftMax - AlphaLatRightMax);
	const float AccLatB  = AccLatB0 + V0y / FMath::Max(MTMParam.TauLatOVM, KINDA_SMALL_NUMBER);

	const float AccLatBClamped = FMath::Clamp(AccLatB, -MTMParam.AccLatBMax, MTMParam.AccLatBMax);

	return FVector2f(AccLongB, AccLatBClamped);
}
