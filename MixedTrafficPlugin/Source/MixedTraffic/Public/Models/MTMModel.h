// Copyright QLwin. All Rights Reserved.
// MTMModel.h – Mixed Traffic Flow Model (MTM)
//
// 2-D generalisation of IDM/ACC to directed mixed traffic.
// Computes longitudinal AND lateral accelerations for each vehicle
// based on nearby vehicles and road boundaries.
//
// Reference: Kanagaraj & Treiber, Physica A 509 (2018) 1–11.
//            arXiv:1805.05076
//
// Ported from MTM class in models.js (Martin Treiber).
#pragma once

#include "CoreMinimal.h"
#include "Models/MTMTypes.h"
#include "Models/ACCModel.h"   // default underlying longitudinal model

/**
 * FMTMModel – 2-D Mixed Traffic Model.
 *
 * Contains an underlying longitudinal car-following model (FACCModel by
 * default) plus lateral interaction and boundary terms.
 *
 * All methods take pure scalar inputs and return scalar outputs so they
 * can be called in parallel (ParallelFor) without any shared mutable state.
 *
 * Usage:
 *   FMTMModel Model;
 *   Model.LongModel.Params = FLongModelParams::CarDefaults();
 *   Model.MTMParam = {};   // use defaults
 *
 *   // Per neighbour:
 *   float AccLong += Model.CalcAccLong(dx, dy, vx, vxl, axl, Ll, Wavg);
 *   float AccLat  += Model.CalcAccLatInt(x,xl, y,yl, vx,vxl, vy,vyl, axl,
 *                                        Lveh,Ll, Wveh,Wl, Wroad);
 *
 *   // Road boundaries:
 *   FVector2f AccB = Model.CalcAccB(WidthLeftFn, WidthRightFn, x,y,vx,vy,Wveh, V0Max);
 */
struct MIXEDTRAFFIC_API FMTMModel
{
	/** Underlying longitudinal car-following model */
	FACCModel LongModel;

	/** MTM-specific lateral and boundary parameters */
	FMTMParams MTMParam;

	/** Derived cap: max lateral interaction accel = 4 × comfortable decel */
	float GetAccLatIntMax() const { return 4.0f * LongModel.Params.B; }

	// ── Free accelerations ────────────────────────────────────────────────

	/** Free longitudinal acceleration (no leader present) [m/s²] */
	FORCEINLINE float CalcAccLongFree(float Vx) const;

	/** Lateral restoring acceleration toward v=0 when lateral speed ≠ 0 [m/s²] */
	FORCEINLINE float CalcAccLatFree(float Vy) const;

	// ── Leader interaction: longitudinal ──────────────────────────────────

	/**
	 * Longitudinal acceleration induced by ONE other vehicle.
	 *
	 * Includes lateral attenuation: interaction falls off as the leader moves
	 * to the side, with scale s0y.  Only vehicles ahead (dx > 0) contribute.
	 *
	 * @param Dx    longitudinal distance = u_leader − u_self  [m]
	 * @param Dy    lateral distance      = v_leader − v_self  [m]
	 * @param Vx    own longitudinal speed [m/s]
	 * @param Vxl   leader longitudinal speed [m/s]
	 * @param Axl   leader longitudinal acceleration [m/s²]
	 * @param Ll    leader length [m]
	 * @param Wavg  average half-width  0.5*(W_own + W_leader) [m]
	 * @return      total longitudinal acceleration [m/s²]
	 */
	float CalcAccLong(float Dx, float Dy,
	                  float Vx, float Vxl, float Axl,
	                  float Ll, float Wavg) const;

	/**
	 * Longitudinal interaction term only (CalcAccLong − CalcAccLongFree).
	 * Useful for backwards-interaction (follower on self) calculations.
	 */
	float CalcLeaderInteraction(float Dx, float Dy,
	                            float Vx, float Vxl, float Axl,
	                            float Ll, float Wavg) const;

	// ── Leader interaction: lateral ───────────────────────────────────────

	/**
	 * Lateral acceleration induced by ONE other vehicle (leader or follower).
	 *
	 * @param X     own longitudinal position [m]
	 * @param Xl    other vehicle longitudinal position [m]
	 * @param Y     own lateral position [m]
	 * @param Yl    other vehicle lateral position [m]
	 * @param Vx    own longitudinal speed [m/s]
	 * @param Vxl   other vehicle longitudinal speed [m/s]
	 * @param Vy    own lateral speed [m/s]
	 * @param Vyl   other vehicle lateral speed [m/s]
	 * @param Axl   other vehicle longitudinal acceleration [m/s²]
	 * @param Lveh  own vehicle length [m]
	 * @param Ll    other vehicle length [m]
	 * @param Wveh  own vehicle width [m]
	 * @param Wl    other vehicle width [m]
	 * @param Wroad total road width [m]
	 * @return      lateral acceleration contribution [m/s²]
	 */
	float CalcAccLatInt(float X,  float Xl,
	                    float Y,  float Yl,
	                    float Vx, float Vxl,
	                    float Vy, float Vyl,
	                    float Axl,
	                    float Lveh, float Ll,
	                    float Wveh, float Wl,
	                    float Wroad) const;

	// ── Road boundary interaction ─────────────────────────────────────────

	/**
	 * Longitudinal and lateral acceleration induced by road boundaries.
	 *
	 * Uses forward-anticipation: evaluates boundary gaps at nj positions
	 * along the anticipated trajectory, weighted by an exponential kernel.
	 *
	 * @param WidthLeftFn   function: arc-length u → left half-width [m]
	 * @param WidthRightFn  function: arc-length u → right half-width [m]
	 * @param X    own arc-length position [m]
	 * @param Y    own lateral position [m]
	 * @param Vx   own longitudinal speed [m/s]
	 * @param Vy   own lateral speed [m/s]
	 * @param Wveh own vehicle width [m]
	 * @param V0Max maximum desired speed on this road (used for scaling) [m/s]
	 * @return     FVector2f{ AccLong_boundary, AccLat_boundary } [m/s²]
	 */
	FVector2f CalcAccB(TFunctionRef<float(float)> WidthLeftFn,
	                   TFunctionRef<float(float)> WidthRightFn,
	                   float X, float Y,
	                   float Vx, float Vy,
	                   float Wveh,
	                   float V0Max) const;

private:
	// Dimensionless longitudinal boundary attenuation as f(gap to boundary)
	FORCEINLINE float AlphaLongB(float Sy) const;
	// Dimensionless lateral boundary attenuation (linearly increasing if exceeded)
	FORCEINLINE float AlphaLatB(float Sy) const;
};

// ---------------------------------------------------------------------------
// Inline helpers
// ---------------------------------------------------------------------------

FORCEINLINE float FMTMModel::CalcAccLongFree(float Vx) const
{
	return LongModel.CalcAccFree(Vx);
}

FORCEINLINE float FMTMModel::CalcAccLatFree(float Vy) const
{
	// Lateral OVM: drives lateral speed back to zero
	return -Vy / MTMParam.TauLatOVM;
}

FORCEINLINE float FMTMModel::AlphaLongB(float Sy) const
{
	return (Sy > 0.0f) ? FMath::Exp(-Sy / MTMParam.S0yB) : 1.0f;
}

FORCEINLINE float FMTMModel::AlphaLatB(float Sy) const
{
	return (Sy > 0.0f) ? FMath::Exp(-Sy / MTMParam.S0yLatB)
	                   : 1.0f - Sy / MTMParam.S0yLatB;  // linearly larger if violated
}
