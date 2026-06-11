// Copyright QLwin. All Rights Reserved.
// MTMTypes.h – shared data structures for the Mixed Traffic plugin.
//
// All physical quantities use SI units: meters [m], seconds [s], m/s, m/s².
// Logical road coordinates:
//   u  – arc-length along road axis (longitudinal) [m]
//   v  – lateral offset from road centre, positive to the RIGHT [m]
//
// Ported from the JavaScript simulation by Martin Treiber
// (https://github.com/mtreiber/mixedTraffic).
// Reference: Kanagaraj & Treiber, Physica A 509 (2018) 1–11.
#pragma once

#include "CoreMinimal.h"
#include "MTMTypes.generated.h"

// ---------------------------------------------------------------------------
// Vehicle type enum
// ---------------------------------------------------------------------------

UENUM(BlueprintType)
enum class ETrafficVehicleType : uint8
{
	Car      UMETA(DisplayName = "Car"),
	Truck    UMETA(DisplayName = "Truck"),
	Bike     UMETA(DisplayName = "Bike / Motorbike"),
	Obstacle UMETA(DisplayName = "Static Obstacle"),
};

// ---------------------------------------------------------------------------
// Longitudinal model parameters (shared by IDM and ACC)
// ---------------------------------------------------------------------------

USTRUCT(BlueprintType)
struct MIXEDTRAFFIC_API FLongModelParams
{
	GENERATED_BODY()

	/** Desired (free-flow) speed [m/s] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal", meta=(ClampMin="0.1", ClampMax="100"))
	float V0 = 13.9f;   // ~50 km/h

	/** Desired time gap to leader [s] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal", meta=(ClampMin="0.1", ClampMax="10"))
	float T = 1.0f;

	/** Minimum bumper-to-bumper gap at standstill [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal", meta=(ClampMin="0.0", ClampMax="20"))
	float S0 = 2.0f;

	/** Maximum acceleration [m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal", meta=(ClampMin="0.1", ClampMax="10"))
	float A = 1.4f;

	/** Comfortable (target) deceleration [m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal", meta=(ClampMin="0.1", ClampMax="10"))
	float B = 2.0f;

	/** Active speed limit; values ≥ 1000 mean "no limit" [m/s] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal")
	float SpeedLimit = 1000.f;

	/** Hard maximum deceleration [m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Longitudinal", meta=(ClampMin="1", ClampMax="20"))
	float BMax = 9.0f;

	// ── Presets ──────────────────────────────────────────────────────────

	static FLongModelParams CarDefaults()
	{
		FLongModelParams P;
		P.V0=25.f; P.T=1.0f; P.S0=2.0f; P.A=2.0f; P.B=2.0f;
		return P;
	}
	static FLongModelParams TruckDefaults()
	{
		FLongModelParams P;
		P.V0=12.f; P.T=1.5f; P.S0=2.0f; P.A=1.0f; P.B=2.0f;
		return P;
	}
	static FLongModelParams BikeDefaults()
	{
		FLongModelParams P;
		P.V0=25.f; P.T=0.5f; P.S0=1.5f; P.A=3.0f; P.B=2.0f;
		return P;
	}
};

// ---------------------------------------------------------------------------
// MTM lateral / boundary parameters
// ---------------------------------------------------------------------------

USTRUCT(BlueprintType)
struct MIXEDTRAFFIC_API FMTMParams
{
	GENERATED_BODY()

	/** Lateral attenuation scale for longitudinal vehicle-vehicle interaction [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Lateral")
	float S0y = 0.15f;

	/** Lateral attenuation scale for lateral vehicle-vehicle interaction [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Lateral")
	float S0yLat = 0.60f;

	/** Longitudinal attenuation scale for wall-vehicle interaction [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary")
	float S0yB = 0.15f;

	/** Lateral attenuation scale for wall-vehicle interaction [m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary")
	float S0yLatB = 0.20f;

	/** Sensitivity: (max desired lateral speed) / (longitudinal acceleration) [s] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Lateral", meta=(ClampMin="0.1", ClampMax="10"))
	float SensLat = 1.0f;

	/** Lateral OVM relaxation time [s] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Lateral", meta=(ClampMin="0.1", ClampMax="20"))
	float TauLatOVM = 3.0f;

	/** FVDM-like lateral relative-speed sensitivity [s/m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Lateral", meta=(ClampMin="0.0", ClampMax="10"))
	float SensDvy = 1.0f;

	/** Maximum lateral acceleration from boundary [m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary", meta=(ClampMin="1", ClampMax="50"))
	float AccLatBMax = 20.0f;

	/** Lateral acceleration when vehicle edge touches boundary [m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary")
	float AccLatBRef = 15.0f;

	/** Longitudinal deceleration when vehicle edge touches boundary [m/s²] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary")
	float AccLongBRef = 0.2f;

	/** Anticipation lookahead as a multiple of the vehicle's time gap T */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary", meta=(ClampMin="0.5", ClampMax="5"))
	float AnticFactorB = 2.0f;

	/** Number of discretisation steps for boundary anticipation integral */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MTM|Boundary", meta=(ClampMin="1", ClampMax="32"))
	int32 Nj = 8;
};

// ---------------------------------------------------------------------------
// Runtime state of a single vehicle (used by the subsystem each tick)
// ---------------------------------------------------------------------------

USTRUCT(BlueprintType)
struct MIXEDTRAFFIC_API FTrafficVehicleState
{
	GENERATED_BODY()

	/** Vehicle type (determines model parameters) */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	ETrafficVehicleType Type = ETrafficVehicleType::Car;

	/** Vehicle length [m] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float Length = 5.0f;

	/** Vehicle width [m] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float Width = 2.0f;

	/** Longitudinal position (arc-length from road start) [m] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float U = 0.0f;

	/** Lateral position (offset from road centre, +right) [m] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float V = 0.0f;

	/** Longitudinal speed [m/s] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float SpeedLong = 0.0f;

	/** Lateral speed [m/s] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float SpeedLat = 0.0f;

	/** Longitudinal acceleration (output of model, set each tick) [m/s²] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float AccLong = 0.0f;

	/** Lateral acceleration (output of model, set each tick) [m/s²] */
	UPROPERTY(BlueprintReadOnly, Category="Traffic")
	float AccLat = 0.0f;

	bool IsObstacle() const { return Type == ETrafficVehicleType::Obstacle; }
};

// ---------------------------------------------------------------------------
// Ballistic (2nd-order) position/velocity update
// Ref: Treiber & Kanagaraj, Physica A 419 (2015) 183–195
// ---------------------------------------------------------------------------

/**
 * Applies one ballistic integration step in-place.
 * velocityVector(t+dt) = v(t) + a(t)*dt
 * posVector(t+dt)      = pos(t) + v(t)*dt + 0.5*a(t)*dt²
 */
inline void BallisticUpdate(FTrafficVehicleState& State, float Dt)
{
	if (State.IsObstacle()) { return; }

	// Position update (2nd order)
	State.U += State.SpeedLong * Dt + 0.5f * State.AccLong * Dt * Dt;
	State.V += State.SpeedLat  * Dt + 0.5f * State.AccLat  * Dt * Dt;

	// Velocity update (1st order Euler)
	State.SpeedLong = FMath::Max(0.0f, State.SpeedLong + State.AccLong * Dt);
	State.SpeedLat  = State.SpeedLat + State.AccLat * Dt;
}
