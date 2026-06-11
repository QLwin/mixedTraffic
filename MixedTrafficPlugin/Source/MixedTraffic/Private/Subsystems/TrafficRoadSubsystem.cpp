// Copyright QLwin. All Rights Reserved.
#include "Subsystems/TrafficRoadSubsystem.h"
#include "Components/TrafficVehicleComponent.h"
#include "Models/MTMModel.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

// ---------------------------------------------------------------------------
// Constants (matching JS sim-straight.js defaults)
// ---------------------------------------------------------------------------

static constexpr float BoundaryMeasure  = 4.0f;  // assumed deceleration [m/s²]
static constexpr float TMeasure         = 1.0f;   // assumed time gap [s]
static constexpr float LMeasure         = 10.0f;  // assumed vehicle length [m]

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

void UTrafficRoadSubsystem::RegisterVehicle(UTrafficVehicleComponent* Vehicle)
{
	if (Vehicle && !Vehicles.Contains(Vehicle))
	{
		Vehicles.Add(Vehicle);
	}
}

void UTrafficRoadSubsystem::UnregisterVehicle(UTrafficVehicleComponent* Vehicle)
{
	Vehicles.RemoveSwap(Vehicle);
}

// ---------------------------------------------------------------------------
// Road geometry
// ---------------------------------------------------------------------------

void UTrafficRoadSubsystem::SetStraightRoad(float LeftWidth, float RightWidth)
{
	WidthLeftFn  = [LeftWidth] (float /*U*/) { return LeftWidth;  };
	WidthRightFn = [RightWidth](float /*U*/) { return RightWidth; };
}

void UTrafficRoadSubsystem::SetRoadBoundaryFunctions(TFunction<float(float)> InWidthLeft,
                                                      TFunction<float(float)> InWidthRight)
{
	WidthLeftFn  = MoveTemp(InWidthLeft);
	WidthRightFn = MoveTemp(InWidthRight);
}

float UTrafficRoadSubsystem::GetRoadWidth(float U) const
{
	if (!WidthLeftFn || !WidthRightFn) { return 0.0f; }
	return WidthLeftFn(U) + WidthRightFn(U);
}

// ---------------------------------------------------------------------------
// UTickableWorldSubsystem
// ---------------------------------------------------------------------------

void UTrafficRoadSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	UpdateSearchRanges();

	// Default: 7-metre half-width straight road (≈ 3.5m × 2 lanes each side)
	if (!WidthLeftFn)
	{
		SetStraightRoad(7.0f, 7.0f);
	}
}

bool UTrafficRoadSubsystem::IsTickable() const
{
	return Vehicles.Num() > 0;
}

TStatId UTrafficRoadSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UTrafficRoadSubsystem, STATGROUP_Tickables);
}

void UTrafficRoadSubsystem::Tick(float DeltaTime)
{
	// Remove stale weak pointers
	Vehicles.RemoveAll([](const TWeakObjectPtr<UTrafficVehicleComponent>& W)
	{
		return !W.IsValid();
	});

	if (Vehicles.IsEmpty()) { return; }

	// 1. Sort by u descending (leading vehicle first, matching JS convention)
	SortVehicles();

	// 2. Calculate all accelerations (parallel-safe: reads prev-tick state only)
	const int32 N = Vehicles.Num();
	ParallelFor(N, [this](int32 I)
	{
		if (Vehicles[I].IsValid())
		{
			CalcAccelerationsForVehicle(I);
		}
	});

	// 3. Integrate and update Actor transforms (GameThread only)
	for (TWeakObjectPtr<UTrafficVehicleComponent>& WeakVeh : Vehicles)
	{
		if (WeakVeh.IsValid())
		{
			ApplyIntegration(WeakVeh.Get(), DeltaTime);
		}
	}
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void UTrafficRoadSubsystem::UpdateSearchRanges()
{
	// Same formula as road.js constructor (dumaxLag / dumaxLead)
	const float V0Max2 = V0Max;
	DumaxLead = LMeasure + 1.0f * (V0Max2 * TMeasure + 0.5f * V0Max2 * V0Max2 / BoundaryMeasure);
	DumaxLag  = LMeasure + 0.5f * (V0Max2 * TMeasure + 0.5f * V0Max2 * V0Max2 / BoundaryMeasure);
}

void UTrafficRoadSubsystem::SortVehicles()
{
	Vehicles.Sort([](const TWeakObjectPtr<UTrafficVehicleComponent>& A,
	                  const TWeakObjectPtr<UTrafficVehicleComponent>& B)
	{
		const float UA = A.IsValid() ? A->State.U : -1e9f;
		const float UB = B.IsValid() ? B->State.U : -1e9f;
		return UA > UB;  // descending: largest U (furthest ahead) first
	});
}

void UTrafficRoadSubsystem::GetNeighbourRange(int32 I, int32& OutIMin, int32& OutIMax) const
{
	const int32 N = Vehicles.Num();
	const float U = Vehicles[I].IsValid() ? Vehicles[I]->State.U : 0.0f;

	// Leaders: scan toward index 0 (higher u)
	OutIMin = I;
	while (OutIMin > 0 && Vehicles[OutIMin - 1].IsValid() &&
	       (Vehicles[OutIMin - 1]->State.U - U) <= DumaxLead)
	{
		--OutIMin;
	}

	// Followers: scan toward index N-1 (lower u)
	OutIMax = I;
	while (OutIMax < N - 1 && Vehicles[OutIMax + 1].IsValid() &&
	       (U - Vehicles[OutIMax + 1]->State.U) <= DumaxLag)
	{
		++OutIMax;
	}
}

void UTrafficRoadSubsystem::CalcAccelerationsForVehicle(int32 I)
{
	UTrafficVehicleComponent* VehComp = Vehicles[I].Get();
	if (!VehComp) { return; }

	FTrafficVehicleState& S = VehComp->State;
	if (S.IsObstacle()) { return; }

	const FMTMModel& Model = VehComp->GetMTMModel();
	const float Wroad = GetRoadWidth(S.U);

	int32 IMin, IMax;
	GetNeighbourRange(I, IMin, IMax);

	// ── Longitudinal acceleration ─────────────────────────────────────────
	// Start with free acceleration (treated as "interaction with self")
	float AccLong = Model.CalcAccLongFree(S.SpeedLong);

	for (int32 J = IMin; J <= IMax; ++J)
	{
		if (J == I) { continue; }
		if (!Vehicles[J].IsValid()) { continue; }

		const FTrafficVehicleState& Sl = Vehicles[J]->State;
		const float Dx   = Sl.U - S.U;
		const float Dy   = Sl.V - S.V;
		const float Wavg = 0.5f * (S.Width + Sl.Width);

		if (Dx > 0.0f)  // leader
		{
			// Replace free acc component with full leader interaction
			const float AccWithLeader = Model.CalcAccLong(Dx, Dy,
			                               S.SpeedLong, Sl.SpeedLong, Sl.AccLong,
			                               Sl.Length, Wavg);
			AccLong = FMath::Min(AccLong, AccWithLeader);
		}
		else if (PushLong > 0.0f)  // follower pushes (Galilean invariance)
		{
			const float DxBack  = -Dx;
			const float BackInt = Model.CalcLeaderInteraction(DxBack, Dy,
			                          Sl.SpeedLong, S.SpeedLong, S.AccLong,
			                          S.Length, Wavg);
			AccLong += PushLong * BackInt;
		}
	}

	// ── Lateral acceleration ──────────────────────────────────────────────
	float AccLat = Model.CalcAccLatFree(S.SpeedLat);

	for (int32 J = IMin; J <= IMax; ++J)
	{
		if (J == I) { continue; }
		if (!Vehicles[J].IsValid()) { continue; }

		const FTrafficVehicleState& Sl = Vehicles[J]->State;
		const float Dx = Sl.U - S.U;

		if (Dx >= 0.0f || PushLat > 0.0f)  // leader or (follower with push)
		{
			const float Contribution = Model.CalcAccLatInt(
				S.U,  Sl.U,
				S.V,  Sl.V,
				S.SpeedLong, Sl.SpeedLong,
				S.SpeedLat,  Sl.SpeedLat,
				Sl.AccLong,
				S.Length,  Sl.Length,
				S.Width,   Sl.Width,
				Wroad);

			AccLat += (Dx >= 0.0f) ? Contribution : PushLat * Contribution;
		}
	}

	// ── Boundary acceleration ─────────────────────────────────────────────
	if (WidthLeftFn && WidthRightFn)
	{
		const FVector2f AccB = Model.CalcAccB(
			WidthLeftFn, WidthRightFn,
			S.U, S.V, S.SpeedLong, S.SpeedLat, S.Width, V0Max);
		AccLong += AccB.X;
		AccLat  += AccB.Y;
	}

	// ── Maximum lateral speed constraint ─────────────────────────────────
	// Prevent vehicles from drifting at extreme angles to the road axis
	const float MaxLatSpeed = S.SpeedLong * 0.5f + 0.5f;  // basic heuristic
	if ((AccLat > 0.0f && S.SpeedLat >= MaxLatSpeed) ||
	    (AccLat < 0.0f && S.SpeedLat <= -MaxLatSpeed))
	{
		AccLat = 0.0f;
	}

	// Write back (atomic w.r.t. the parallel loop: only writes own index)
	S.AccLong = AccLong;
	S.AccLat  = AccLat;
}

void UTrafficRoadSubsystem::ApplyIntegration(UTrafficVehicleComponent* Veh, float Dt) const
{
	if (!Veh) { return; }
	FTrafficVehicleState& S = Veh->State;

	// Ballistic integration (2nd-order position, 1st-order velocity)
	BallisticUpdate(S, Dt);

	// Push world position back to the Actor (straight road: X=u, Y=v, Z=const)
	// M3 will replace this with proper spline projection.
	AActor* Owner = Veh->GetOwner();
	if (!Owner) { return; }

	const float CurrentZ = Owner->GetActorLocation().Z;
	const FVector NewLocation(S.U * 100.0f, S.V * 100.0f, CurrentZ);  // m → cm
	const float   Yaw       = FMath::RadiansToDegrees(FMath::Atan2(S.SpeedLat, FMath::Max(S.SpeedLong, 0.01f)));
	const FRotator NewRot(0.0f, Yaw, 0.0f);

	Owner->SetActorLocationAndRotation(NewLocation, NewRot, false, nullptr, ETeleportType::TeleportPhysics);
}
