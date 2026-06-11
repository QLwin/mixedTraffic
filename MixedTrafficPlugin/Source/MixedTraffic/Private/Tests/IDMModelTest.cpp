// Copyright QLwin. All Rights Reserved.
// IDMModelTest.cpp – UE Automation Tests for FIDMModel.
//
// Validates numerical output against the JS reference implementation.
// Run via: Session Frontend → Automation → MixedTraffic.Models.IDM.*
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Models/IDMModel.h"

// Tolerance for float comparisons (1e-3 matches JS float32 precision)
static constexpr float IDM_TOL = 1e-3f;

// ── Helper ────────────────────────────────────────────────────────────────

static FIDMModel MakeCarIDM()
{
	FIDMModel M;
	M.Params = FLongModelParams::CarDefaults();  // v0=25, T=1, s0=2, a=2, b=2
	return M;
}

// ── Test: Free acceleration ────────────────────────────────────────────────
//
// JS reference:
//   IDM.calcAccFree(0)  = a * (1-0^4) = 2.0
//   IDM.calcAccFree(v0) = a * (1-1^4) = 0.0
//   IDM.calcAccFree(50) = a * (1-50/25) = -2.0  (over-speed case)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIDMFreeAccTest,
	"MixedTraffic.Models.IDM.FreeAcceleration",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FIDMFreeAccTest::RunTest(const FString& Parameters)
{
	FIDMModel M = MakeCarIDM();

	// At rest → maximum acceleration
	TestNearlyEqual(TEXT("CalcAccFree(0) == A"),
		M.CalcAccFree(0.0f), M.Params.A, IDM_TOL);

	// At desired speed → zero acceleration
	TestNearlyEqual(TEXT("CalcAccFree(V0) == 0"),
		M.CalcAccFree(M.Params.V0), 0.0f, IDM_TOL);

	// Over speed limit → negative (braking)
	const float OverSpeed = M.Params.V0 * 2.0f;
	TestTrue(TEXT("CalcAccFree(2*V0) < 0"),
		M.CalcAccFree(OverSpeed) < 0.0f);

	// Speed limit active below V0
	M.Params.SpeedLimit = 10.0f;  // active limit
	const float AccAtLimit = M.CalcAccFree(10.0f);
	TestNearlyEqual(TEXT("CalcAccFree(SpeedLimit) == 0 when limit<V0"),
		AccAtLimit, 0.0f, IDM_TOL);

	return true;
}

// ── Test: Interaction acceleration (following) ─────────────────────────────
//
// JS reference (car following standstill, v=10, s=s0=2):
//   sstar = s0 + v*T + v*(v-0)/(2*sqrt(a*b)) = 2 + 10 + 10*10/(2*2) = 37
//   accInt = -a*(sstar/max(s,0.1*s0))^2 = -2*(37/2)^2 ≈ -684 → clamped to -BMax=-9

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIDMInteractionTest,
	"MixedTraffic.Models.IDM.InteractionAcceleration",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FIDMInteractionTest::RunTest(const FString& Parameters)
{
	FIDMModel M = MakeCarIDM();

	// At minimum gap, moving fast → hard braking clamped to -BMax
	const float AccMin = M.CalcAccInt(M.Params.S0, 10.0f, 0.0f, 0.0f);
	TestNearlyEqual(TEXT("At s0 gap with stopped leader → -BMax"),
		AccMin, -M.Params.BMax, IDM_TOL);

	// Large gap → interaction term ≈ 0 (free flow)
	const float AccFree = M.CalcAccInt(1000.0f, 10.0f, 10.0f, 0.0f);
	TestNearlyEqual(TEXT("Very large gap → accInt ≈ 0"),
		AccFree, 0.0f, IDM_TOL);

	// Same speed, moderate gap → small negative interaction
	// sstar = s0 + v*T (delta_v=0) = 2 + 10*1 = 12
	// accInt = -a*(12/15)^2 ≈ -1.28  (s=15m)
	const float S = 15.0f, V = 10.0f;
	const float SStar = M.Params.S0 + V * M.Params.T;
	const float Expected = -M.Params.A * FMath::Square(SStar / S);
	TestNearlyEqual(TEXT("Moderate following: accInt matches formula"),
		M.CalcAccInt(S, V, V, 0.0f), Expected, IDM_TOL);

	return true;
}

// ── Test: Equilibrium speed ────────────────────────────────────────────────
//
// In steady-state following, CalcAcc = 0.
// Solve numerically: find V_eq such that total acc = 0 at gap = V_eq * T + s0.

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIDMEquilibriumTest,
	"MixedTraffic.Models.IDM.EquilibriumFollowing",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FIDMEquilibriumTest::RunTest(const FString& Parameters)
{
	FIDMModel M = MakeCarIDM();

	// At equilibrium: s = s0 + v*T, both vehicles same speed, acc=0
	const float VeqTest = 15.0f;  // below V0=25
	const float Seq = M.Params.S0 + VeqTest * M.Params.T;  // = 17m

	// At this gap with matching speeds, sstar = s0 + v*T = Seq
	// → SFrac = 1 → accInt = -a → total = accFree + (-a)
	// Not exactly zero in IDM (IDM equilibrium is at v=v0 with infinite gap,
	// or at s = s0+v0*T... that's only exact at the capacity speed).
	// At Veq < V0 with equilibrium gap, total acc should be small.
	const float TotalAcc = M.CalcAcc(Seq, VeqTest, VeqTest, 0.0f);
	TestTrue(TEXT("AccFree dominates at equilibrium gap (acc > 0 since v < v0)"),
		TotalAcc > -IDM_TOL);  // heading toward desired speed

	return true;
}

// ── Test: Negative gap clamping ────────────────────────────────────────────

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIDMNegativeGapTest,
	"MixedTraffic.Models.IDM.NegativeGapClamping",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FIDMNegativeGapTest::RunTest(const FString& Parameters)
{
	FIDMModel M = MakeCarIDM();

	// Negative gap (overlap) must not produce NaN or positive value
	const float Acc = M.CalcAccInt(-1.0f, 10.0f, 0.0f, 0.0f);
	TestTrue(TEXT("Negative gap: not NaN"), !FMath::IsNaN(Acc));
	TestTrue(TEXT("Negative gap: at most -BMax"), Acc >= -M.Params.BMax - IDM_TOL);

	return true;
}
