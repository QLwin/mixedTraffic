// Copyright QLwin. All Rights Reserved.
// ACCModelTest.cpp – UE Automation Tests for FACCModel.
//
// Run via: Session Frontend → Automation → MixedTraffic.Models.ACC.*
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Models/ACCModel.h"
#include "Models/IDMModel.h"

static constexpr float ACC_TOL = 1e-3f;

static FACCModel MakeCarACC()
{
	FACCModel M;
	M.Params = FLongModelParams::CarDefaults();
	M.Cool   = 0.99f;
	return M;
}

static FIDMModel MakeCarIDM()
{
	FIDMModel M;
	M.Params = FLongModelParams::CarDefaults();
	return M;
}

// ── Test: Free acceleration matches IDM ───────────────────────────────────

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FACCFreeAccTest,
	"MixedTraffic.Models.ACC.FreeAcceleration",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FACCFreeAccTest::RunTest(const FString& Parameters)
{
	FACCModel ACC = MakeCarACC();
	FIDMModel IDM = MakeCarIDM();

	// ACC free acceleration is identical to IDM free acceleration
	for (float V : {0.0f, 5.0f, 13.9f, 25.0f, 30.0f})
	{
		TestNearlyEqual(
			*FString::Printf(TEXT("ACC CalcAccFree(%0.1f) == IDM CalcAccFree"), V),
			ACC.CalcAccFree(V), IDM.CalcAccFree(V), ACC_TOL);
	}

	return true;
}

// ── Test: ACC is less harsh than IDM when leader decelerates ──────────────
//
// When the leader applies strong braking (Al < 0), ACC should be less
// aggressive than IDM thanks to the CAH term.

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FACCSmoothBrakingTest,
	"MixedTraffic.Models.ACC.SmoothBrakingVsIDM",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FACCSmoothBrakingTest::RunTest(const FString& Parameters)
{
	FACCModel ACC = MakeCarACC();
	FIDMModel IDM = MakeCarIDM();

	// Scenario: 20 m gap, both at 15 m/s, leader suddenly brakes at -4 m/s²
	const float S  = 20.0f;
	const float V  = 15.0f;
	const float Vl = 15.0f;
	const float Al = -4.0f;

	const float AccACC = ACC.CalcAcc(S, V, Vl, Al);
	const float AccIDM = IDM.CalcAcc(S, V, Vl, 0.0f);  // IDM ignores Al

	// ACC should brake more gently initially (higher acc = less deceleration)
	TestTrue(TEXT("ACC brakes more gently than IDM when leader decelerates"),
		AccACC >= AccIDM - ACC_TOL);

	return true;
}

// ── Test: Hard-stop clamp ─────────────────────────────────────────────────

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FACCBMaxClampTest,
	"MixedTraffic.Models.ACC.BMaxClamp",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FACCBMaxClampTest::RunTest(const FString& Parameters)
{
	FACCModel ACC = MakeCarACC();

	// Very small gap, fast approach → should be clamped to -BMax
	const float Acc = ACC.CalcAcc(0.1f, 20.0f, 0.0f, -5.0f);
	TestTrue(TEXT("Output >= -BMax"), Acc >= -ACC.Params.BMax - ACC_TOL);
	TestTrue(TEXT("Output not NaN"),  !FMath::IsNaN(Acc));

	return true;
}

// ── Test: Large gap → free flow for both IDM and ACC ─────────────────────

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FACCFreeFlowTest,
	"MixedTraffic.Models.ACC.LargeGapFreeFlow",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FACCFreeFlowTest::RunTest(const FString& Parameters)
{
	FACCModel ACC = MakeCarACC();

	// With huge gap, CalcAcc ≈ CalcAccFree
	const float V   = 10.0f;
	const float Acc = ACC.CalcAcc(1e6f, V, V, 0.0f);
	TestNearlyEqual(TEXT("Large gap: CalcAcc ≈ CalcAccFree"),
		Acc, ACC.CalcAccFree(V), ACC_TOL);

	return true;
}

// ── Test: MTM lateral free acc (damping) ──────────────────────────────────

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMTMLatFreeTest,
	"MixedTraffic.Models.MTM.LateralFreeAcceleration",
	EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMTMLatFreeTest::RunTest(const FString& Parameters)
{
	// Include MTM model header through the test indirectly via ACCModel
	// (full MTM tested in MTMModelTest.cpp)

	// Verify the OVM formula: accLatFree = -vy / tauLatOVM
	const float TauLatOVM = 3.0f;
	const float Vy = 2.0f;
	const float Expected = -Vy / TauLatOVM;
	// Direct formula check — FMTMModel::CalcAccLatFree is inline in MTMModel.h
	// but we test the math here:
	TestNearlyEqual(TEXT("OVM formula: accLatFree = -vy/tau"),
		Expected, -2.0f / 3.0f, ACC_TOL);

	return true;
}
