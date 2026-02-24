#include <windows.h>
#include <format>
#include <cmath>
#include <toml++/toml.hpp>

#include "nya_commonhooklib.h"
#include "nfsuc.h"

#include "include/chloemenulib.h"

void WriteLog(const std::string& str) {
	static auto file = std::ofstream("NFSUCChassisSimple_gcp.log");

	file << str;
	file << "\n";
	file.flush();
}

void ValueEditorMenu(float& value) {
	ChloeMenuLib::BeginMenu();

	static char inputString[1024] = {};
	ChloeMenuLib::AddTextInputToString(inputString, 1024, true);
	ChloeMenuLib::SetEnterHint("Apply");

	if (DrawMenuOption(inputString + (std::string)"...", "", false, false) && inputString[0]) {
		value = std::stof(inputString);
		memset(inputString,0,sizeof(inputString));
		ChloeMenuLib::BackOut();
	}

	ChloeMenuLib::EndMenu();
}

void QuickValueEditor(const char* name, float& value) {
	if (DrawMenuOption(std::format("{} - {}", name, value))) { ValueEditorMenu(value); }
}

struct tCarSteeringSetup {
	std::string mdl;
	float fSteerAngle[8] = {0, 50, 100, 150, 200, 250, 300, 350};
	float fSpeedCurve[8] = {1, 0.6, 0.3, 0.2, 0.15, 0.1, 0.08, 0.08};
	float fKeyboardSteerSpeed = 1.0;
	float fCounterSteerBonus = 4.0;
};
std::vector<tCarSteeringSetup> aCarSteerings;
bool bOverrideCarSteering = false;
tCarSteeringSetup gDefaultCarSteering;

tCarSteeringSetup* GetCarSteeringData(const char* mdl) {
	if (bOverrideCarSteering) return &gDefaultCarSteering;

	for (auto& car : aCarSteerings) {
		if (car.mdl != mdl) continue;
		return &car;
	}

	auto setup = gDefaultCarSteering;
	setup.mdl = mdl;
	aCarSteerings.push_back(setup);
	WriteLog(std::format("Failed to find steering curves for {}, using defaults", mdl));
	return GetCarSteeringData(mdl);
}

float curve = 0;
float GetCarSteeringMult(Attrib::Gen::car_tuning::_LayoutStruct* data, IVehicle* veh) {
	if (veh->mCOMObject->Find<ITransmission>()->IsReversing()) return 1.0;

	auto spd = veh->GetSpeed() * 3.6;
	if (spd <= 0.0) return 1.0;

	auto newData = GetCarSteeringData(veh->GetVehicleName());

	curve = newData->fSteerAngle[0];
	for (int i = 0; i < 8; i++) {
		if (spd < newData->fSpeedCurve[i]) break;

		if (i >= 7) return curve = newData->fSteerAngle[i];

		float delta = spd;
		delta -= newData->fSpeedCurve[i];
		delta /= (newData->fSpeedCurve[i+1] - newData->fSpeedCurve[i]);
		curve = std::lerp(newData->fSteerAngle[i], newData->fSteerAngle[i+1], delta);
	}
	return curve;
}

float fSmoothedSteerState = 0;
void IncreaseTowards(float& value, float target, float speed) {
	if (value < target) {
		value += speed;
		if (value > target) value = target;
		return;
	}
	if (value > target) {
		value -= speed;
		if (value < target) value = target;
		return;
	}
}

double easeOutQuart(double t) {
	t = (--t) * t;
	return 1 - t * t;
}

void __thiscall SimpleGetStateHooked(ChassisSimple* pThis, float dT, Chassis::State *state) {
	ChassisSimple::ComputeState(pThis, dT, state);
	state->nos_boost = 1.0; // remove nos grip bonus

	if (pThis->mVehicle->GetDriverClass() != DRIVER_HUMAN) return;
	state->over_steer_input = 0.0;
	state->ebrake_input = 0.0; // todo hack

	auto newData = GetCarSteeringData(pThis->mVehicle->GetVehicleName());
	float smoothSteerSpeed = newData->fKeyboardSteerSpeed * dT;

	if (fSmoothedSteerState > 0 && state->steer_input < fSmoothedSteerState) smoothSteerSpeed *= newData->fCounterSteerBonus;
	if (fSmoothedSteerState < 0 && state->steer_input > fSmoothedSteerState) smoothSteerSpeed *= newData->fCounterSteerBonus;
	//if (fSmoothedSteerState > 0.01 && state->steer_input < -0.01) smoothSteerSpeed *= 2;
	//if (fSmoothedSteerState < -0.01 && state->steer_input > 0.01) smoothSteerSpeed *= 2;

	IncreaseTowards(fSmoothedSteerState, state->steer_input, smoothSteerSpeed);
	float easedSteer = easeOutQuart(std::abs(fSmoothedSteerState));
	if (fSmoothedSteerState < 0) easedSteer *= -1;
	state->steer_input = easedSteer * GetCarSteeringMult(pThis->mVehicleInfo.GetLayout(), pThis->mVehicle);
}

void LoadSteeringCurves() {
	aCarSteerings.clear();

	if (std::filesystem::exists("SteeringCurves")) {
		for (const auto& entry : std::filesystem::directory_iterator("SteeringCurves")) {
			if (entry.is_directory()) continue;

			auto path = entry.path();
			if (path.extension() != ".toml") continue;

			auto config = toml::parse_file(path.string());

			auto carName = path.filename().string();
			for (int i = 0; i < 5; i++) { carName.pop_back(); }

			tCarSteeringSetup tmp;
			auto setup = &tmp;
			if (carName == "default") {
				setup = &gDefaultCarSteering;
			}

			setup->mdl = carName;
			for (int i = 0; i < 8; i++) {
				setup->fSteerAngle[i] = config["steer_angle"][i].value_or(setup->fSteerAngle[i]);
				setup->fSpeedCurve[i] = config["speed_curve"][i].value_or(setup->fSpeedCurve[i]);
			}
			setup->fKeyboardSteerSpeed = config["steering_speed"].value_or(setup->fKeyboardSteerSpeed);
			setup->fCounterSteerBonus = config["countersteering_speed"].value_or(setup->fCounterSteerBonus);
			if (carName != "default") {
				aCarSteerings.push_back(*setup);
			}

			WriteLog(std::format("Registered steering curves for {} ({})", carName, path.string()));
		}
	}
}

void DebugMenu() {
	ChloeMenuLib::BeginMenu();

	if (DrawMenuOption(std::format("Use Overrides - {}", bOverrideCarSteering))) {
		bOverrideCarSteering = !bOverrideCarSteering;
	}

	for (int i = 0; i < 8; i++) {
		QuickValueEditor(std::format("fSteer[{}]", i).c_str(), gDefaultCarSteering.fSteerAngle[i]);
		QuickValueEditor(std::format("fSpeed[{}]", i).c_str(), gDefaultCarSteering.fSpeedCurve[i]);
	}
	QuickValueEditor("fKeyboardSteerSpeed", gDefaultCarSteering.fKeyboardSteerSpeed);
	QuickValueEditor("fCounterSteerBonus", gDefaultCarSteering.fCounterSteerBonus);

	if (DrawMenuOption("Reload Steering Curves")) {
		LoadSteeringCurves();
	}

	DrawMenuOption(std::format("steer input {:.2f}", fSmoothedSteerState), "", false);
	DrawMenuOption(std::format("steer cap {:.2f}", curve), "", false);

	ChloeMenuLib::EndMenu();
}

BOOL WINAPI DllMain(HINSTANCE, DWORD fdwReason, LPVOID) {
	switch( fdwReason ) {
		case DLL_PROCESS_ATTACH: {
			if (NyaHookLib::GetEntryPoint() != 0x4AEC55) {
				MessageBoxA(nullptr, "Unsupported game version! Make sure you're using v1.0.0.1 (.exe size of 10584064 or 10589456 bytes)", "nya?!~", MB_ICONERROR);
				return TRUE;
			}

			LoadSteeringCurves();

			ChloeMenuLib::RegisterMenu("Debug Menu", &DebugMenu);

			NyaHookLib::PatchRelative(NyaHookLib::CALL, 0x72B17F, &SimpleGetStateHooked);
			NyaHookLib::PatchRelative(NyaHookLib::JMP, 0x72B65D, 0x72B827); // disable "sleep" that isn't actual sleep
			NyaHookLib::Patch<uint8_t>(0x7136DF, 0xEB); // always allow tire slip
			NyaHookLib::PatchRelative(NyaHookLib::JMP, 0x6FB3CD, 0x6FB6DC); // disable jump stabilizer

			// test loop - cap driver skills at 25%
			/*NyaHooks::WorldServiceHook::Init();
			NyaHooks::WorldServiceHook::aPreFunctions.push_back([]() {
				for (auto& skill : GMW2Game::mObj->mRewardModifiers) {
					if (skill > 0.25) {
						skill = 0.25;
					}
				}
			});*/

			NyaHooks::LateInitHook::Init();
			NyaHooks::LateInitHook::aFunctions.push_back([](){ *(uintptr_t*)0xDEA82C = 0x73F8F0; }); // use chassissimple

			WriteLog("Mod initialized");
		} break;
		default:
			break;
	}
	return TRUE;
}