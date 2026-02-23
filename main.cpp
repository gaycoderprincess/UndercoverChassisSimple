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
	float fMinSteer = 0.05;
	float fMaxSteer = 0.3;
	float fMinSpeed = 0;
	float fMaxSpeed = 300;
	float fKeyboardSteerSpeed = 0.25;
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

	spd -= newData->fMinSpeed;
	if (spd < 0) spd = 0;
	spd /= (newData->fMaxSpeed - newData->fMinSpeed);

	auto f = std::lerp(newData->fMinSteer, newData->fMaxSteer, 1.0 - spd);
	if (f < newData->fMinSteer) return curve = newData->fMinSteer;
	if (f > newData->fMaxSteer) return curve = newData->fMaxSteer;
	return curve = f;

	//return curve = Curve::GetValue(&data->STEERING_RANGE, spd);
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

void __thiscall SimpleSteeringHooked(ChassisSimple* pThis, Chassis::State *state, UMath::Vector3 *right, UMath::Vector3 *left) {
	if (pThis->mVehicle->GetDriverClass() != DRIVER_HUMAN) return ChassisSimple::DoSteering(pThis, state, right, left);

	static auto lastTime = Sim::GetTime();
	auto delta = (double)(Sim::GetTime() - lastTime) / 1000.0;
	if (delta < 0.0) delta = 0.0;
	if (delta > 1.0 / 15.0) delta = 1.0 / 15.0;
	lastTime = Sim::GetTime();

	auto newState = *state;
	newState.steer_input *= GetCarSteeringMult(pThis->mVehicleInfo.GetLayout(), pThis->mVehicle);
	IncreaseTowards(fSmoothedSteerState, newState.steer_input, GetCarSteeringData(pThis->mVehicle->GetVehicleName())->fKeyboardSteerSpeed * delta);
	newState.steer_input = fSmoothedSteerState;
	ChassisSimple::DoSteering(pThis, &newState, right, left);
}

void __thiscall SimpleDriveForceHooked(ChassisSimple* pThis, Chassis::State *state) {
	auto newState = *state;
	newState.nos_boost = 1.0;
	ChassisSimple::DoDriveForces(pThis, &newState);
}

void __thiscall SimpleGetStateHooked(ChassisSimple* pThis, float dT, Chassis::State *state) {
	ChassisSimple::ComputeState(pThis, dT, state);
	state->nos_boost = 1.0; // remove nos grip bonus

	if (pThis->mVehicle->GetDriverClass() != DRIVER_HUMAN) return;
	state->over_steer_input = 0.0;

	static auto lastTime = Sim::GetTime();
	auto delta = (double)(Sim::GetTime() - lastTime) / 1000.0;
	if (delta < 0.0) delta = 0.0;
	if (delta > 1.0 / 15.0) delta = 1.0 / 15.0;
	lastTime = Sim::GetTime();

	auto newState = *state;
	newState.steer_input *= GetCarSteeringMult(pThis->mVehicleInfo.GetLayout(), pThis->mVehicle);
	IncreaseTowards(fSmoothedSteerState, newState.steer_input, GetCarSteeringData(pThis->mVehicle->GetVehicleName())->fKeyboardSteerSpeed * delta);
	newState.steer_input = fSmoothedSteerState;
}

void DebugMenu() {
	ChloeMenuLib::BeginMenu();

	if (DrawMenuOption(std::format("Use Overrides - {}", bOverrideCarSteering))) {
		bOverrideCarSteering = !bOverrideCarSteering;
	}
	QuickValueEditor("fMinSteer", gDefaultCarSteering.fMinSteer);
	QuickValueEditor("fMaxSteer", gDefaultCarSteering.fMaxSteer);
	QuickValueEditor("fMinSpeed", gDefaultCarSteering.fMinSpeed);
	QuickValueEditor("fMaxSpeed", gDefaultCarSteering.fMaxSpeed);
	QuickValueEditor("fKeyboardSteerSpeed", gDefaultCarSteering.fKeyboardSteerSpeed);

	DrawMenuOption("Debug State", "", false);
	DrawMenuOption(std::format("steer input {:.2f}", fSmoothedSteerState), "", false);
	DrawMenuOption(std::format("steer output {:.2f}", curve), "", false);

	ChloeMenuLib::EndMenu();
}

BOOL WINAPI DllMain(HINSTANCE, DWORD fdwReason, LPVOID) {
	switch( fdwReason ) {
		case DLL_PROCESS_ATTACH: {
			if (NyaHookLib::GetEntryPoint() != 0x4AEC55) {
				MessageBoxA(nullptr, "Unsupported game version! Make sure you're using v1.0.0.1 (.exe size of 10584064 or 10589456 bytes)", "nya?!~", MB_ICONERROR);
				return TRUE;
			}

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
					setup->fMinSteer = config["min_steer_angle"].value_or(setup->fMinSteer);
					setup->fMaxSteer = config["max_steer_angle"].value_or(setup->fMaxSteer);
					setup->fMinSpeed = config["min_steer_angle_speed"].value_or(setup->fMinSpeed);
					setup->fMaxSpeed = config["max_steer_angle_speed"].value_or(setup->fMaxSpeed);
					setup->fKeyboardSteerSpeed = config["steering_speed"].value_or(setup->fKeyboardSteerSpeed);
					if (carName != "default") {
						aCarSteerings.push_back(*setup);
					}

					WriteLog(std::format("Registered steering curves for {} ({})", carName, path.string()));
				}
			}

			ChloeMenuLib::RegisterMenu("Debug Menu", &DebugMenu);

			NyaHookLib::PatchRelative(NyaHookLib::CALL, 0x713979, &SimpleSteeringHooked);
			NyaHookLib::PatchRelative(NyaHookLib::CALL, 0x72B594, &SimpleDriveForceHooked);
			//NyaHookLib::PatchRelative(NyaHookLib::CALL, 0x72B17F, &SimpleGetStateHooked);
			NyaHookLib::PatchRelative(NyaHookLib::JMP, 0x72B65D, 0x72B827); // disable "sleep" that isn't actual sleep
			NyaHookLib::Patch<uint8_t>(0x7136DF, 0xEB); // always allow tire slip

			NyaHooks::WorldServiceHook::Init();
			NyaHooks::WorldServiceHook::aPreFunctions.push_back([]() {
				for (auto& skill : GMW2Game::mObj->mRewardModifiers) {
					if (skill > 0.25) {
						skill = 0.25;
					}
				}
			});
			NyaHooks::LateInitHook::Init();
			NyaHooks::LateInitHook::aFunctions.push_back([](){ *(uintptr_t*)0xDEA82C = 0x73F8F0; }); // use chassissimple

			WriteLog("Mod initialized");
		} break;
		default:
			break;
	}
	return TRUE;
}