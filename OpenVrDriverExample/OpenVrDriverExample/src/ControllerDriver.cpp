#include <ControllerDriver.h>
#include <Xinput.h>

#define rightHand false

EVRInitError ControllerDriver::Activate(uint32_t unObjectId)
{
	driverId = unObjectId; //unique ID for your driver

	PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(driverId); //this gets a container object where you store all the information about your driver
	VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json"); //tell OpenVR where to get your driver's Input Profile

	VRProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "lighthouse");
	VRProperties()->SetStringProperty(props, vr::Prop_ResourceRoot_String, "htc");
	VRProperties()->SetUint64Property(props, Prop_CurrentUniverseId_Uint64, 31);
	VRProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "DeltaNeverUsed");
	VRProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "100% real index controller");

	VRProperties()->SetStringProperty(props, vr::Prop_HardwareRevision_String, "14");
	VRProperties()->SetUint64Property(props, vr::Prop_HardwareRevision_Uint64, 14U);

	VRProperties()->SetFloatProperty(props, vr::Prop_DeviceBatteryPercentage_Float, 1.f);
	VRProperties()->SetBoolProperty(props, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);

	VRProperties()->SetInt32Property(props, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
	VRProperties()->SetBoolProperty(props, vr::Prop_Identifiable_Bool, true);

	VRProperties()->SetInt32Property(
		props, vr::Prop_ControllerRoleHint_Int32, rightHand ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);

	VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, 2147483647);
	VRProperties()->SetStringProperty(props, vr::Prop_SerialNumber_String, "LHR-E217CD69");
	VRProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "knuckles");

	VRProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, rightHand ? "{indexcontroller}valve_controller_knu_1_0_right" : "{indexcontroller}valve_controller_knu_1_0_left");

	VRDriverInput()->CreateScalarComponent(props, "/input/thumbstick/y", &joystickYHandle, EVRScalarType::VRScalarType_Absolute,
		EVRScalarUnits::VRScalarUnits_NormalizedTwoSided); //sets up handler you'll use to send joystick commands to OpenVR with, in the Y direction (forward/backward)
	VRDriverInput()->CreateScalarComponent(props, "/input/thumbstick/x", &joystickXHandle, EVRScalarType::VRScalarType_Absolute,
		EVRScalarUnits::VRScalarUnits_NormalizedTwoSided); //Why VRScalarType_Absolute? Take a look at the comments on EVRScalarType.

	VRDriverInput()->CreateBooleanComponent(props, "/input/a/click", &AHandle);
	VRDriverInput()->CreateBooleanComponent(props, "/input/b/click", &BHandle);

	VRDriverInput()->CreateScalarComponent(props, "/input/trigger/value", &TriggerPullHandle, EVRScalarType::VRScalarType_Absolute,
		EVRScalarUnits::VRScalarUnits_NormalizedOneSided); //Why VRScalarType_Absolute? Take a look at the comments on EVRScalarType.
	VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &TriggerHandle);


	
	//The following properites are ones I tried out because I saw them in other samples, but I found they were not needed to get the sample working.
	//There are many samples, take a look at the openvr_header.h file. You can try them out.

	//VRProperties()->SetUint64Property(props, Prop_CurrentUniverseId_Uint64, 2);
	//VRProperties()->SetBoolProperty(props, Prop_HasControllerComponent_Bool, true);
	//VRProperties()->SetBoolProperty(props, Prop_NeverTracked_Bool, true);
	//VRProperties()->SetInt32Property(props, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
	//VRProperties()->SetInt32Property(props, Prop_Axis2Type_Int32, k_eControllerAxis_Joystick);
	//VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "example_controler_serial");
	//VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "vr_controller_vive_1_5");
	//uint64_t availableButtons = ButtonMaskFromId(k_EButton_SteamVR_Touchpad) |
	//	ButtonMaskFromId(k_EButton_IndexController_JoyStick);
	//VRProperties()->SetUint64Property(props, Prop_SupportedButtons_Uint64, availableButtons);

	return VRInitError_None;
}

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
	vr::HmdVector3_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}

vr::HmdQuaternion_t ovrQuatfmul(vr::HmdQuaternion_t q1, vr::HmdQuaternion_t q2) {
	vr::HmdQuaternion_t result = { 0 };
	result.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	result.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	result.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	result.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	return result;
}
vr::HmdVector3_t rotateVector(const vr::HmdVector3_t _V, vr::HmdQuaternion_t q) {
	vr::HmdVector3_t vec;   // any constructor will do
	float r, i, j, k;
	r = q.w;
	i = q.x;
	j = q.y;
	k = q.z;
	vec.v[0] = 2 * (r * _V.v[2] * j + i * _V.v[2] * k - r * _V.v[1] * k + i * _V.v[1] * j) + _V.v[0] * (r * r + i * i - j * j - k * k);
	vec.v[1] = 2 * (r * _V.v[0] * k + i * _V.v[0] * j - r * _V.v[2] * i + j * _V.v[2] * k) + _V.v[1] * (r * r - i * i + j * j - k * k);
	vec.v[2] = 2 * (r * _V.v[1] * i - r * _V.v[0] * j + i * _V.v[0] * k + j * _V.v[1] * k) + _V.v[2] * (r * r - i * i - j * j + k * k);
	return vec;
}
#define M_PI           3.14159265358979323846
vr::HmdVector3_t QuaternionToEuler(vr::HmdQuaternion_t q)
{
	vr::HmdVector3_t euler;

	// if the input quaternion is normalized, this is exactly one. Otherwise, this acts as a correction factor for the quaternion's not-normalizedness
	float unit = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);

	// this will have a magnitude of 0.5 or greater if and only if this is a singularity case
	float test = q.x * q.w - q.y * q.z;

	if (test > 0.4995f * unit) // singularity at north pole
	{
		euler.v[0] = M_PI / 2;
		euler.v[1] = 2. * atan2(q.y, q.x);
		euler.v[2] = 0;
	}
	else if (test < -0.4995f * unit) // singularity at south pole
	{
		euler.v[0] = -M_PI / 2;
		euler.v[1] = -2. * atan2(q.y, q.x);
		euler.v[2] = 0;
	}
	else // no singularity - this is the majority of cases
	{
		euler.v[0] = asin(2. * (q.w * q.x - q.y * q.z));
		euler.v[1] = atan2(2. * q.w * q.y + 2. * q.z * q.x, 1. - 2. * (q.x * q.x + q.y * q.y)); // I don't even fucking know, man. Fuck you quaternions.
		euler.v[2] = atan2(2. * q.w * q.z + 2. * q.x * q.y, 1. - 2. * (q.z * q.z + q.x * q.x));
	}

	return euler;
}

vr::HmdQuaternion_t EulerToQuaternion(vr::HmdVector3_t euler)
{
	float xOver2 = euler.v[0] * 0.5f;
	float yOver2 = euler.v[1] * 0.5f;
	float zOver2 = euler.v[2] * 0.5f;

	float sinXOver2 = sin(xOver2);
	float cosXOver2 = cos(xOver2);
	float sinYOver2 = sin(yOver2);
	float cosYOver2 = cos(yOver2);
	float sinZOver2 = sin(zOver2);
	float cosZOver2 = cos(zOver2);

	vr::HmdQuaternion_t result;
	result.x = cosYOver2 * sinXOver2 * cosZOver2 + sinYOver2 * cosXOver2 * sinZOver2;
	result.y = sinYOver2 * cosXOver2 * cosZOver2 - cosYOver2 * sinXOver2 * sinZOver2;
	result.z = cosYOver2 * cosXOver2 * sinZOver2 - sinYOver2 * sinXOver2 * cosZOver2;
	result.w = cosYOver2 * cosXOver2 * cosZOver2 + sinYOver2 * sinXOver2 * sinZOver2;

	return result;
}

float dot_product(vr::HmdVector3_t vector_a, vr::HmdVector3_t vector_b) {
	float product = 0;
	product += vector_a.v[0] * vector_b.v[0];
	product += vector_a.v[1] * vector_b.v[1];
	product += vector_a.v[2] * vector_b.v[2];
	return product;
}

DriverPose_t last_pose;
bool locked;

DriverPose_t ControllerDriver::GetPose()
{

	TrackedDevicePose_t hmd_pose[10];
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmd_pose, 10);


	int controllerInt = -1;

	for (int i = 0; i < 10; i++)
	{

		auto props = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);

		std::string renderModel = VRProperties()->GetStringProperty(props, Prop_RenderModelName_String);

		if (renderModel.find(!rightHand ? "valve_controller_knu_1_0_right" : "valve_controller_knu_1_0_left") != std::string::npos) {
			controllerInt = i;
			break;
		}

		//VRDriverLog()->Log("Device %d = %s %s\n", i, modelNumber.c_str(), renderModel.c_str());
	}
	//hmd_pose[controllerInt].mDeviceToAbsoluteTracking

	//VRDriverLog()->Log(std::to_string((float)hmd_pose[controllerInt].mDeviceToAbsoluteTracking.m[0][0]).c_str());

	DriverPose_t pose = { 0 }; //This example doesn't use Pose, so this method is just returning a default Pose.
	pose.poseIsValid = !locked;
	pose.result = locked ? TrackingResult_Running_OutOfRange : TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	if (controllerInt != -1) {

		auto hmdrot = GetRotation(hmd_pose[0].mDeviceToAbsoluteTracking);
		auto controllerrot = GetRotation(hmd_pose[controllerInt].mDeviceToAbsoluteTracking);

		auto hmdrotEu = QuaternionToEuler(hmdrot);
		hmdrotEu.v[0] = 0;
		hmdrotEu.v[2] = 0;
		auto hmdrotY = EulerToQuaternion(hmdrotEu);
		vr::HmdQuaternion_t hmdrotYInverted;
		hmdrotYInverted.w = hmdrotY.w;
		hmdrotYInverted.x = -hmdrotY.x;
		hmdrotYInverted.y = -hmdrotY.y;
		hmdrotYInverted.z = -hmdrotY.z;

		HmdVector3_t tempPos;
		tempPos.v[0] = hmd_pose[controllerInt].mDeviceToAbsoluteTracking.m[0][3] - hmd_pose[0].mDeviceToAbsoluteTracking.m[0][3];
		tempPos.v[1] = hmd_pose[controllerInt].mDeviceToAbsoluteTracking.m[1][3] - hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3];
		tempPos.v[2] = hmd_pose[controllerInt].mDeviceToAbsoluteTracking.m[2][3] - hmd_pose[0].mDeviceToAbsoluteTracking.m[2][3];

		vr::HmdQuaternion_t temprot;
		temprot.w = 0.707;
		temprot.x = 0.0;
		temprot.y = 0.707;
		temprot.z = 0.0;

		auto worldTolocalRot = ovrQuatfmul(temprot, hmdrotYInverted);
		auto localToworldRot = ovrQuatfmul(controllerrot, hmdrotY);

		auto tempPosRotThing = rotateVector(tempPos, hmdrotY);

		HmdVector3_t tempoffset;
		tempoffset.v[0] = 1;
		tempoffset.v[1] = 0;
		tempoffset.v[2] = 0;
		tempoffset = rotateVector(tempoffset, hmdrotY);

		tempoffset.v[0] = -dot_product(tempoffset, tempPos)*2;
		tempoffset.v[1] = 0;
		tempoffset.v[2] = 0;
		tempoffset = rotateVector(tempoffset, hmdrotY);


		//HmdVector3_t tempOffset;
		//tempOffset.x = 0.5;

		tempPos.v[0] += tempoffset.v[0];
		tempPos.v[1] += tempoffset.v[1];
		tempPos.v[2] += tempoffset.v[2];

		pose.vecPosition[0] = tempPos.v[0] + hmd_pose[0].mDeviceToAbsoluteTracking.m[0][3];
		pose.vecPosition[1] = tempPos.v[1] + hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3];
		pose.vecPosition[2] = tempPos.v[2] + hmd_pose[0].mDeviceToAbsoluteTracking.m[2][3];

		pose.qRotation = controllerrot;
	}
	
	HmdQuaternion_t quat;
	quat.w = 1;
	quat.x = 0;
	quat.y = 0;
	quat.z = 0;

	pose.qWorldFromDriverRotation = quat;
	pose.qDriverFromHeadRotation = quat;

	last_pose = pose;

	return pose;
}

bool just_locked;

#pragma comment(lib,"XInput.lib")

#pragma comment(lib,"Xinput9_1_0.lib")

void ControllerDriver::RunFrame()
{
	XINPUT_STATE state;
	XInputGetState(0, &state);

	if (state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) {
		if (!just_locked)
			locked = !locked;
		just_locked = true;
	}
	else {
		just_locked = false;
	}

	

	VRDriverInput()->UpdateBooleanComponent(AHandle, state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN, 0);
	VRDriverInput()->UpdateBooleanComponent(BHandle, state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP, 0);

	VRDriverInput()->UpdateBooleanComponent(TriggerHandle, state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT, 0);
	VRDriverInput()->UpdateScalarComponent(TriggerPullHandle, (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) ? 1. : 0., 0);

	float ythumb = (float)state.Gamepad.sThumbLY / 32767.;
	float xthumb = (float)state.Gamepad.sThumbLX / 32767.;

	ythumb = (ythumb > 0.25 || ythumb < -0.25) ? ythumb : 0;
	xthumb = (xthumb > 0.25 || xthumb < -0.25) ? xthumb : 0;

	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(driverId, GetPose(), sizeof(vr::DriverPose_t));
	//Since we used VRScalarUnits_NormalizedTwoSided as the unit, the range is -1 to 1.
	VRDriverInput()->UpdateScalarComponent(joystickYHandle, ythumb, 0); //move forward
	//VRDriverInput()->UpdateScalarComponent(trackpadYHandle, 0.95f, 0); //move foward
	VRDriverInput()->UpdateScalarComponent(joystickXHandle, xthumb, 0); //change the value to move sideways
	//VRDriverInput()->UpdateScalarComponent(trackpadXHandle, 0.0f, 0); //change the value to move sideways
}

void ControllerDriver::Deactivate()
{
	driverId = k_unTrackedDeviceIndexInvalid;
}

void* ControllerDriver::GetComponent(const char* pchComponentNameAndVersion)
{
	//I found that if this method just returns null always, it works fine. But I'm leaving the if statement in since it doesn't hurt.
	//Check out the IVRDriverInput_Version declaration in openvr_driver.h. You can search that file for other _Version declarations 
	//to see other components that are available. You could also put a log in this class and output the value passed into this 
	//method to see what OpenVR is looking for.
	if (strcmp(IVRDriverInput_Version, pchComponentNameAndVersion) == 0)
	{
		return this;
	}
	return NULL;
}

void ControllerDriver::EnterStandby() {}

void ControllerDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) 
{
	if (unResponseBufferSize >= 1)
	{
		pchResponseBuffer[0] = 0;
	}
}