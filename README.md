# VRChat wouldn't let me calibrate fullbody without two controllers, so i made a steamvr driver
This is a fork of finallyfunctional's [openvr-driver-example](https://github.com/finallyfunctional/openvr-driver-example)

If you want to use this for your right hand you'll have to recompile it yourself and change [#define rightHand false](https://github.com/DeltaNeverUsed/Fake-Index-Controller-Because-VRChat-wont-let-me-calibrate-fullbody-without-two-controllers/blob/59a9064e698f77175c83e846c99940ea4e5c20bc/OpenVrDriverExample/OpenVrDriverExample/src/ControllerDriver.cpp#L4) to true.
you'd probably also want to change the controls in the [RunFrame()](https://github.com/DeltaNeverUsed/Fake-Index-Controller-Because-VRChat-wont-let-me-calibrate-fullbody-without-two-controllers/blob/59a9064e698f77175c83e846c99940ea4e5c20bc/OpenVrDriverExample/OpenVrDriverExample/src/ControllerDriver.cpp#L290) function.

# Controls by default
![Controls image](/Images/controller.png)

# Installation
1. Download the [latest release](https://github.com/DeltaNeverUsed/Fake-Index-Controller-Because-VRChat-wont-let-me-calibrate-fullbody-without-two-controllers/releases/latest) from the releases tab
2. Extract it and plop that bad boy in SteamVR's driver folder, probably at ``C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers``
3. Make sure multiple drivers are enabled in ``steamvr.vrsettings`` also probably at ``C:\Program Files (x86)\Steam\config``
    1. Find ``"steamvr" : `` in there there should be a line saying ``"activateMultipleDrivers" : true,``
    2. If activateMultipleDrivers is false, change it to true.
    3. If it's not there, just add it.
4. Use something like [HidHide](https://github.com/ViGEm/HidHide) to hide your controller from VRChat.
5. You're done.

# Why is VRChat like this
IK 2.0 was supposed to support pretty much any configuration of trackers, for gods sake you can track just your left foot or elbow if you want, but you aren't allowed to track anything if you don't have two controllers. Why? this is such a stupid limitation that makes no sense.

The quality control of the index controllers and the headset are absolutely atrous,
and it kind of sucks not being able to use any of the expense vive pucks, tundra trackers, or whatever you have whenever the index decides to break randomly for no reason.

Dear Valve and VRChat, please fix.