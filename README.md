# BotW-BetterVR
A project aimed at providing a PC VR mode for Breath of the Wild using the Wii U emulator called Cemu. To provide the better experience it tries to run the game at high framerates and resolutions.

### Requirements
#### Supported VR headsets:
The app currently utilizes OpenXR, which is supported on all the major headsets (Valve Index, HTC Vive, Oculus Rift, Windows Mixed Reality etc). You can also use Trinus VR to use this with your phone's VR accessory.  
For Oculus headsets you have to switch to the Public Test Channel for OpenXR support.
For Trinus VR you'll have to enable OpenXR in the Trinus VR settings menu.
#### Other Requirements:
* A pretty decent PC with a good CPU (a recent Intel i5 or Ryzen 5 are recommended at least)!
* A copy of BotW for the Wii U
* A properly set up [Cemu](http://cemu.info/) emulator that's already able to run at 60FPS or higher. See [this guide](https://cemu.cfw.guide/) for more info.

### Mod Installation
// todo: Fully finish these instructions

1. Download the latest release of the mod from the [Releases](https://github.com/Crementif/BotW-BetterVR/releases) page.
2. Extract the downloaded `.zip` file where your `Cemu.exe` is stored.
3. Double click on `BetterVR-Launcher.bat` to start Cemu.
4. Go to `Options`-> `Graphic packs`-> `The Legend of Zelda: Breath of the Wild` and check the graphic pack named `BetterVR`.
5. Start the game like you normally would.
6. Go to `Options`->`Fullscreen` to make the game fullscreen or alternatively press `Alt+Enter`.
7. Put the VR Headset on your head and enjoy.

Each time you want to play the game in VR from now on you can just use the `BetterVR-launcher.bat` file to start it in VR mode again.
You can leave the graphic pack enabled while playing in non-VR without any issue.

### Building
1. Make sure that you've got [vcpkg](https://github.com/microsoft/vcpkg) installed.
2. Install the required dependencies using `vcpkg install openxr-loader:x64-windows eigen3:x64-windows glm:x64-windows`.
3. Open the project using Visual Studio and build it.

The OpenXR application has three different modes that you can change in the `BotWHook.cpp` file:
- `GFX_PACK_PASSTHROUGH` is only useful for releasing. It moves the camera computations from the app to the compiled code in the graphic pack itself to reduce latency code.
- `APP_CALC_CONVERT` is used as an intermediary form of the library-dependent code and is able to be compiled into a graphic pack patch that works with the `GFX_PACK_PASSTHROUGH` code.
- `APP_CALC_LIBRARY` will be the most useful mode for most people. It allows you to interact with the camera using the eigen and glm libraries so that it's easy to develop and experiment with.

### Licenses
This project is licensed under the MIT license.

Licenses from parts of the code are:
- `/OpenXR/` directory contains an application that is based on MIT licensed code from Microsoft's [OpenXR MixedReality repo](https://github.com/microsoft/OpenXR-MixedReality).