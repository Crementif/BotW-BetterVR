# BotW BetterVR

A project aimed at providing a PC VR mode for Breath of the Wild using the Wii U emulator called Cemu. To provide the
better experience it tries to run the game at high framerates and resolutions.

### Requirements

#### Supported VR headsets:

The app currently utilizes OpenXR, which is supported on all the major headsets (Valve Index, HTC Vive, Oculus Rift, Meta Quest,
Windows Mixed Reality etc.). You can also use apps like Trinus VR to use this with your phone's VR accessory.

#### Other Requirements:

* A pretty decent PC with a good CPU (a recent Intel i5 or Ryzen 5 are recommended at least)!

* A copy of BotW for the Wii U

* A properly set up [Cemu](http://cemu.info/) emulator that's already able to run at 60FPS or higher.
  See [this guide](https://cemu.cfw.guide/) for more info.  

* A recent Cemu version (at least Cemu 2.3 or newer, though only the latest version is tested to work).


### Mod Installation


0. The mod **will** crash if you don't have the latest [Microsoft Visual C++ Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe) installed!
   You might have to click on Repair if you already have an older version installed.

1. Download the latest release of the mod from the [Releases](https://github.com/Crementif/BotW-BetterVR/releases) page.

2. Extract the downloaded `.zip` file where your `Cemu.exe` is stored.
   There should now be a .dll, .json and .bat file in the same folder as your `Cemu.exe`.
   Make sure you start Cemu.exe once so that it can create the folder structure, if you've never done that before.

3. The `BreathOfTheWild_BetterVR` folder has to be put in either inside the `graphicPacks` folder next to Cemu.exe if there **IS** an existing `settings.xml` file (portable mode) or the `%appdata%\Cemu\graphicPacks` folder if there's **NO** `settings.xml` next to the Cemu.exe (non-portable mode) or 

4. Double-click on `Launch_BetterVR.bat` to start Cemu.

5. Go to `Options`-> `Graphic packs`-> `The Legend of Zelda: Breath of the Wild` and enable the graphic pack named `BetterVR`.
   This is where you can change any VR settings like the first/third-person mode etc.
   You'll also want to enable FPS++, or else the game will crash!

6. (Recommended) For an enjoyable experience you should change some other graphic packs in this same window too:
   - `Graphics` graphic pack: Use any (non-ultrawide!) resolution of 2160p (4k) or higher for clarity. Also change antialiasing to Nvidia FXAA.
   - `FPS++` graphic pack: Change the FPS limit to at least 120FPS or higher.
   - `Enhancements`: graphic pack: Change anisotropic filtering to 16x and use your preferred preset.

7. Close the settings and start the game like normal from Cemu's game list. You can now put on your VR headset and if installed correctly it should now work!

Each time you want to play the game in VR from now on you can just use the `Launch_BetterVR.bat` file to start it in VR mode again.

You'll need to disable the `BetterVR` graphic pack if you want to change to non-VR and not use the `Launch_BetterVR.bat` file.


### Build Instructions

1. Install the latest Vulkan SDK from https://vulkan.lunarg.com/sdk/home#windows and make sure that VULKAN_SDK was added
   to your environment variables.

2. Install [vcpkg](https://github.com/microsoft/vcpkg) and use the following command to install the required dependencies:
   `vcpkg install openxr-loader:x64-windows-static-md glm:x64-windows-static-md vulkan-headers:x64-windows-static-md imgui:x64-windows-static-md`

3. [Optional] Download and extract a new Cemu installation to the Cemu folder that's included.
   This step is technically not required, but it's the default install location and makes debugging much easier.

4. Use Clion or Visual Studio to open the CMake project. Make sure that it's compiling a x64 build.

5. If you want to use it outside visual studio, you can go to the `/[cmake-output-folder]/bin/` folder for the BetterVR_Layer.dll.
   The `BetterVR_Layer.json` and `Launch_BetterVR.bat` can be found in the [resources](/resources) folder.
   Then you can launch Cemu with the hook using the Launch_BetterVR.bat file to start Cemu with the hook.


### Licenses

This project is licensed under the MIT license.
BetterVR also uses the following libraries:
 - [vkroots (MIT licensed)](https://github.com/Joshua-Ashton/vkroots/blob/main/LICENSES/MIT.txt)
 - [imgui (MIT licensed)](https://github.com/ocornut/imgui/blob/master/LICENSE.txt)
 - [ImPlot3D (MIT licensed)](https://github.com/brenocq/implot3d/blob/main/LICENSE)
 - [ImPlot (MIT licensed)](https://github.com/epezent/implot/blob/master/LICENSE)
