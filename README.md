# <img width="3840" height="1037" alt="BetterVRLogo(1)" src="https://github.com/user-attachments/assets/4f6d6ce2-daed-4411-a5c4-8c5d288ac921" />

BetterVR is a VR mod/hook that adds a PC-VR mode for BotW using the Wii U emulator called Cemu.

It currently supports the following features:
* Fully stereo-rendered with 6DOF. No alternated eye rendering is used.
* Full hands and arms support. You can deck yourself out in all the fanciest clothes.
* Wield weapons, torches and bokoblin arms into combat.
* Gestures to equip and throw weapons.
* Use motion controls to interact with the world to solve puzzles or start fires.
* Large mod compatibility. BetterVR only modifies the code and no game data. Most other mods should be compatible.
* Optional third-person mode (though its a bit broken at the moment).

### Requirements

#### Supported VR headsets:

The app currently utilizes OpenXR, which is supported on all the major headsets (Valve Index, HTC Vive, Oculus Rift, Meta Quest,
Windows Mixed Reality etc.). However, controller bindings are currently only provided for Oculus Touch controllers.
While more integrated solutions are being found out, there's probably ways to setup OpenXR mappings through SteamVR or other applications.

#### Other Requirements:

* A gaming PC with a CPU that is good at single-threaded workloads (a recent Intel i5 or Ryzen 5 are recommended at least)!

* A legal copy of BotW for the Wii U.

* Windows OS

* A properly set up [Cemu](http://cemu.info/) emulator that's able to run at 60FPS or higher. See [this guide](https://cemu.cfw.guide/) for more info.
  * **Before reporting issues, make sure that you have a WORKING version of the game that can go in-game on your PC before you install this mod!**  

* A recent Cemu version. Only Cemu 2.6 is tested to work.

> [!WARNING]
> ### Current Limitations & Known Issues
> Since this is an unofficial mod and not a VR port, some things do not work perfectly (yet).  
> Some issues will be much easier to fix then others.  
> The game is fully tested to be completeable, from start to finish.
> 
> If you want to help to improve the mod and tackle some of these issues, reach out in the ZBW Development Channel in the [Flat2VR Discord](https://discord.com/invite/flat2vr) for extra info, context and requirements!
>
> **Critical Issues:**
> * Climbing ladders requires looking away with the camera using your controller stick.
> * **Weapon Glitch:** Sometimes weapons will stop registering hits on enemies.
>   * *Quick Fix:* Quickly drop and pick up your weapon (`Right Grip` + `R-Stick Down`).
>   * *Alt Fix:* Throw the weapon or cycle through weapons in the inventory menu.
>   * *Last Resort:* Teleport to a tower/shrine or reload your save.
> * Third-person mode (and cutscenes) often has the player being partially/largely invisible.
> * Our AMD GPU system has a crash after the load screen, which we're working on fixing.
> * Gravity is higher. Jumping isn't affected, but some shrines might require creative solutions/glitches for now.

**Audio & Visuals**
- Slight audio crackling may occur when loading the game or opening menus quickly.
- Video cutscenes are slow and may have out-of-sync audio. Some voice-acted cutscenes may also overlap or play out of sync.
- While inside the Divine Beasts, skyboxes appear to sway with the camera more then intended.

**Gameplay & Combat**
- Flurry Rush can be triggered but does not work
- Bow Aiming is done via a crosshair on the VR headset. Bow support might be added at some point.
- Enemies will ocassionally not detect you
- No roomscale support. You can freely move around your room, but enemies and physics will use your center point. 

**Traversal & Physics**
- Exiting the water while swimming can be difficult at certain angles. Swim dashing sometimes doesn't work. 
- Magnesis & Stasis aim is off-center at far distances. Point your gaze to the **right** of the object to highlight it.

### Mod Installation

1. Download the latest release of the mod from the [Releases](https://github.com/Crementif/BotW-BetterVR/releases) page.

2. Extract the contents of the downloaded `.zip` file into the same folder where your `Cemu.exe` is stored.
   There should now be **at least** .dll, .json and multiple .bat files in the same folder as your `Cemu.exe`.
   Make sure you start Cemu.exe once so that it can create the folder structure, if you've never done that before.

3. Double-click on `BetterVR LAUNCH CEMU IN VR.bat` to start Cemu. This'll install the graphic pack automatically to the right folder.

4. Go to `Options`-> `Graphic packs`-> `The Legend of Zelda: Breath of the Wild` and make sure that the graphic pack named `BetterVR` is enabled.
   This is ALSO where you can change any VR settings like the first/third-person mode etc.
   **You'll also want to enable the FPS++ graphic pack, or else the game will crash!**
   **You can't change the BetterVR options while you're in-game.**

5. (Recommended) For an enjoyable experience you should change some other graphic packs in this same window too:
   - `Graphics` graphic pack: Use any (non-ultrawide!) resolution of 1440p (2k) or higher for clarity. Also change anti-aliasing to Nvidia FXAA.
   - `FPS++` graphic pack: Change the FPS limit to at least 120FPS or 144FPS. The OpenXR headset will dictate the framerate anyway.
   - `Enhancements`: graphic pack: Change anisotropic filtering to 16x and use your preferred preset.

6. Close the settings and start the game like normal from Cemu's game list. You can now put on your VR headset and if installed correctly it should now work!

From now on you can play the game in VR by just starting the `BetterVR LAUNCH CEMU IN VR.bat` file.  
If you want to undo the installation (temporarily) to play the game without VR, use the `BetterVR UNINSTALL.bat` file.  
You can just use the `BetterVR LAUNCH CEMU IN VR.bat` file to reinstall and start the VR mod again.

### Controls

<img width="2366" height="3423" alt="image" src="https://github.com/user-attachments/assets/ff44b2d8-ef64-417f-8f72-f7ff887f00c2" />


---

### Technical Overview
#### Rendering an image to the VR headset
This mod ships with no game files, so you might ask how it works.

The game starts with the BetterVR Vulkan layer enabled. The Vulkan layer, which comes in the form of the .DLL file, is then able to intercept the Vulkan commands that Cemu submits so that we can get the final frame to render to the VR headset and draw the debugging tools.

A technical hurdle here was that due to OpenXR frameworks not being designed to be instantiated inside something that is intercepting Vulkan commands, this mod utilizes Vulkan <-> D3D12 interop to pipe the rendered output from Vulkan to a D3D12 application that's *just* used for rendering the captured image to the VR headset. That way the OpenXR framework is just interacting with root-level rendering handles, instead of what'd occur in the Vulkan hook.

Using an external DLL originally made a lot of sense when Cemu wasn't open-sourced (though it also makes it slightly less tied to a specific emulator or version of Cemu, and prevents a VR specific version of Cemu that'll quickly become outdated). In hindsight, it probably would've saved a lot of time spent trying to get the mod to work without using D3D12.

#### How to make it VR
However, while drawing the game's rendered output to the VR headset is one thing, getting a native game to render a 3D image is a whole other thing. For that, the mod has a bunch of PowerPC assembly patches (the Wii U has a PowerPC CPU) to modify the game's code. For example, an important patch is to make it so that the game renders two frames before updating all of the game's systems and objects that are on-screen. Then, among many other patches, you'll also find patches that change the camera or player model positions each frame, or trigger an attack.

Usually the assembly code will call into the C++ code if it wants to do complicated algebra to specify where the camera or Link's hands should be for example. And some assembly patches use a clearing instruction for the Wii U's GPU which, after being translated, will signal the Vulkan hook to send the almost-finished final game image to the D3D12 code where it can present it inside the VR headset.

Additionally, since combat is a large part of the original game, there's also a new swing and stab detection system that allows the player to cut trees and enemies down when they execute proper swings and stabbing motions. This prevents a situation where weapon hitboxes are abused to instantly stagger an enemy. There's plans for an even deeper integration, but as of today that's about it. This is fully optional since the mod still features an attack button, but the latter will offer a lot more immersion.

Understanding how the game works, finding and patching the exact parts inside the game's executable is by far the most difficult part and it took thousands of hours of reverse-engineering. Its without a doubt the most time consuming task of this VR mod, especially since this game uses a custom C++ engine of which is not much known about other then the good work of the (largely unfinished, but still very helpful) decompilation project.

If you want to know more about the technical details, feel free to ask in the BetterVR related channels in the [Flat2VR Discord server](https://discord.com/invite/flat2vr).
There's enough that was skipped over or left out in this explanation.


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


### Credits
Crementif: Main Developer  
Acudofy: Sword & stab analysis system  
Holydh: Developed some of the new input systems  
leoetlino: For the [BotW Decomp project](https://github.com/zeldaret/botw), which was very useful  
Exzap: Technical support and optimization help  
Mako Marci: Edited the trailer  
Tim, Mako Marci, Solarwolf07 & Elliott Tate: Helped with testing, recording, feedback and support  

### Licenses

This project is licensed under the MIT license.
BetterVR also uses the following libraries:
 - [vkroots (MIT licensed)](https://github.com/Joshua-Ashton/vkroots/blob/main/LICENSES/MIT.txt)
 - [imgui (MIT licensed)](https://github.com/ocornut/imgui/blob/master/LICENSE.txt)
 - [ImPlot3D (MIT licensed)](https://github.com/brenocq/implot3d/blob/main/LICENSE)
 - [ImPlot (MIT licensed)](https://github.com/epezent/implot/blob/master/LICENSE)
