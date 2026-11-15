# <img width="3840" height="1037" alt="BetterVR" src="https://github.com/user-attachments/assets/4f6d6ce2-daed-4411-a5c4-8c5d288ac921" />

BetterVR is a VR mod that adds a PC-VR mode for BotW using the Wii U emulator called Cemu.

It currently supports the following features:
* Fully stereo-rendered with 6DOF, with full roomscale support. No alternated eye rendering is used.
* Full hands and arms support. You can deck yourself out in all the fanciest clothes.
* Wield weapons, torches and bokoblin arms into combat.
* Gestures to equip and throw weapons.
* Use motion controls to interact with the world to solve puzzles or start fires.
* Large mod compatibility. BetterVR only modifies the code and no game data. Most other mods should be compatible.
* Optional third-person mode.

### Requirements

* It supports all OpenXR headsets, as long as the VR controllers have enough buttons.

* A gaming PC with a CPU that is good at single-threaded workloads (a recent Intel i5 or Ryzen 5 are recommended at least)!

* A legal copy of BotW for the Wii U, with update `V208` (shown in Cemu's game list) or `1.5.0` (shown on the title screen). It'll warn you if it's wrong though.

* A properly set up [Cemu](http://cemu.info/) emulator that's able to run the game. See [this guide](https://cemu.cfw.guide/) for more info.
  * **Before reporting issues, make sure that you have a WORKING version of the game that can go in-game on your PC before you install this mod!**

### Known Issues

* The first launch after installing or updating stutters more than usual, since BetterVR keeps its own shader caches and has to rebuild them. This goes away after a bit of playing.

### Mod Installation

> [!TIP]
> Meta/Oculus Link will run slower while having a chance of making the grass/hands leave black wiggle-y waves, even while using a cable.
> It's HIGHLY recommended to use [ALVR](https://github.com/alvr-org/ALVR) (free), [Virtual Desktop](https://www.meta.com/en-gb/experiences/virtual-desktop/2017050365004772/) (paid, most performant) or [Steam Link](https://www.meta.com/en-gb/experiences/steam-link/5841245619310585/) (free) instead for Meta Quest headsets.

> [!NOTE]
> `BetterVR_Launcher.exe` is its own customized version of Cemu, so your regular Cemu stays as it is. It shares your games, saves, play time, controllers and your own graphic packs with it, and keeps its own settings in `settings_bettervr.xml` and its own shader caches.

1. Download the latest release from the [Releases](https://github.com/Crementif/BotW-BetterVR/releases) page and extract it into the same folder as `Cemu.exe`.

2. Connect your VR headset to your PC and start Virtual Desktop, SteamVR etc. BetterVR will tell you how to set up its OpenXR runtime if it's not set properly.

3. Run `BetterVR_Launcher.exe` and double click the game in its game list while your VR headset is turned on and connected. If things went correctly, you should now be playing the game in VR!

4. It'll show a big popup telling you to hold `(X)` (or the equivalent on your controller) to open the BetterVR menu. In this menu you'll find all the BetterVR settings and the controls guide.

5. Most settings in the BetterVR menu are perfectly fine by default. If you don't want to fiddle, the most important things are basically:
   - On the Play Style page, whether you're playing in first or third person, and standing or seated.
   - On the Comfort page, pick Gentle if you're new to VR, Balanced for most people, or Immersive if you're used to VR. Open its Advanced section to change things one by one.
   - On the System page, use `Open On Desktop` next to Graphic Packs to open Cemu's Graphic Packs window outside VR. This is where you'll also find a good number of tweaks, quality settings and mods to customize your game. Here's some that you might want to change:
     - `Graphics`: Set the VR Resolution Multiplier to increase/decrease the resolution of the game. You can usually also change this in SteamVR, Virtual Desktop, Oculus Link etc.
     - `Enhancements`: Set anisotropic filtering to 8x or 16x, and optionally set a Clarity preset to make the colors less washed out.
     - Any other setting of your choosing, like stamina, durability etc.

#### Troubleshooting

Here's some steps to help you debug issues when the game fails to launch:

1. Verify that your game files are fully intact and present. Open `BetterVR_Launcher.exe`, right-click the game and use `Verify integrity of game files`.
2. Verify that you've got a working game copy by launching it normally in Cemu.exe and going in-game.
3. Make sure that if you're using a laptop, both your integrated graphics card and your dedicated graphics card drivers are updated. Make sure that it's also plugged into a wall outlet and that it's NOT using a power saving setting.
4. Unfortunately, some older AMD GPUs might have issues. See contact options below to help us squash these.
5. Uninstall any Vulkan overlays or layers that might cause issues. Programs like Overwolf, RTSS and other things CAN cause issues, though certainly not guaranteed.
6. If Cemu shows a Vulkan error like -13, these issues are likely GPU driver related.

If none of these steps helped, check the BotW support channel on the [Flat2VR Discord server](https://discord.com/invite/flat2vr) (recommended) or make a GitHub issue. Me and other people will try to help you in our spare time.

### Controls

**You can view the controls in-game by opening the BetterVR menu by holding the X button on your left VR controller.**
With Valve Index Controllers you can use the A button.
With Steam Frame Controllers you can use the View button on the left controller. The Frame's D-pad, bumpers and lettered buttons also work like a Wii U gamepad's, while swinging, drawing the bow and grabbing stay motion controls.
For Xbox/Playstation etc. controllers, long press the Start button and use the mouse for changing settings.

You can find the controller image here (click to enlarge):  
<a href="https://raw.githubusercontent.com/Crementif/BotW-BetterVR/refs/heads/main/resources/controller_help.png">
<img src="https://raw.githubusercontent.com/Crementif/BotW-BetterVR/refs/heads/main/resources/controller_help.png" width="540">
</a>

### Technical Overview
#### Rendering an image to the VR headset
This mod ships with no game files, so you might ask how it works.

Up until 0.9.23, BetterVR was a Vulkan layer, which came in the form of a .DLL file, that intercepted the Vulkan commands that Cemu submits. Since OpenXR frameworks aren't designed to be instantiated inside something that is intercepting Vulkan commands, it used Vulkan <-> D3D12 interop to pipe the rendered output to a D3D12 application that was *just* used for rendering the captured image to the VR headset.

Using an external DLL originally made a lot of sense when Cemu wasn't open-sourced. In hindsight, it probably would've saved a lot of time spent trying to get the mod to work without using D3D12. So `BetterVR_Launcher.exe` is now its own build of Cemu, a thin fork found in `cemu-src` with the mod compiled into it. The game runs each frame once, and Cemu draws that same frame a second time for the other eye with that eye's camera. It also means that the mod can talk to the VR headset directly, which is what made a Linux version possible.

#### How to make it VR
However, while drawing the game's rendered output to the VR headset is one thing, getting a native game to render a 3D image is a whole other thing. For that, the mod has a bunch of PowerPC assembly patches (the Wii U has a PowerPC CPU) to modify the game's code. Among many other patches, you'll find patches that change the camera or player model positions each frame, or trigger an attack.

Usually the assembly code will call into the C++ code if it wants to do complicated algebra to specify where the camera or Link's hands should be for example.

Additionally, since combat is a large part of the original game, there's also a new swing and stab detection system that allows the player to cut trees and enemies down when they execute proper swings and stabbing motions. This prevents a situation where weapon hitboxes are abused to instantly stagger an enemy. There's plans for an even deeper integration, but as of today that's about it. This is fully optional since the mod still features an attack button, but the latter will offer a lot more immersion.

Understanding how the game works, finding and patching the exact parts inside the game's executable is by far the most difficult part and it took thousands of hours of reverse-engineering. Its without a doubt the most time consuming task of this VR mod, especially since this game uses a custom C++ engine of which is not much known about other then the good work of the (largely unfinished, but still very helpful) decompilation project.

If you want to know more about the technical details, feel free to ask in the BetterVR related channels in the [Flat2VR Discord server](https://discord.com/invite/flat2vr).
There's enough that was skipped over or left out in this explanation.

### Build Instructions (For Developers)

```
git submodule update --init --recursive
cmake --preset Release
cmake --build cmake-build-Release --target CemuBin
```

`cemu-src` is a thin fork of Cemu that adds a host module interface, stereo rendering and foveated shading. The mod in `src/` is compiled into it.

### Credits
Crementif: Main Developer  
Acudofy: Sword & stab analysis system  
Holydh: Developed the input systems  
leoetlino: For the [BotW Decomp project](https://github.com/zeldaret/botw), which was very useful  
Exzap: Technical support and optimization help  
Mako Marci: Edited the trailer, made the logo and controller guide  
Tim, Mako Marci, Solarwolf07, Elliott Tate & Derra: Helped with testing, recording, feedback and support  
WunderbarVR: Made cool trailers  

### Licenses

This project is licensed under the MIT license. Cemu is licensed under the MPL 2.0.
BetterVR also uses the following libraries:
 - [vkroots (MIT licensed)](https://github.com/Joshua-Ashton/vkroots/blob/main/LICENSES/MIT.txt)
 - [imgui (MIT licensed)](https://github.com/ocornut/imgui/blob/master/LICENSE.txt)
 - [ImPlot (MIT licensed)](https://github.com/epezent/implot/blob/master/LICENSE)
 - [OpenXR loader (Apache 2.0 licensed)](https://github.com/KhronosGroup/OpenXR-SDK/blob/main/LICENSE)
 - [Roboto font (OFL 1.1 licensed)](https://github.com/googlefonts/roboto-classic/blob/main/OFL.txt)
