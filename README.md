# <img width="3840" height="1037" alt="BetterVR" src="https://github.com/user-attachments/assets/4f6d6ce2-daed-4411-a5c4-8c5d288ac921" />

BetterVR is a VR mod that adds a PC-VR mode for BotW using the Wii U emulator called Cemu.

It currently supports the following features:

* Fully stereo-rendered with 6DOF, with full roomscale support.
* Full hands and arms support. You can deck yourself out in all the fanciest clothes.
* Wield weapons, torches and stalkoblins arms into combat.
* Realistically hurl bombs, rocks and other objects towards your enemy.
* Move metal objects using your magnesis powers.
* Bows, weapons, runes and more all have proper VR controls.
* Use motion controls to interact with the world to solve puzzles or start fires.
* Large mod compatibility. BetterVR only modifies the code and no game data. Most other mods should be compatible.
* And obviously a third-person mode too!

### Requirements

* Windows (x64), or Linux (x64 or ARM64).
* It supports OpenXR headsets, as long as the VR controllers have enough buttons and the runtime supports Vulkan.
* A gaming PC with a CPU that is good at single-threaded workloads (a recent Intel i5 or Ryzen 5 are recommended at least)!
* A legal copy of BotW for the Wii U, with update `V208` (shown in Cemu's game list) or `1.5.0` (shown on the title screen). It'll warn you if it's wrong though.
* A properly set up [Cemu](http://cemu.info/) emulator that's able to run the game. See [this guide](https://cemu.cfw.guide/) for more info.
  * **Before reporting issues, make sure that you have a WORKING version of the game that can go in-game on your PC before you install this mod!**

### Known Issues

* You might experience stutters when you first execute an action. Unfortunately this is shader compilation, something that due to the nature of the mod's stereo rendering has to be done separately from existing Cemu shader caches.
* Some menus and UI elements might have black backgrounds or minor transparency glitches.
* **Outside the mod's control:** Meta/Oculus Link app will have rendering issues like black wiggle-y waves and generally bad interpolation, even when you use a cable. It's HIGHLY recommended to use [ALVR](https://github.com/alvr-org/ALVR) (free), [Virtual Desktop](https://www.meta.com/en-gb/experiences/virtual-desktop/2017050365004772/) (paid, most performant) or [Steam Link](https://www.meta.com/en-gb/experiences/steam-link/5841245619310585/) (free) instead for Meta Quest headsets.

### Mod Installation

> [!NOTE]
> `BetterVR_Launcher.exe` is its own customized version of Cemu, so your regular Cemu stays as it is. It shares your games, saves, play time, controllers and your own graphic packs with it, and keeps its own settings in `settings_bettervr.xml` and its own shader caches.

The required graphic packs, including FPS++, are bundled and enabled automatically.

#### Windows

1. Download the Windows `.zip` from the [Releases](https://github.com/Crementif/BotW-BetterVR/releases) page (not the Source Code `.zip`) and extract everything into the same folder as `Cemu.exe`. Both `BetterVR_Launcher.exe` and the `BetterVR` folder should end up next to it.

2. Connect your VR headset to your PC and start Virtual Desktop, SteamVR etc. BetterVR will tell you how to set up its OpenXR runtime if it's not set properly.

3. Run `BetterVR_Launcher.exe` and double click the game in its game list while your VR headset is turned on and connected. If things went correctly, you should now be playing the game in VR!

#### Linux

1. Download the AppImage ending in `linux-x64.AppImage` for an Intel/AMD PC, or `linux-arm64.AppImage` for an ARM64 device, from the [Releases](https://github.com/Crementif/BotW-BetterVR/releases) page. Choose for the device that will run BetterVR.
2. Allow it to run as a program in your file manager's permissions.
3. Start SteamVR and make sure it's set as the active OpenXR runtime. If you use [WiVRn](https://github.com/WiVRn/WiVRn) or [Monado](https://monado.freedesktop.org/) instead, set that as the active runtime.
4. Run the AppImage with your headset connected and double click the game in its game list.

If your games or saves are missing, set `Game Paths` and `Custom MLC path` under `Options > General settings` to the same locations as your working Cemu. You can use the Tools -> Title Manager to import and export save files, though it should generally work automatically.

#### First time in VR

It'll show a big popup telling you to hold `(X)` (or the equivalent on your controller) to open the BetterVR menu. In this menu you'll find all the BetterVR settings and the controls guide. Use the stick and buttons to navigate it.

Most settings in the BetterVR menu are perfectly fine by default. If you don't want to fiddle, the most important things are basically:

* On the Play Style page, whether you're playing in first or third person, and standing or seated.
* On the Comfort page, pick Gentle if you're new to VR, Balanced for most people, or Immersive if you're used to VR. Open its Advanced section to change things one by one.
* On the System page, use `Open On Desktop` next to Graphic Packs to open Cemu's Graphic Packs window outside VR. This is where you'll also find a good number of tweaks, quality settings and mods to customize your game. Here's some that you might want to change:
  - `Graphics`: Set the VR Resolution Multiplier to increase/decrease the resolution of the game. You can usually also change this in SteamVR, Virtual Desktop, Oculus Link etc.
  - `Enhancements`: Set anisotropic filtering to 8x or 16x, and optionally set a Clarity preset to make the colors less washed out.
  - Any other setting of your choosing, like stamina, durability etc.

Restart BetterVR after changing graphic packs so the changes take effect.

### How To Update

Close Cemu and BetterVR first. On Windows, extract the entire new zip into the same folder and overwrite whenever asked. On Linux, use the new AppImage, keeping it beside the same `portable` folder if you use one.

Your settings and saves stay in place.

### Troubleshooting

Here's some steps to help you debug issues when the game fails to launch:

1. Verify that your game files are fully intact and present. In BetterVR's game list, right-click the game and use `Verify integrity of game files`.
2. Verify that you've got a working game copy by launching it normally in your regular Cemu and going in-game.
3. Make sure that if you're using a laptop, both your integrated graphics card and your dedicated graphics card drivers are updated. Make sure that it's also plugged into a wall outlet and that it's NOT using a power saving setting.
4. Unfortunately, some older AMD GPUs might have issues. See contact options below to help us squash these.
5. Temporarily disable any Vulkan overlays or layers that might cause issues. Programs like Overwolf, RTSS and other things CAN cause issues, though certainly not guaranteed.
6. On Windows, if BetterVR is missing its files, check that the whole zip was extracted and that the `BetterVR` folder is beside `BetterVR_Launcher.exe`.
7. If the headset isn't found, check that it's awake and visible in your VR software, and that the correct OpenXR runtime is active.
8. If you use other mods, try disabling them to see whether they cause the problem. Leave BetterVR's required packs enabled.

If none of these steps helped, check the BotW support channel on the [Flat2VR Discord server](https://discord.com/invite/flat2vr) (recommended) or [make a GitHub issue](https://github.com/Crementif/BotW-BetterVR/issues). Me and other people will try to help you in our spare time.

Please include your BetterVR version, operating system, CPU/GPU, headset, runtime, any extra mods and what went wrong. Attach `BetterVR_log.txt` and `log.txt` from the failed run; `File > Open Cemu folder` in BetterVR opens their location. Copy them before starting another session.

### Controls

**You can view the controls in-game by opening the BetterVR menu by holding the X button on your left VR controller.**
Tap X briefly for a 3D guide showing where your bow, weapon and runes sit on your body. This guide only appears while you're on foot.
With Valve Index Controllers you can use the A button.
With Steam Frame Controllers you can use the View button on the left controller.
For Xbox/Playstation etc. controllers, long press the Start button and use the mouse for changing settings.

You can find the controller image here (click to enlarge):  
<a href="resources/controller_help.png">
<img src="resources/controller_help.png" width="540" alt="BetterVR controls and gestures">
</a>

### Technical Overview
#### Rendering an image to the VR headset

Since this is an emulated game, and not a native PC game you might wonder how it works.

As of BetterVR 1.0, it consists of three components:
- A special Cemu fork that's capable of emitting proper stereo draw calls when instructed to do so and a few other modifications (it was very important that it is easily updateable with upstream changes)
- A C++ BetterVR module that contains almost all of the VR gameplay and VR headset code.
- A ton of assembly patches that change the game's code that make the game able to output stereo imagery, change how mechanics work in VR, cause your weapon to hit enemies, patch various undesirable qualities and way more stuff.

#### How to make it VR

Well, a ton of work is basically how you make the game talk to the VR headset.

For example, we hook the code that decides whether a weapon can hit something. When the game reaches that point, it calls our C++ module, which checks your controller movements to decide whether you're swinging.

The trickiest part is always finding the right place to hook something. It requires a lot of knowledge about how a game might function, and how ideally we patch it to enable a great VR-native feeling. Over the 5 years that this mod has been in development for, this has taken thousands of hours of reverse-engineering, and I think is what probably contributes to the great result.

If you want to know more about the technical details, feel free to ask in the BetterVR related channels in the [Flat2VR Discord server](https://discord.com/invite/flat2vr).
There's enough that was skipped over or left out in this explanation.

### Build Instructions (For Developers)

You'll need Git, CMake 3.27 or newer, Ninja and a C++23 toolchain. Use an x64 Visual Studio developer shell on Windows. See the [build workflow](.github/workflows/build.yml) for Linux dependencies and toolchain versions.

```
git submodule update --init --recursive
```

Bootstrap vcpkg with `cemu-src\dependencies\vcpkg\bootstrap-vcpkg.bat` on Windows, or `bash cemu-src/dependencies/vcpkg/bootstrap-vcpkg.sh` on Linux. Then, from the repository root:

```
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
Elliottate: Provided various parts for optimizations  
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
