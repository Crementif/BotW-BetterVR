# <img width="3840" height="1037" alt="BetterVR" src="https://github.com/user-attachments/assets/4f6d6ce2-daed-4411-a5c4-8c5d288ac921" />

BetterVR is a PC-VR mode for Breath of the Wild, built into a fork of the Wii U emulator [Cemu](https://github.com/cemu-project/Cemu).

* Fully stereo-rendered with 6DOF and roomscale support. No alternated eye rendering.
* Full hands and arms, motion-controlled weapons, gestures and puzzle interactions.
* Only changes the game's code, so most other mods stay compatible.
* Optional third-person mode.

### Requirements

* An OpenXR headset whose controllers have enough buttons.
* A CPU that is good at single-threaded workloads. The CPU is the bottleneck, not the GPU.
* A legal copy of BotW for the Wii U, with update `V208`.

### Installation

1. Download the latest release and extract it into the folder that has your `Cemu.exe`.
2. Make sure your headset's OpenXR runtime is active (SteamVR, Virtual Desktop, ALVR, Meta Link).
3. Run `BetterVR_Launcher.exe` and start BotW with the headset connected.

`BetterVR_Launcher.exe` is its own build of Cemu, so your regular Cemu stays as it is. It shares your games, saves, play time, controllers and your own graphic packs with it, and keeps its own settings in `settings_bettervr.xml` and its own shader caches.

Hold X on the left controller (A on Index controllers, long press Start on a gamepad) to open the BetterVR menu with the settings and controls.

### Building

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

BetterVR is licensed under the MIT license. Cemu is licensed under the MPL 2.0.
BetterVR also uses [vkroots](https://github.com/Joshua-Ashton/vkroots/blob/main/LICENSES/MIT.txt), [imgui](https://github.com/ocornut/imgui/blob/master/LICENSE.txt) and [ImPlot](https://github.com/epezent/implot/blob/master/LICENSE), all MIT licensed.
