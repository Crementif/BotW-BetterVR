cmake_minimum_required(VERSION 3.27.0)

# BetterVR is Cemu with the mod compiled in. This folder is one CMake workspace: its build is the Cemu fork in
# cemu-src/, which adds src/ as a host module through CEMU_EXTERNAL_MODULES. See docs/cemu-fork.md.
set(BETTERVR_CEMU_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cemu-src")
if (NOT EXISTS "${BETTERVR_CEMU_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "cemu-src/ is empty. Fetch the Cemu fork with: git submodule update --init --recursive")
endif ()

# Cemu sets up vcpkg before its project(), which is too late from a subdirectory, so the workspace does it with
# Cemu's manifest and overlay ports
option(ENABLE_VCPKG "Enable the vcpkg package manager" ON)
if (ENABLE_VCPKG)
    if (UNIX AND NOT APPLE)
        set(VCPKG_OVERLAY_PORTS "${BETTERVR_CEMU_DIR}/dependencies/vcpkg_overlay_ports_linux;${BETTERVR_CEMU_DIR}/dependencies/vcpkg_overlay_ports")
    elseif (APPLE)
        set(VCPKG_OVERLAY_PORTS "${BETTERVR_CEMU_DIR}/dependencies/vcpkg_overlay_ports_mac;${BETTERVR_CEMU_DIR}/dependencies/vcpkg_overlay_ports")
    else ()
        set(VCPKG_OVERLAY_PORTS "${BETTERVR_CEMU_DIR}/dependencies/vcpkg_overlay_ports_win;${BETTERVR_CEMU_DIR}/dependencies/vcpkg_overlay_ports")
    endif ()
    set(VCPKG_MANIFEST_DIR "${BETTERVR_CEMU_DIR}")
    set(CMAKE_TOOLCHAIN_FILE "${BETTERVR_CEMU_DIR}/dependencies/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
    set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)
    if (WIN32)
        set(VCPKG_TARGET_TRIPLET "x64-windows-static" CACHE STRING "")
    endif ()
endif ()
set(THREADS_PREFER_PTHREAD_FLAG true)

project(BetterVR VERSION 1.0.0 LANGUAGES C CXX)

set(CEMU_EXTERNAL_MODULES "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE STRING "Source directories of modules to compile into Cemu")
add_subdirectory("${BETTERVR_CEMU_DIR}" cemu)

# Cemu sets its startup project in its own directory scope, which the workspace root never sees, so Visual Studio
# would otherwise have no executable to Run.
if (MSVC)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT CemuBin)
endif ()

# Cemu finds its settings, graphic packs and saves from its executable's own path, so the build writes Cemu straight
# into Cemu/. A symlink or a copy elsewhere would start it in the wrong data folder. A vulkan-1.dll left there by the
# old launcher would load a second copy of the mod.
set(BETTERVR_DEV_CEMU_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Cemu")
set_target_properties(CemuBin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${BETTERVR_DEV_CEMU_DIR}")
install(CODE "file(REMOVE \"${BETTERVR_DEV_CEMU_DIR}/vulkan-1.dll\")")
install(DIRECTORY "${BETTERVR_CEMU_DIR}/bin/resources" "${BETTERVR_CEMU_DIR}/bin/gameProfiles" DESTINATION "${BETTERVR_DEV_CEMU_DIR}/BetterVR")

option(BETTERVR_BUILD_TESTS "Build focused host-side tests" OFF)
if (BETTERVR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()
