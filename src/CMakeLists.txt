# The mod as a host module of the Cemu fork. Cemu adds this folder through
# CEMU_EXTERNAL_MODULES and links the BetterVR library into its executable.
get_filename_component(BETTERVR_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

if (NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Only 64-bit builds are supported")
endif ()

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Cemu links the ImGui core, so the headers the mod compiles against have to be Cemu's
foreach (imguiHeader imgui.h imgui_internal.h imconfig.h)
    file(READ "${BETTERVR_ROOT}/dependencies/imgui/${imguiHeader}" bettervrHeader)
    file(READ "${Cemu_SOURCE_DIR}/dependencies/imgui/${imguiHeader}" cemuHeader)
    string(REPLACE "\r" "" bettervrHeader "${bettervrHeader}")
    string(REPLACE "\r" "" cemuHeader "${cemuHeader}")
    if (NOT bettervrHeader STREQUAL cemuHeader)
        message(FATAL_ERROR "dependencies/imgui/${imguiHeader} is not the one Cemu uses. Copy in the ImGui commit Cemu's dependencies/imgui is at.")
    endif ()
endforeach ()
add_subdirectory("${BETTERVR_ROOT}/dependencies" "${CMAKE_CURRENT_BINARY_DIR}/dependencies")

add_library(BetterVR STATIC
    instance.h
    shader.h
    utils/vulkan_utils.h
    utils/logger.cpp
    utils/logger.h
    utils/platform.h
    utils/controller_bindings.h
    utils/mod_settings.h
    utils/mod_settings.cpp
    utils/profiler.h
    utils/render_utils.h
    utils/debug_draw.h
    utils/debug_draw.cpp
    utils/eye_resolution.h
    utils/update_check.h
    utils/update_check.cpp

    cemu/capture.h
    cemu/capture.cpp
    cemu/cemu_host.h
    cemu/cemu_host.cpp
    cemu/cemu_settings.h
    cemu/cemu_settings.cpp
    cemu/guest_memory.h
    cemu/host_module.cpp
    cemu/vulkan_device.cpp
    cemu/vulkan_device.h
    cemu/vulkan_loader.h
    cemu/host_window.h
    cemu/host_window.cpp

    game/game.h
    game/botw/game.cpp
    game/botw/game_state.h
    game/botw/game_state.cpp
    game/botw/game_utils.h
    game/botw/pack_shared_memory.h
    game/botw/camera/camera.h
    game/botw/camera/camera.cpp
    game/botw/camera/stereo_view.h
    game/botw/camera/stereo_view.cpp
    game/botw/player/player.h
    game/botw/player/player.cpp
    game/botw/player/skeleton.h
    game/botw/player/skeleton.cpp
    game/botw/player/link_skeleton.h
    game/botw/player/link_skeleton.cpp
    game/botw/player/head_visibility.h
    game/botw/player/head_visibility.cpp
    game/botw/player/weapon.h
    game/botw/player/weapon.cpp
    game/botw/player/weapon_motion.h
    game/botw/player/weapon_motion.cpp
    game/botw/player/bow.h
    game/botw/player/bow.cpp
    game/botw/player/hand_aim.h
    game/botw/player/hand_aim.cpp
    game/botw/player/magnesis.h
    game/botw/player/magnesis.cpp
    game/botw/player/roomscale.h
    game/botw/player/roomscale.cpp
    game/botw/input/controls.h
    game/botw/input/controls.cpp
    game/botw/input/button_state.h
    game/botw/input/openxr_motion_bridge.h
    game/botw/interface/interface_hooks.h
    game/botw/interface/interface_hooks.cpp
    game/botw/debug/debug_hooks.h
    game/botw/debug/debug_hooks.cpp
    game/botw/debug/entity_debugger.h
    game/botw/debug/entity_debugger.cpp
    game/botw/debug/weapon_attack_debugger.h
    game/botw/debug/weapon_attack_debugger.cpp

    rendering/vk_backend.cpp
    rendering/vk_backend.h
    rendering/renderer.cpp
    rendering/renderer.h
    rendering/openxr.cpp
    rendering/openxr.h
    rendering/rumble.h
    rendering/haptics.h
    rendering/haptics.cpp
    rendering/tracking.h
    rendering/tracking.cpp
    rendering/swapchain.cpp
    rendering/swapchain.h
    rendering/texture.cpp
    rendering/texture.h
    rendering/vulkan.cpp
    rendering/vulkan.h
    rendering/vulkan_imgui.cpp
    rendering/imgui_menus.h
    rendering/imgui_menus.cpp
    rendering/supporters_generated.h
)
if (WIN32)
    target_sources(BetterVR PRIVATE utils/platform_win32.cpp)
else ()
    target_sources(BetterVR PRIVATE utils/platform_linux.cpp)
endif ()
set_property(GLOBAL APPEND PROPERTY CEMU_HOST_MODULES BetterVR)

set_target_properties(BetterVR PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
if (MSVC)
    target_compile_definitions(BetterVR PRIVATE NOMINMAX)
    target_compile_options(BetterVR PRIVATE /utf-8)
endif ()
target_precompile_headers(BetterVR PRIVATE "${BETTERVR_ROOT}/include/pch.h")
target_include_directories(BetterVR BEFORE PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
target_include_directories(BetterVR PRIVATE "${BETTERVR_ROOT}/include" "${BETTERVR_ROOT}/dependencies")
# only the host module interface, so Cemu's source tree can't shadow the mod's own headers
set_source_files_properties(cemu/host_module.cpp PROPERTIES INCLUDE_DIRECTORIES "${Cemu_SOURCE_DIR}/src")

find_package(glm CONFIG REQUIRED)
target_link_libraries(BetterVR PRIVATE BetterVR_ImPlot BetterVR_ImGuiVulkan OpenXR::openxr_loader glm::glm fmt::fmt pugixml::pugixml CURL::libcurl)
# std::atomic of structs wider than 8 bytes calls into libatomic outside of MSVC
if (NOT MSVC)
    target_link_libraries(BetterVR PRIVATE atomic)
endif ()

# the version comes from the workspace's project() line, so a standalone Cemu configure gets it too
set(BETTERVR_VERSION_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/version")
file(STRINGS "${BETTERVR_ROOT}/CMakeLists.txt" bettervrProjectLine REGEX "^project\\(BetterVR VERSION ")
string(REGEX MATCH "VERSION ([0-9.]+)" bettervrVersionMatch "${bettervrProjectLine}")
if (DEFINED ENV{GITHUB_ACTIONS} OR DEFINED ENV{CI})
    set(BETTERVR_CI_BUILD_DEFAULT ON)
else ()
    set(BETTERVR_CI_BUILD_DEFAULT OFF)
endif ()
option(BETTERVR_CI_BUILD "Build as a CI build, which leaves the commit hash out of the version string" ${BETTERVR_CI_BUILD_DEFAULT})
if (NOT BETTERVR_CI_BUILD)
    target_compile_definitions(BetterVR PRIVATE BETTERVR_PACKS_DIR="${BETTERVR_ROOT}/resources")
endif ()
add_custom_target(BetterVR_Version
    COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${BETTERVR_ROOT} -DOUTPUT_HEADER=${BETTERVR_VERSION_DIR}/bettervr_version.h -DVERSION=${CMAKE_MATCH_1} -DCI_BUILD=${BETTERVR_CI_BUILD} -P "${BETTERVR_ROOT}/cmake/generate_version_header.cmake"
    BYPRODUCTS "${BETTERVR_VERSION_DIR}/bettervr_version.h"
    COMMENT "Resolving BetterVR build version"
    VERBATIM
)
add_dependencies(BetterVR BetterVR_Version)
target_include_directories(BetterVR PRIVATE "${BETTERVR_VERSION_DIR}")

# GLSL -> SPIR-V. glslc regenerates the checked-in binaries in place so they can never ship stale; without the
# Vulkan SDK the build takes them as checked in.
set(BETTERVR_SHADER_DIR "${BETTERVR_ROOT}/resources/shaders")
set(BETTERVR_SHADER_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/shaders")
find_program(BETTERVR_GLSLC NAMES glslc HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin")
if (NOT BETTERVR_GLSLC)
    message(STATUS "glslc not found, using the SPIR-V binaries checked in under ${BETTERVR_SHADER_DIR}/prebuilt")
endif ()
set(BETTERVR_SHADER_SPIRV "")
foreach (shader present.vert present.frag present_depth.frag debug_draw.vert debug_draw.frag)
    set(shaderBinary "${BETTERVR_SHADER_DIR}/prebuilt/${shader}.spv")
    list(APPEND BETTERVR_SHADER_SPIRV "${shaderBinary}")
    if (BETTERVR_GLSLC)
        add_custom_command(
            OUTPUT "${shaderBinary}"
            COMMAND "${BETTERVR_GLSLC}" -O --target-env=vulkan1.0 -o "${shaderBinary}" "${BETTERVR_SHADER_DIR}/${shader}"
            DEPENDS "${BETTERVR_SHADER_DIR}/${shader}"
            COMMENT "Compiling ${shader} -> SPIR-V"
            VERBATIM
        )
    endif ()
    target_sources(BetterVR PRIVATE "${BETTERVR_SHADER_DIR}/${shader}")
    set_source_files_properties("${BETTERVR_SHADER_DIR}/${shader}" PROPERTIES HEADER_FILE_ONLY TRUE)
endforeach ()
add_custom_command(
    OUTPUT "${BETTERVR_SHADER_HEADER_DIR}/shaders_spirv.h"
    COMMAND ${CMAKE_COMMAND} "-DSPIRV_FILES=${BETTERVR_SHADER_SPIRV}" "-DOUTPUT=${BETTERVR_SHADER_HEADER_DIR}/shaders_spirv.h" -P "${BETTERVR_ROOT}/cmake/generate_shader_header.cmake"
    DEPENDS ${BETTERVR_SHADER_SPIRV} "${BETTERVR_ROOT}/cmake/generate_shader_header.cmake"
    COMMENT "Generating shaders_spirv.h"
    VERBATIM
)
target_sources(BetterVR PRIVATE "${BETTERVR_SHADER_HEADER_DIR}/shaders_spirv.h")
target_include_directories(BetterVR PRIVATE "${BETTERVR_SHADER_HEADER_DIR}")

# The graphic pack, shown in the IDE. Its C++ sources compile to patch_compiled.asm with CemuPatchCompiler, a
# 58 MB Windows binary that isn't tracked, so without it (or wine off Windows) the checked-in file is used.
set(BETTERVR_PACK_DIR "${BETTERVR_ROOT}/resources/BreathOfTheWild_BetterVR")
file(GLOB_RECURSE BETTERVR_PACK_FILES CONFIGURE_DEPENDS "${BETTERVR_PACK_DIR}/*")
list(FILTER BETTERVR_PACK_FILES EXCLUDE REGEX "CemuPatchCompiler")
source_group(TREE "${BETTERVR_PACK_DIR}" PREFIX "BreathOfTheWild_BetterVR" FILES ${BETTERVR_PACK_FILES})
set_source_files_properties(${BETTERVR_PACK_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
target_sources(BetterVR PRIVATE ${BETTERVR_PACK_FILES})

# Like the BotW Mod Loader, take the newest build of a PatchCompiler checkout beside the main checkout,
# and fall back to a copy in the pack folder.
execute_process(COMMAND git rev-parse --path-format=absolute --git-common-dir WORKING_DIRECTORY "${BETTERVR_ROOT}" OUTPUT_VARIABLE BETTERVR_GIT_COMMON_DIR OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if (BETTERVR_GIT_COMMON_DIR)
    get_filename_component(BETTERVR_CHECKOUTS_DIR "${BETTERVR_GIT_COMMON_DIR}/../.." ABSOLUTE)
else ()
    get_filename_component(BETTERVR_CHECKOUTS_DIR "${BETTERVR_ROOT}/.." ABSOLUTE)
endif ()
file(GLOB BETTERVR_PATCH_COMPILER_BUILDS "${BETTERVR_CHECKOUTS_DIR}/PatchCompiler*/*/CemuPatchCompiler-release.exe")
set(BETTERVR_PATCH_COMPILER "")
set(BETTERVR_PATCH_COMPILER_TIME "")
foreach (BETTERVR_CANDIDATE ${BETTERVR_PATCH_COMPILER_BUILDS})
    file(TIMESTAMP "${BETTERVR_CANDIDATE}" BETTERVR_CANDIDATE_TIME "%Y%m%d%H%M%S" UTC)
    if (BETTERVR_CANDIDATE_TIME STRGREATER BETTERVR_PATCH_COMPILER_TIME)
        set(BETTERVR_PATCH_COMPILER "${BETTERVR_CANDIDATE}")
        set(BETTERVR_PATCH_COMPILER_TIME "${BETTERVR_CANDIDATE_TIME}")
    endif ()
endforeach ()
if (NOT BETTERVR_PATCH_COMPILER)
    find_file(BETTERVR_PATCH_COMPILER NAMES CemuPatchCompiler-release.exe CemuPatchCompiler-debug.exe PATHS "${BETTERVR_PACK_DIR}" NO_DEFAULT_PATH NO_CACHE)
endif ()
message(STATUS "CemuPatchCompiler: ${BETTERVR_PATCH_COMPILER}")
set(BETTERVR_PATCH_COMPILER_LAUNCHER "")
if (BETTERVR_PATCH_COMPILER AND NOT WIN32)
    find_program(BETTERVR_WINE NAMES wine64 wine)
    if (BETTERVR_WINE)
        set(BETTERVR_PATCH_COMPILER_LAUNCHER "${BETTERVR_WINE}")
    else ()
        set(BETTERVR_PATCH_COMPILER "")
    endif ()
endif ()
if (BETTERVR_PATCH_COMPILER)
    file(GLOB BETTERVR_PACK_SOURCES CONFIGURE_DEPENDS "${BETTERVR_PACK_DIR}/src/*.cpp" "${BETTERVR_PACK_DIR}/src/*.h")
    # the check is patch_compiled.asm itself: a stamp per build tree would let a file deleted behind the build stay deleted
    add_custom_command(
        OUTPUT "${BETTERVR_PACK_DIR}/patch_compiled.asm"
        COMMAND ${BETTERVR_PATCH_COMPILER_LAUNCHER} "${BETTERVR_PATCH_COMPILER}" "." "../patch_compiled.asm"
        WORKING_DIRECTORY "${BETTERVR_PACK_DIR}/src"
        DEPENDS ${BETTERVR_PACK_SOURCES} "${BETTERVR_PATCH_COMPILER}"
        COMMENT "Compiling graphic pack patches -> patch_compiled.asm"
        VERBATIM
    )
    add_custom_target(BetterVR_Patches DEPENDS "${BETTERVR_PACK_DIR}/patch_compiled.asm")
else ()
    message(STATUS "CemuPatchCompiler not found in ${BETTERVR_CHECKOUTS_DIR}/PatchCompiler* or ${BETTERVR_PACK_DIR}; using the checked-in patch_compiled.asm")
    add_custom_target(BetterVR_Patches)
endif ()

add_custom_target(BetterVR_ValidateAsm
    COMMAND "${CMAKE_COMMAND}" -DPACK_DIR=${BETTERVR_PACK_DIR} -P "${BETTERVR_ROOT}/cmake/validate_asm_relocs.cmake"
    COMMAND "${CMAKE_COMMAND}" -DPACK_DIR=${BETTERVR_PACK_DIR} -P "${BETTERVR_ROOT}/cmake/validate_asm_labels.cmake"
    COMMAND "${CMAKE_COMMAND}" -DPACK_DIR=${BETTERVR_PACK_DIR} -P "${BETTERVR_ROOT}/cmake/validate_patch_compiled.cmake"
    COMMENT "Checking hand-written asm for broken address materialization and unresolved labels, and patch_compiled.asm for an outdated compiler"
    VERBATIM
)
add_dependencies(BetterVR_ValidateAsm BetterVR_Patches)
add_dependencies(BetterVR BetterVR_ValidateAsm)
set_target_properties(BetterVR BetterVR_Version BetterVR_Patches BetterVR_ValidateAsm PROPERTIES FOLDER "BetterVR")
