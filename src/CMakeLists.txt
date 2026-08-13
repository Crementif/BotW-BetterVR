add_library(BetterVR_Layer SHARED)

file(GENERATE OUTPUT "$<TARGET_FILE_DIR:BetterVR_Layer>/BetterVR_Layer.json" INPUT "${CMAKE_SOURCE_DIR}/resources/BetterVR_Layer.json")
file(GENERATE OUTPUT "$<TARGET_FILE_DIR:BetterVR_Layer>/BetterVR_Layer.direct.json" INPUT "${CMAKE_SOURCE_DIR}/resources/BetterVR_Layer.direct.json")

target_link_options(BetterVR_Layer PRIVATE "$<$<CONFIG:Debug>:/INCREMENTAL>")
target_precompile_headers(BetterVR_Layer PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_include_directories(BetterVR_Layer BEFORE PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(BetterVR_Layer AFTER PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_sources(BetterVR_Layer PUBLIC
    ${CMAKE_SOURCE_DIR}/src/instance.h
    ${CMAKE_SOURCE_DIR}/src/shader.h
    ${CMAKE_SOURCE_DIR}/src/utils/d3d12_utils.h
    ${CMAKE_SOURCE_DIR}/src/utils/vulkan_utils.h
    ${CMAKE_SOURCE_DIR}/src/utils/game_utils.h
    ${CMAKE_SOURCE_DIR}/src/utils/logger.cpp
    ${CMAKE_SOURCE_DIR}/src/utils/logger.h
    ${CMAKE_SOURCE_DIR}/src/utils/controller_bindings.h
    ${CMAKE_SOURCE_DIR}/src/utils/mod_settings.h
    ${CMAKE_SOURCE_DIR}/src/utils/profiler.h
    ${CMAKE_SOURCE_DIR}/src/utils/debug_draw.h
    ${CMAKE_SOURCE_DIR}/src/utils/debug_draw.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/framebuffer.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/framebuffer.h
    ${CMAKE_SOURCE_DIR}/src/hooking/layer.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/layer.h
    ${CMAKE_SOURCE_DIR}/src/hooking/cemu_hooks.h
    ${CMAKE_SOURCE_DIR}/src/hooking/camera.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/game_state.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/settings.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/profiler_hooks.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/weapon.h
    ${CMAKE_SOURCE_DIR}/src/hooking/weapon.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/imgui_menus.h
    ${CMAKE_SOURCE_DIR}/src/hooking/imgui_menus.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/controls.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/entity_debugger.h
    ${CMAKE_SOURCE_DIR}/src/hooking/entity_debugger.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/rumble.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/rumble.h
    ${CMAKE_SOURCE_DIR}/src/hooking/skeleton.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/openxr_motion_bridge.h
    ${CMAKE_SOURCE_DIR}/src/hooking/entity_controller.cpp
    ${CMAKE_SOURCE_DIR}/src/hooking/bow.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/d3d12.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/telemetry.cpp
    ${CMAKE_SOURCE_DIR}/src/utils/ipc_control.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/d3d12.h
    ${CMAKE_SOURCE_DIR}/src/rendering/renderer.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/renderer.h
    ${CMAKE_SOURCE_DIR}/src/rendering/openxr.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/openxr.h
    ${CMAKE_SOURCE_DIR}/src/rendering/swapchain.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/swapchain.h
    ${CMAKE_SOURCE_DIR}/src/rendering/texture.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/texture.h
    ${CMAKE_SOURCE_DIR}/src/rendering/vulkan.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/vulkan.h
    ${CMAKE_SOURCE_DIR}/src/rendering/vulkan_imgui.cpp
)

file(GLOB_RECURSE GRAPHIC_PACK_FILES "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR/*")
if(GRAPHIC_PACK_FILES)
    foreach(GRAPHIC_PACK_FILE ${GRAPHIC_PACK_FILES})
        file(RELATIVE_PATH REL_PATH "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR" "${GRAPHIC_PACK_FILE}")
        string(REPLACE "\\" "/" REL_PATH "${REL_PATH}")
        source_group("BreathOfTheWild_BetterVR" FILES "${GRAPHIC_PACK_FILE}")
        set_source_files_properties("${GRAPHIC_PACK_FILE}" PROPERTIES HEADER_FILE_ONLY TRUE)
        list(APPEND GRAPHIC_PACK_HEADER_FILES "${GRAPHIC_PACK_FILE}")
    endforeach()
    target_sources(BetterVR_Layer PRIVATE ${GRAPHIC_PACK_HEADER_FILES})
endif()

set(BETTERVR_PACK_DIR "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR")

add_custom_target(BetterVR_ValidateAsm ALL
    COMMAND "${CMAKE_COMMAND}" -DPACK_DIR=${BETTERVR_PACK_DIR} -P "${CMAKE_SOURCE_DIR}/cmake/validate_asm_relocs.cmake"
    COMMENT "Checking hand-written asm for broken address materialization"
    VERBATIM
)
set_target_properties(BetterVR_ValidateAsm PROPERTIES FOLDER "BreathOfTheWild_BetterVR")
add_dependencies(BetterVR_Layer BetterVR_ValidateAsm)

target_link_libraries(BetterVR_Layer PRIVATE OpenXR::headers OpenXR::openxr_loader)
target_include_directories(BetterVR_Layer SYSTEM PRIVATE ${VULKAN_HEADERS_INCLUDE_DIRS})
target_link_libraries(BetterVR_Layer PRIVATE glm::glm)
target_link_libraries(BetterVR_Layer PRIVATE imgui::imgui)
target_link_libraries(BetterVR_Layer PRIVATE implot::implot)

target_compile_definitions(BetterVR_Layer PRIVATE IMGUI_IMPL_VULKAN_NO_PROTOTYPES)
target_sources(BetterVR_Layer PRIVATE ${CMAKE_SOURCE_DIR}/dependencies/imgui_impl_vulkan.cpp)
target_include_directories(BetterVR_Layer PRIVATE ${CMAKE_SOURCE_DIR}/dependencies)

# Install/Debugger related stuff. This is really only used for direct calls
set(BETTERVR_GRAPHIC_PACK_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/graphicPacks/BreathOfTheWild_BetterVR")

if(CMAKE_INSTALL_PREFIX STREQUAL "${CMAKE_SOURCE_DIR}/Cemu")
    install(CODE [[
        file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/graphicPacks/BreathOfTheWild_BetterVR")
    ]])
endif()

install(TARGETS BetterVR_Layer
    RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}"
)
install(FILES "$<TARGET_FILE_DIR:BetterVR_Layer>/BetterVR_Layer.direct.json" DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES $<TARGET_PDB_FILE:BetterVR_Layer> DESTINATION "${CMAKE_INSTALL_PREFIX}" CONFIGURATIONS Debug RelWithDebInfo OPTIONAL)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR/"
    DESTINATION "${BETTERVR_GRAPHIC_PACK_INSTALL_DIR}"
    CONFIGURATIONS Debug
)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR/"
    DESTINATION "${BETTERVR_GRAPHIC_PACK_INSTALL_DIR}"
    CONFIGURATIONS Release RelWithDebInfo MinSizeRel
    PATTERN "patch_debug_*.asm" EXCLUDE
)

set_target_properties(BetterVR_Layer PROPERTIES
    VS_DEBUGGER_COMMAND "${CMAKE_INSTALL_PREFIX}/Cemu.exe"
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}"
    VS_DEBUGGER_ENVIRONMENT "ENABLE_BETTERVR_MOD=1"
)
