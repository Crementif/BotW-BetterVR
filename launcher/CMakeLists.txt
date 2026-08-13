set(BETTERVR_LAUNCHER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/launcher_common.h
    ${CMAKE_CURRENT_SOURCE_DIR}/openxr_checker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/update_checker.cpp
)

set(BETTERVR_LAUNCHER_EMBED_DIR "${CMAKE_BINARY_DIR}/generated/launcher")
set(BETTERVR_LAUNCHER_EMBED_HEADER "${BETTERVR_LAUNCHER_EMBED_DIR}/embedded_assets.h")
set(BETTERVR_LAUNCHER_EMBED_SOURCE "${BETTERVR_LAUNCHER_EMBED_DIR}/embedded_assets.cpp")
set(BETTERVR_LAUNCHER_EMBED_RC "${BETTERVR_LAUNCHER_EMBED_DIR}/embedded_assets.rc")
set(BETTERVR_LAUNCHER_EMBED_STAMP "${BETTERVR_LAUNCHER_EMBED_DIR}/embedded_assets.stamp")
set(BETTERVR_LAUNCHER_EMBED_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/generate_embedded_assets.cmake")
set(BETTERVR_GRAPHIC_PACK_DIR "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR")
set(BETTERVR_BOTW_GRAPHICS_DIR "${CMAKE_SOURCE_DIR}/resources/BreathOfTheWild_Graphics")

file(GLOB_RECURSE GRAPHIC_PACK_FILES CONFIGURE_DEPENDS "${BETTERVR_GRAPHIC_PACK_DIR}/*")
file(GLOB_RECURSE BOTW_GRAPHICS_FILES CONFIGURE_DEPENDS "${BETTERVR_BOTW_GRAPHICS_DIR}/*")

# Adding or removing an asset changes no surviving file's timestamp, so the list
# itself is folded into the command line to keep the generator from going stale.
string(MD5 BETTERVR_LAUNCHER_ASSET_LIST_HASH "${GRAPHIC_PACK_FILES};${BOTW_GRAPHICS_FILES}")

add_executable(BetterVR_Launcher ${BETTERVR_LAUNCHER_SOURCES})
target_precompile_headers(BetterVR_Launcher PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_include_directories(BetterVR_Launcher BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(BetterVR_Launcher BEFORE PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(BetterVR_Launcher AFTER PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(BetterVR_Launcher AFTER PRIVATE ${CMAKE_SOURCE_DIR}/dependencies)
target_include_directories(BetterVR_Launcher SYSTEM PRIVATE ${VULKAN_HEADERS_INCLUDE_DIRS})
target_link_libraries(BetterVR_Launcher PRIVATE OpenXR::headers OpenXR::openxr_loader ktmw32)
target_compile_definitions(BetterVR_Launcher PRIVATE BETTERVR_LAUNCHER_VERSION="${PROJECT_VERSION}")
target_link_options(BetterVR_Launcher PRIVATE /SUBSYSTEM:WINDOWS /ENTRY:wmainCRTStartup)

add_custom_command(
    OUTPUT ${BETTERVR_LAUNCHER_EMBED_STAMP}
    BYPRODUCTS ${BETTERVR_LAUNCHER_EMBED_HEADER} ${BETTERVR_LAUNCHER_EMBED_SOURCE} ${BETTERVR_LAUNCHER_EMBED_RC}
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT_HEADER=${BETTERVR_LAUNCHER_EMBED_HEADER}
        -DOUTPUT_SOURCE=${BETTERVR_LAUNCHER_EMBED_SOURCE}
        -DOUTPUT_RC=${BETTERVR_LAUNCHER_EMBED_RC}
        -DINPUT_DLL=$<TARGET_FILE:BetterVR_Layer>
        -DINPUT_JSON=$<TARGET_FILE_DIR:BetterVR_Layer>/BetterVR_Layer.json
        -DGRAPHIC_PACK_DIR=${BETTERVR_GRAPHIC_PACK_DIR}
        -DBOTW_GRAPHICS_DIR=${BETTERVR_BOTW_GRAPHICS_DIR}
        -DEXCLUDE_DEBUG_PATCHES=$<IF:$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>,ON,OFF>
        -DVERSION=${PROJECT_VERSION}
        -DASSET_LIST_HASH=${BETTERVR_LAUNCHER_ASSET_LIST_HASH}
        -P ${BETTERVR_LAUNCHER_EMBED_SCRIPT}
    COMMAND ${CMAKE_COMMAND} -E touch ${BETTERVR_LAUNCHER_EMBED_STAMP}
    DEPENDS
        BetterVR_Layer
        $<TARGET_FILE:BetterVR_Layer>
        ${CMAKE_SOURCE_DIR}/resources/BetterVR_Layer.json
        ${BETTERVR_LAUNCHER_EMBED_SCRIPT}
        ${GRAPHIC_PACK_FILES}
        ${BOTW_GRAPHICS_FILES}
    VERBATIM
)

add_custom_target(BetterVR_LauncherAssets DEPENDS ${BETTERVR_LAUNCHER_EMBED_STAMP})
add_dependencies(BetterVR_Launcher BetterVR_LauncherAssets)

# The .rc only names the embedded files by path, so nothing in its own contents
# changes when an asset's bytes change. Ninja cannot see that dependency, so the
# stamp is what forces rc.exe to re-read the assets.
set_source_files_properties(${BETTERVR_LAUNCHER_EMBED_RC} PROPERTIES OBJECT_DEPENDS ${BETTERVR_LAUNCHER_EMBED_STAMP})
set_property(TARGET BetterVR_Launcher APPEND PROPERTY LINK_DEPENDS ${BETTERVR_LAUNCHER_EMBED_STAMP})

target_sources(BetterVR_Launcher PRIVATE ${BETTERVR_LAUNCHER_EMBED_SOURCE} ${BETTERVR_LAUNCHER_EMBED_HEADER} ${BETTERVR_LAUNCHER_EMBED_RC})
target_include_directories(BetterVR_Launcher PRIVATE ${BETTERVR_LAUNCHER_EMBED_DIR})

install(TARGETS BetterVR_Launcher DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(FILES $<TARGET_PDB_FILE:BetterVR_Launcher> DESTINATION "${CMAKE_INSTALL_PREFIX}" CONFIGURATIONS Debug RelWithDebInfo OPTIONAL)
