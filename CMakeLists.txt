set(VCPKG_TARGET_TRIPLET x64-windows-static-md)

cmake_minimum_required(VERSION 3.7.0)
project(BetterVR_Layer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check architecture support
if (NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Only x64 architecture is supported")
endif ()

# Set global options
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Set output/install directories
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/Cemu")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Set compilation options
add_compile_definitions(NOMINMAX)
add_compile_options("/await")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(VCPKG_TARGET_TRIPLET x64-windows-static-md)
set(VCPKG_HOST_TRIPLET x64-windows-static-md)

# Find vcpkg dependencies
find_path(VULKAN_HEADERS_INCLUDE_DIRS "vk_video/vulkan_video_codec_h264std.h")
find_package(OpenXR CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
#find_package(Eigen3 CONFIG REQUIRED)


# Add DLL target
add_library(BetterVR_Layer SHARED)

# Add (precompiled) headers for DLL
target_precompile_headers(BetterVR_Layer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/pch.h)
target_include_directories(BetterVR_Layer BEFORE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(BetterVR_Layer AFTER PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add source for DLL
target_sources(BetterVR_Layer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/instance.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/d3d12_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/logger.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/framebuffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/framebuffer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/layer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/layer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/cemu_hooks.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/settings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/d3d12.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/d3d12.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/renderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/openxr.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/openxr.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/swapchain.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/swapchain.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/texture.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/texture.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/vulkan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/vulkan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hooking/hands.cpp
)

# Add dependencies for DLL
target_link_libraries(BetterVR_Layer PRIVATE OpenXR::headers OpenXR::openxr_loader)
target_include_directories(BetterVR_Layer SYSTEM PRIVATE ${VULKAN_HEADERS_INCLUDE_DIRS})
target_link_libraries(BetterVR_Layer PRIVATE glm::glm)
#target_link_libraries(BetterVR_Layer PRIVATE Eigen3::Eigen)

# Set install rules
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/BetterVR_Layer.json" "${CMAKE_CURRENT_SOURCE_DIR}/resources/Launch_BetterVR.bat" DESTINATION "${CMAKE_INSTALL_PREFIX}")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources/BreathOfTheWild_BetterVR" DESTINATION "${CMAKE_INSTALL_PREFIX}/graphicPacks")
install(TARGETS BetterVR_Layer DESTINATION "${CMAKE_INSTALL_PREFIX}")