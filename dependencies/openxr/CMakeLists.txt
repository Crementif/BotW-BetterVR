# The OpenXR loader from the OpenXR-SDK 1.1.45 release, static, with its bundled jsoncpp. The SDK generates the
# headers and the xr_generated_* sources from the registry with Python; they are checked in as generated.
add_library(BetterVR_OpenXR STATIC
    src/loader/api_layer_interface.cpp
    src/loader/loader_core.cpp
    src/loader/loader_init_data.cpp
    src/loader/loader_instance.cpp
    src/loader/loader_logger.cpp
    src/loader/loader_logger_recorders.cpp
    src/loader/manifest_file.cpp
    src/loader/runtime_interface.cpp
    src/loader/xr_generated_loader.cpp
    src/xr_generated_dispatch_table_core.c
    src/common/filesystem_utils.cpp
    src/common/object_info.cpp
    src/external/jsoncpp/src/lib_json/json_reader.cpp
    src/external/jsoncpp/src/lib_json/json_value.cpp
    src/external/jsoncpp/src/lib_json/json_writer.cpp
)
set_target_properties(BetterVR_OpenXR PROPERTIES FOLDER "dependencies" C_VISIBILITY_PRESET hidden CXX_VISIBILITY_PRESET hidden)

include(CheckFunctionExists)
check_function_exists(secure_getenv HAVE_SECURE_GETENV)
check_function_exists(__secure_getenv HAVE___SECURE_GETENV)
configure_file(src/common_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/generated/common_config.h")

target_include_directories(BetterVR_OpenXR PUBLIC include)
target_include_directories(BetterVR_OpenXR PRIVATE src src/common src/loader src/external/jsoncpp/include "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_compile_definitions(BetterVR_OpenXR PRIVATE OPENXR_HAVE_COMMON_CONFIG XRLOADER_DISABLE_EXCEPTION_HANDLING JSON_USE_EXCEPTION=0 XR_USE_TIMESPEC)
if (WIN32)
    target_compile_definitions(BetterVR_OpenXR PRIVATE XR_OS_WINDOWS XR_USE_PLATFORM_WIN32 NOMINMAX)
    target_link_libraries(BetterVR_OpenXR PUBLIC advapi32)
else ()
    target_compile_definitions(BetterVR_OpenXR PRIVATE XR_OS_LINUX FALLBACK_CONFIG_DIRS="/etc/xdg" FALLBACK_DATA_DIRS="/usr/local/share:/usr/share" SYSCONFDIR="/etc")
    find_package(Threads REQUIRED)
    target_link_libraries(BetterVR_OpenXR PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
endif ()

add_library(OpenXR::openxr_loader ALIAS BetterVR_OpenXR)
