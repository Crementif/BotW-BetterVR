// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <openxr/openxr.h>
#include <openxr/openxr_reflection.h>

#include <string>

// Macro to generate stringify functions for OpenXR enumerations based data provided in openxr_reflection.h
// clang-format off
#define ENUM_CASE_STR(name, val) case name: return #name;

// Returns C string pointing to a string literal. Unknown values are returned as 'Unknown <type>'.
#define MAKE_TO_CSTRING_FUNC(enumType)                      \
    constexpr const char* ToCString(enumType e) noexcept {  \
        switch (e) {                                        \
            XR_LIST_ENUM_##enumType(ENUM_CASE_STR)          \
            default: return "Unknown " #enumType;           \
        }                                                   \
    }

// Returns a STL string. Unknown values are stringified as an integer.
#define MAKE_TO_STRING_FUNC(enumType)                  \
    inline std::string ToString(enumType e) {          \
        switch (e) {                                   \
            XR_LIST_ENUM_##enumType(ENUM_CASE_STR)     \
            default: return std::to_string(e);         \
        }                                              \
    }

#define MAKE_TO_STRING_FUNCS(enumType) \
    MAKE_TO_CSTRING_FUNC(enumType) \
    MAKE_TO_STRING_FUNC(enumType)
// clang-format on

namespace xr {
    MAKE_TO_STRING_FUNCS(XrReferenceSpaceType);
    MAKE_TO_STRING_FUNCS(XrViewConfigurationType);
    MAKE_TO_STRING_FUNCS(XrEnvironmentBlendMode);
    MAKE_TO_STRING_FUNCS(XrSessionState);
    MAKE_TO_STRING_FUNCS(XrResult);
    MAKE_TO_STRING_FUNCS(XrStructureType);
    MAKE_TO_STRING_FUNCS(XrFormFactor);
    MAKE_TO_STRING_FUNCS(XrEyeVisibility);
    MAKE_TO_STRING_FUNCS(XrObjectType);
    MAKE_TO_STRING_FUNCS(XrActionType);
    MAKE_TO_STRING_FUNCS(XrVisibilityMaskTypeKHR);
} // namespace xr

