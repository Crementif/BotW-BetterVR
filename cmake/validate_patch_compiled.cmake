if(NOT DEFINED PACK_DIR)
    message(FATAL_ERROR "validate_patch_compiled.cmake requires -DPACK_DIR=<dir>")
endif()

# CemuPatchCompiler builds from before its v1.1.0 string fix emit data as .string lines, which Cemu cuts at the first
# '#' byte and then rejects the whole pack. Those builds report the same version, so their output is the only tell.
set(VAR_FILE "${PACK_DIR}/patch_compiled.asm")
if(NOT EXISTS "${VAR_FILE}")
    message(FATAL_ERROR "${VAR_FILE} is missing")
endif()

file(STRINGS "${VAR_FILE}" VAR_STRING_LINES REGEX "^[ \t]*\\.(string|ascii|asciz)[ \t]")
if(VAR_STRING_LINES)
    message(FATAL_ERROR "patch_compiled.asm has string directives, so it was compiled by an outdated CemuPatchCompiler. Replace ${PACK_DIR}/CemuPatchCompiler-release.exe with a current build and delete patch_compiled.asm to regenerate it.")
endif()
