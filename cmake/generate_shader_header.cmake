# Emits a header with one constexpr uint32_t array per SPIR-V module.
# Invoked as: cmake -DSPIRV_FILES=<;-list> -DOUTPUT=<header> -P generate_shader_header.cmake

if(NOT DEFINED SPIRV_FILES OR NOT DEFINED OUTPUT)
    message(FATAL_ERROR "generate_shader_header.cmake needs SPIRV_FILES and OUTPUT")
endif()

set(CONTENT "#pragma once\n\n#include <cstdint>\n\n// Generated from resources/shaders by generate_shader_header.cmake, do not edit.\n\n")

foreach(SPIRV_FILE ${SPIRV_FILES})
    if(NOT EXISTS "${SPIRV_FILE}")
        message(FATAL_ERROR "Missing SPIR-V module ${SPIRV_FILE}; install the Vulkan SDK (glslc) or restore the prebuilt binaries")
    endif()

    get_filename_component(SPIRV_NAME "${SPIRV_FILE}" NAME)
    string(REPLACE "." "_" SYMBOL_NAME "${SPIRV_NAME}")

    file(READ "${SPIRV_FILE}" HEX_DATA HEX)
    string(LENGTH "${HEX_DATA}" HEX_LENGTH)
    math(EXPR WORD_COUNT "${HEX_LENGTH} / 8")
    math(EXPR REMAINDER "${HEX_LENGTH} % 8")
    if(NOT REMAINDER EQUAL 0 OR WORD_COUNT EQUAL 0)
        message(FATAL_ERROR "${SPIRV_FILE} is not a whole number of SPIR-V words")
    endif()

    set(WORDS "")
    set(WORD_INDEX 0)
    while(WORD_INDEX LESS WORD_COUNT)
        math(EXPR BYTE_OFFSET "${WORD_INDEX} * 8")
        string(SUBSTRING "${HEX_DATA}" ${BYTE_OFFSET} 8 WORD_HEX)
        string(SUBSTRING "${WORD_HEX}" 0 2 BYTE0)
        string(SUBSTRING "${WORD_HEX}" 2 2 BYTE1)
        string(SUBSTRING "${WORD_HEX}" 4 2 BYTE2)
        string(SUBSTRING "${WORD_HEX}" 6 2 BYTE3)

        math(EXPR LINE_POSITION "${WORD_INDEX} % 8")
        if(LINE_POSITION EQUAL 0)
            string(APPEND WORDS "\n    ")
        endif()
        string(APPEND WORDS "0x${BYTE3}${BYTE2}${BYTE1}${BYTE0}, ")

        math(EXPR WORD_INDEX "${WORD_INDEX} + 1")
    endwhile()

    string(APPEND CONTENT "inline constexpr uint32_t ${SYMBOL_NAME}[] = {${WORDS}\n};\n\n")
endforeach()

get_filename_component(OUTPUT_DIR "${OUTPUT}" DIRECTORY)
file(MAKE_DIRECTORY "${OUTPUT_DIR}")
file(WRITE "${OUTPUT}" "${CONTENT}")
