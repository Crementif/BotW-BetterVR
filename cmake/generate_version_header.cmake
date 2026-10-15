if(NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_HEADER OR NOT DEFINED VERSION)
    message(FATAL_ERROR "Missing required version generator arguments")
endif()

if(NOT DEFINED CI_BUILD)
    set(CI_BUILD OFF)
endif()

get_filename_component(output_dir "${OUTPUT_HEADER}" DIRECTORY)
file(MAKE_DIRECTORY "${output_dir}")

function(write_if_different path contents)
    if(EXISTS "${path}")
        file(READ "${path}" existing_contents)
        if(existing_contents STREQUAL contents)
            return()
        endif()
    endif()
    file(WRITE "${path}" "${contents}")
endfunction()

set(commit "")
set(dirty OFF)

find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=8 HEAD
        WORKING_DIRECTORY "${SOURCE_DIR}"
        OUTPUT_VARIABLE commit
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE rev_parse_result
    )
    if(NOT rev_parse_result EQUAL 0)
        set(commit "")
    endif()

    if(NOT commit STREQUAL "")
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
            WORKING_DIRECTORY "${SOURCE_DIR}"
            OUTPUT_VARIABLE working_tree_status
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
            RESULT_VARIABLE status_result
        )
        if(status_result EQUAL 0 AND NOT working_tree_status STREQUAL "")
            set(dirty ON)
        endif()
    endif()
endif()

if(CI_BUILD)
    set(ci_flag 1)
    set(version_string "${VERSION}")
elseif(commit STREQUAL "")
    set(ci_flag 0)
    set(version_string "${VERSION}+unknown")
elseif(dirty)
    set(ci_flag 0)
    set(version_string "${VERSION}+${commit}-dirty")
else()
    set(ci_flag 0)
    set(version_string "${VERSION}+${commit}")
endif()

set(header_contents "#pragma once\n\n")
string(APPEND header_contents "#define BETTERVR_VERSION \"${VERSION}\"\n")
string(APPEND header_contents "#define BETTERVR_COMMIT \"${commit}\"\n")
string(APPEND header_contents "#define BETTERVR_CI_BUILD ${ci_flag}\n")
string(APPEND header_contents "#define BETTERVR_VERSION_STRING \"${version_string}\"\n")

write_if_different("${OUTPUT_HEADER}" "${header_contents}")
