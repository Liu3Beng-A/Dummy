# One authoritative output directory for both production firmware projects.
include_guard(GLOBAL)
get_filename_component(FIRMWARE_ARTIFACT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../firmware_artifacts" ABSOLUTE)
file(MAKE_DIRECTORY "${FIRMWARE_ARTIFACT_DIR}")

function(firmware_export_artifacts target)
    if(NOT TARGET "${target}")
        message(FATAL_ERROR "Unknown firmware target: ${target}")
    endif()
    string(REGEX REPLACE "\\.elf$" "" _stem "${target}")
    set_target_properties("${target}" PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${FIRMWARE_ARTIFACT_DIR}")
    set(_hex "${FIRMWARE_ARTIFACT_DIR}/${_stem}.hex")
    set(_bin "${FIRMWARE_ARTIFACT_DIR}/${_stem}.bin")
    set(_map "${FIRMWARE_ARTIFACT_DIR}/${_stem}.map")
    set(_manifest "${FIRMWARE_ARTIFACT_DIR}/${_stem}.manifest.txt")
    target_link_options("${target}" PRIVATE "-Wl,-Map=${_map}")
    add_custom_command(TARGET "${target}" POST_BUILD
            BYPRODUCTS "${_hex}" "${_bin}" "${_map}" "${_manifest}"
            COMMAND "${CMAKE_OBJCOPY}" -Oihex "$<TARGET_FILE:${target}>" "${_hex}"
            COMMAND "${CMAKE_OBJCOPY}" -Obinary "$<TARGET_FILE:${target}>" "${_bin}"
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${CMAKE_BINARY_DIR}/generated/fw_build_manifest.txt" "${_manifest}"
            COMMENT "Exporting ${_stem} ELF/MAP/HEX/BIN/manifest to ${FIRMWARE_ARTIFACT_DIR}"
            VERBATIM)
endfunction()
