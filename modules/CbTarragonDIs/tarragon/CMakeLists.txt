set(PLUGLOCK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

target_sources(${MODULE_NAME}
    PRIVATE
        "CbTarragonDIPWM.cpp"
)

target_link_libraries(${MODULE_NAME}
    PRIVATE
        utils
)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

target_include_directories(${MODULE_NAME} PRIVATE ${GENERATED_INCLUDE_DIR})

add_dependencies(${MODULE_NAME} generate_cpp_files)
