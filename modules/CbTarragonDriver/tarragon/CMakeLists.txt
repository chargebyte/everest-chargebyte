set(TARRAGON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(tarragon STATIC)

target_sources(tarragon
    PRIVATE
        "CbTarragonPWM.cpp"
        "CbTarragonCP.cpp"
        "CbTarragonCPADC.cpp"
        "CbTarragonPP.cpp"
        "CbTarragonRCM.cpp"
        "CbTarragonRelay.cpp"
        "CbTarragonContactorControl.cpp"
)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

target_include_directories(tarragon PRIVATE ${UTILS_INCLUDE_DIR} ${GENERATED_INCLUDE_DIR})

add_dependencies(tarragon generate_cpp_files)

target_link_libraries(tarragon PRIVATE utils everest::framework everest::log)
