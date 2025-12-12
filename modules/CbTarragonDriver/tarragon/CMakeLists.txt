set(TARRAGON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(tarragon STATIC)

target_sources(tarragon
    PRIVATE
        "CbActuator.cpp"
        "CbTarragonPWM.cpp"
        "CbTarragonCP.cpp"
        "CbTarragonCPADC.cpp"
        "CbTarragonPP.cpp"
        "CbTarragonRCM.cpp"
        "CbTarragonRelay.cpp"
        "CbTarragonContactor.cpp"
        "CbTarragonContactorControl.cpp"
        "CbTarragonContactorControlSimple.cpp"
        "CbTarragonContactorControlSerial.cpp"
        "CbTarragonContactorControlSimultaneous.cpp"
        "CbTarragonContactorControlMutual.cpp"
)

get_property(LINUX_HW_INCLUDE_DIR GLOBAL PROPERTY LINUX_HW_INCLUDE_DIR)
get_property(CP_INCLUDE_DIR GLOBAL PROPERTY CP_INCLUDE_DIR)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

target_include_directories(tarragon
    PRIVATE 
        ${LINUX_HW_INCLUDE_DIR}
        ${CP_INCLUDE_DIR}
        ${GENERATED_INCLUDE_DIR}
    )

add_dependencies(tarragon generate_cpp_files)

target_link_libraries(tarragon
    PRIVATE
        utils::linux_hw
        everest::framework
        everest::log
)
