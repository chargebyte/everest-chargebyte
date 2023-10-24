set(TARRAGON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(tarragon STATIC)

target_sources(tarragon
    PRIVATE
        "CbTarragonPWM.cpp"
        "CbTarragonCP.cpp"
        "CbTarragonCPADC.cpp"
        "CbTarragonPP.cpp"
        "CbTarragonRelay.cpp"
)

get_target_property(EVEREST_GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

target_include_directories(tarragon PRIVATE ${UTILS_INCLUDE_DIR} ${EVEREST_GENERATED_INCLUDE_DIR})
target_link_libraries(tarragon PRIVATE utils everest::framework)
