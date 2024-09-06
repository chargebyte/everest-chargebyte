set(CHARGESOM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(chargesom STATIC)

target_sources(chargesom
    PRIVATE
        "CbChargeSOM.cpp"
)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

target_include_directories(chargesom PRIVATE ${LIBCBUART_INCLUDE_DIR} ${GENERATED_INCLUDE_DIR})

add_dependencies(chargesom generate_cpp_files)

target_link_libraries(chargesom PRIVATE libcbuart Pal::Sigslot)
