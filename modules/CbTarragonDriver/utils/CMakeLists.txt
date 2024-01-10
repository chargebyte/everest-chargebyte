set(UTILS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(utils STATIC)

target_sources(utils
    PRIVATE
        "gpiodUtils.cpp"
        "IIOADC.cpp"
        "IIOADCUtils.cpp"
        "PWM.cpp"
        "PWMChip.cpp"
        "SysfsDevice.cpp"
)

target_link_libraries(utils PRIVATE ${LIBGPIOD_LIBRARIES})
