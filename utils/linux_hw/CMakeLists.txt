set_property(GLOBAL PROPERTY LINUX_HW_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(linux_hw STATIC)
add_library(utils::linux_hw ALIAS linux_hw)

target_sources(linux_hw
    PRIVATE
        "gpiodUtils.cpp"
        "IIOADC.cpp"
        "IIOADCUtils.cpp"
        "PWM.cpp"
        "PWMChip.cpp"
        "SysfsDevice.cpp"
)

target_link_libraries(linux_hw PRIVATE ${LIBGPIOD_LIBRARIES})

target_include_directories(linux_hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

