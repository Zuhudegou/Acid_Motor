cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MOTOR)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # App scaffolding (new structure, staged migration in progress)
    App/util/motor_util.c
    App/motor_board.c
    App/motor_hal.c
    App/board/board_stm32g431.c
    Lib/Motor/motor.c
    Lib/Motor/motor_control.c
    # LCD driver
    App/Lcd/lcd_drv.c
    App/Lcd/lcd_font.c
    # UI
    App/UI/motor_ui.c
    App/Command/motor_command.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Lib/Motor
    App
    App/board
    App/util
    App/UI
    App/Command
    App/Lcd
    Lib/Motor/Control
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

find_program(OPENOCD_EXECUTABLE openocd)
find_program(BASH_EXECUTABLE bash)
if(OPENOCD_EXECUTABLE)
    add_custom_target(flash-dap
        COMMAND ${OPENOCD_EXECUTABLE}
            -f ${CMAKE_SOURCE_DIR}/tools/openocd_cmsis_dap.cfg
            -c "init"
            -c "halt"
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify"
            -c "reset run"
            -c "shutdown"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with OpenOCD using CMSIS-DAP"
    )

    add_custom_target(flash-openocd
        COMMAND ${OPENOCD_EXECUTABLE}
            -f ${CMAKE_SOURCE_DIR}/tools/openocd_cmsis_dap.cfg
            -c "init"
            -c "halt"
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify"
            -c "reset run"
            -c "shutdown"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with OpenOCD using CMSIS-DAP"
    )

    add_custom_target(flash-dap-hid
        COMMAND ${OPENOCD_EXECUTABLE}
            -f ${CMAKE_SOURCE_DIR}/tools/openocd_cmsis_dap.cfg
            -c "cmsis_dap_backend hid"
            -c "init"
            -c "halt"
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify"
            -c "reset run"
            -c "shutdown"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with OpenOCD using CMSIS-DAP HID backend"
    )

    add_custom_target(flash-dap-bulk
        COMMAND ${OPENOCD_EXECUTABLE}
            -f ${CMAKE_SOURCE_DIR}/tools/openocd_cmsis_dap.cfg
            -c "cmsis_dap_backend usb_bulk"
            -c "init"
            -c "halt"
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify"
            -c "reset run"
            -c "shutdown"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with OpenOCD using CMSIS-DAP USB bulk backend"
    )

    add_custom_target(flash-dap-reset
        COMMAND ${OPENOCD_EXECUTABLE}
            -f ${CMAKE_SOURCE_DIR}/tools/openocd_cmsis_dap_under_reset.cfg
            -c "init"
            -c "reset halt"
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify"
            -c "reset run"
            -c "shutdown"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with OpenOCD using CMSIS-DAP connect-under-reset"
    )

    if(BASH_EXECUTABLE)
        add_custom_target(flash-dap-powercycle
            COMMAND ${BASH_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/tools/flash_dap_powercycle.sh
                $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            DEPENDS ${CMAKE_PROJECT_NAME}
            USES_TERMINAL
            COMMENT "Flash with CMSIS-DAP by repeatedly trying while the target is power-cycled"
        )
    endif()

    add_custom_target(flash-stlink
        COMMAND ${OPENOCD_EXECUTABLE}
            -f ${CMAKE_SOURCE_DIR}/tools/openocd_stlink_under_reset.cfg
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with OpenOCD using ST-Link connect-under-reset"
    )
endif()

find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI
    PATHS /opt/stm32cubeclt/STM32CubeProgrammer/bin
    NO_DEFAULT_PATH
)
find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI)
if(STM32_PROGRAMMER_CLI)
    add_custom_target(flash-stlink-ur
        COMMAND ${STM32_PROGRAMMER_CLI}
            -c port=SWD mode=UR reset=HWrst freq=1000
            -w $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            -v
            -rst
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flash with STM32CubeProgrammer using under-reset mode"
    )
endif()
