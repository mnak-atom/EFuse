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
set(CMAKE_PROJECT_NAME EFuse_Lo_alt)

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

# Collect all C source files in src/
file(GLOB SRC_FILES CONFIGURE_DEPENDS
    "Core/Src/*.c"
    "Core/Src/task_src/*.c"
)

# List of duplicate source file names to exclude (already included in CubeMX subdir)
set(DUPLICATE_SOURCE_NAMES
    system_stm32l0xx.c
    
)

# Auto-remove duplicates by filename
foreach(src_file ${SRC_FILES})
    get_filename_component(file_name ${src_file} NAME)
    list(FIND DUPLICATE_SOURCE_NAMES ${file_name} is_duplicate)
    if(NOT is_duplicate EQUAL -1)
        list(REMOVE_ITEM SRC_FILES ${src_file})
    endif()
endforeach()

# Collect all header files in inc/ (optional)
file(GLOB HEADER_FILES CONFIGURE_DEPENDS
    "Core/Inc/*.h"
    "Core/Inc/task_inc/*.h"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SRC_FILES}
    # Add user sources here
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    Core/Inc/task_inc
    Drivers/STM32L0xx_HAL_Driver/Inc
    Drivers/CMSIS/Device/ST/STM32L0xx/Include
    Drivers/CMSIS/Include
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
