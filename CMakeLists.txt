cmake_minimum_required(VERSION 3.10)
project(mid360_driver)

## use c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable color output
set(CMAKE_COLOR_DIAGNOSTICS ON)

## export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# enable compile warning
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
elseif(MSVC)
    add_compile_options("/W4")
    add_compile_options("/wd4100")
endif()

# fix issues in MSVC
if(MSVC)
    add_compile_options("/utf-8")
endif()

# optimize performance
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-march=native")
    add_compile_options("-ffast-math")
    add_compile_options("-fno-math-errno")
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

file(GLOB_RECURSE SOURCE src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE})
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN mid360_driver::Mid360DriverNode
    EXECUTABLE ${PROJECT_NAME}_node
)
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN mid70_driver::Mid70DriverNode
    EXECUTABLE mid70_driver_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
    launch
)
