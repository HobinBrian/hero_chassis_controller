cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    pluginlib controller_interface hardware_interface
    roslint
    control_toolbox
    )

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    control_toolbox
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare the controller shared library
add_library(${PROJECT_NAME}
    src/hero_chassis_controller.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

install(
    DIRECTORY doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########################
## Static code analysis ##
##########################

roslint_cpp()
