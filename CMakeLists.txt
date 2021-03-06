cmake_minimum_required(VERSION 2.8.3)
project(ros_end_effector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS srdfdom 
                                        kdl_parser
                                        roscpp
                                        message_generation 
                                        )
                                                                               
# find_package(matlogger2 REQUIRED)

find_package(Eigen3 REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
DIRECTORY msgs
FILES
EEGraspControl.msg
)

add_message_files(
DIRECTORY msgs
FILES
EEPinchControl.msg
)

generate_messages(
DEPENDENCIES
std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_end_effector
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ROSEEParser
    src/Parser.cpp
)

add_library(ROSEEInterface
    src/EEInterface.cpp
)

add_library(DummyHal
    src/EEHal.cpp
    src/DummyHal.cpp
)

add_library(UniversalRosEndEffectorExecutor
    src/UniversalRosEndEffectorExecutor.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(UniversalRosEndEffector src/UniversalRosEndEffector.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ROSEEParser
    ${catkin_LIBRARIES}
    yaml-cpp
)

target_link_libraries(ROSEEInterface
    ${catkin_LIBRARIES}
    ROSEEParser
)

target_link_libraries(DummyHal
    yaml-cpp
)

target_link_libraries(UniversalRosEndEffectorExecutor
    ${catkin_LIBRARIES}
    ROSEEParser
    ROSEEInterface
    
    # do it automatically
    DummyHal
    
)

target_link_libraries(UniversalRosEndEffector
    UniversalRosEndEffectorExecutor
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(EEInterface-test test/test_ee_interface.cpp)
    if(TARGET EEInterface-test)
        target_link_libraries(EEInterface-test ${catkin_LIBRARIES}
                                                ROSEEParser
                                                ROSEEInterface)
    endif()

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## doxygen documentation

#A macro to allow clean, readable inclusion of subdirectories
macro(optional_build name path ONOFF)
  option(BUILD_${name} ${ONOFF})
  if( BUILD_${name} )
    #We this need to make include files available examples... a bit brute force
    #include_directories("${PROJECT_SOURCE_DIR}/${path}")
    add_subdirectory(${path})
  endif()
endmacro(optional_build name path ONOFF)

optional_build(documentation doc ON)
