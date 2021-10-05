
cmake_minimum_required(VERSION 2.8.3)
project(end_effector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

##test code coverage
option(ROSEE_ENABLE_COVERAGE "Compile with coverage information" ON)
if(${ROSEE_ENABLE_COVERAGE})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -g")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS srdfdom 
                                        kdl_parser
                                        roscpp
                                        rospy
                                        message_generation 
                                        moveit_ros_planning_interface
                                        rosee_msg
                                        )
                                                                               
find_package(Eigen3 REQUIRED)
#TODO on my pc, find_package works. On iit pc, I need the find_library. Why?
#find_package(muparser REQUIRED) 
find_library(_MUPARSER_LIB muparser REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(matlogger2 QUIET)
if (${matlogger2_FOUND})
    message("Logging with MatLogger2")
    add_definitions(-D_MATLOGGER2)
endif (${matlogger2_FOUND})

#Xbot stuff
option(BUILD_WITH_XBOT2 "Compile also the HAL used with XBot2" OFF)

if (BUILD_WITH_XBOT2)
    message("Compiling with XBOT2")
    option(XBOT2_ENABLE_XENO OFF "Compile against xenomai") #TODO necessary?
    find_package(xbot2 REQUIRED)

else()
    message("Compiling without XBOT2")
endif(BUILD_WITH_XBOT2)

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
    INCLUDE_DIRS include
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
     src/HAL/DummyHal.cpp
)

add_library(EEHal
     src/HAL/EEHal.cpp
)

if (BUILD_WITH_XBOT2)

    add_library(XBot2Hal
        src/HAL/XBot2Hal.cpp
    )

endif(BUILD_WITH_XBOT2)


add_library(UniversalRosEndEffectorExecutor
    src/UniversalRosEndEffectorExecutor.cpp
)

add_library(FindActions 
    src/FindActions.cpp
)

add_library(ParserMoveIt
    src/ParserMoveIt.cpp
)

add_library(YamlWorker
    src/YamlWorker.cpp
)

add_library(RosActionServer
    src/RosActionServer.cpp
)

add_library(RosServiceHandler
    src/RosServiceHandler.cpp
)

add_library(MapActionHandler
    src/MapActionHandler.cpp
)

add_library(Action 
    src/GraspingActions/Action.cpp 
)

add_library(ActionPrimitive 
    src/GraspingActions/ActionPrimitive.cpp
)

add_library(ActionPinchTight
    src/GraspingActions/ActionPinchTight.cpp
)
add_library(ActionPinchGeneric
    src/GraspingActions/ActionPinchGeneric.cpp
)
add_library(ActionPinchLoose
    src/GraspingActions/ActionPinchLoose.cpp
)
add_library(ActionTrig
    src/GraspingActions/ActionTrig.cpp
)
add_library(ActionSingleJointMultipleTips
    src/GraspingActions/ActionSingleJointMultipleTips.cpp
)
add_library(ActionMultiplePinchTight
    src/GraspingActions/ActionMultiplePinchTight.cpp
)
add_library(ActionGeneric
    src/GraspingActions/ActionGeneric.cpp 
)
add_library(ActionComposed
    src/GraspingActions/ActionComposed.cpp 
)
add_library(ActionTimed
    src/GraspingActions/ActionTimed.cpp 
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(UniversalRosEndEffector src/UniversalRosEndEffector.cpp)
add_executable(UniversalFindActions src/UniversalFindActions.cpp)
add_executable(EEHalExecutor src/HAL/EEHalExecutor.cpp)
# # add_dependencies(UniversalRosEndEffectorExecutor ros_end_effector_generate_messages_cpp)
# # add_dependencies(UniversalFindActions ros_end_effector_generate_messages_cpp)


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

target_link_libraries(UniversalRosEndEffectorExecutor
    ${catkin_LIBRARIES}
    ROSEEParser
    ROSEEInterface
    YamlWorker
    MapActionHandler
    RosActionServer
    RosServiceHandler
    
)

target_link_libraries(YamlWorker
    yaml-cpp
    Action
    ActionPrimitive
    ActionPinchTight
    ActionTrig
    ActionPinchLoose
    ActionSingleJointMultipleTips
    ActionMultiplePinchTight
    ActionComposed
    ActionTimed
)

target_link_libraries(MapActionHandler
    Action
    ActionPrimitive
    ActionGeneric
    ActionTimed
    YamlWorker
)

target_link_libraries(RosActionServer
    ${catkin_LIBRARIES}
)
add_dependencies(RosActionServer rosee_msg_generate_messages_cpp)

target_link_libraries(RosServiceHandler
    ${catkin_LIBRARIES}
)
add_dependencies(RosServiceHandler rosee_msg_generate_messages_cpp)

target_link_libraries(ActionPinchTight
    ActionPinchGeneric
)

target_link_libraries(ActionPinchGeneric
    ActionPrimitive
)

target_link_libraries(ActionPinchLoose
    ActionPinchGeneric
)

target_link_libraries(ActionTrig
    ActionPrimitive
)

target_link_libraries(ActionSingleJointMultipleTips
    ActionPrimitive
)

target_link_libraries(ActionMultiplePinchTight
    ActionPinchGeneric
)

target_link_libraries(ActionGeneric
    Action
)

target_link_libraries(ActionComposed
    ActionGeneric
)

target_link_libraries(ActionTimed
    Action
)

target_link_libraries(FindActions
    ${catkin_LIBRARIES}
    ParserMoveIt
    YamlWorker
    Action
    ActionPinchTight
    ActionTrig
    ActionPinchLoose
    ActionSingleJointMultipleTips
    ActionMultiplePinchTight
    muparser
)

target_link_libraries(ParserMoveIt
    ${catkin_LIBRARIES}
)

target_link_libraries(UniversalRosEndEffector
    UniversalRosEndEffectorExecutor
    
)

target_link_libraries(UniversalFindActions
    ParserMoveIt
    ROSEEParser
    FindActions
    Action
    ActionPrimitive
    ActionTrig
    ActionPinchTight
    ActionPinchLoose
    ActionSingleJointMultipleTips
    ActionTimed
    ActionGeneric
    MapActionHandler
)

if (${matlogger2_FOUND})
    target_link_libraries(EEHalExecutor
        ${catkin_LIBRARIES}
        EEHal
        matlogger2::matlogger2
        #KEEP THIS. ERROR HAPPENS OTHERWISE. not anymore?
        #XBot2Hal
    )
    
else()
    target_link_libraries(EEHalExecutor
        ${catkin_LIBRARIES}
        EEHal
        #KEEP THIS. ERROR HAPPENS OTHERWISE. not anymore?
        #XBot2Hal
    )
endif(${matlogger2_FOUND})

target_link_libraries(EEHal 
    ${catkin_LIBRARIES}
    yaml-cpp
)
add_dependencies(EEHal rosee_msg_generate_messages_cpp)

target_link_libraries(DummyHal
    ${catkin_LIBRARIES}
    EEHal 
)

if(BUILD_WITH_XBOT2)
    target_link_libraries(XBot2Hal
        xbot2::xbot2
        EEHal
    )
endif(BUILD_WITH_XBOT2)


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
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

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
    #catkin_add_gtest(EEInterface_test test/test_ee_interface.cpp)
    add_executable(EEInterface_test test/test_ee_interface.cpp)
    target_link_libraries(EEInterface_test ${catkin_LIBRARIES}
                                           ROSEEParser
                                           ROSEEInterface
                                           gtest)
                                           
    add_executable(FindPinches_test test/test_find_pinches.cpp)
    target_link_libraries(FindPinches_test ${catkin_LIBRARIES}
                                            FindActions
                                            ActionPrimitive
                                            ActionPinchTight
                                            ActionPinchLoose
                                            gtest)
                                            
    add_executable(FindTrigs_test test/test_find_trigs.cpp)
    target_link_libraries(FindTrigs_test ${catkin_LIBRARIES}
                                         FindActions
                                         ActionPrimitive
                                         ActionTrig
                                         gtest)
                                         
    add_executable(ComposedAction_test test/test_composedAction.cpp)
    target_link_libraries(ComposedAction_test ${catkin_LIBRARIES}
                                              FindActions
                                              Action
                                              ActionPrimitive
                                              ActionTrig
                                              ActionComposed
                                              gtest)
                                              
    add_executable(TimedAction_test test/test_timedAction.cpp)
    target_link_libraries(TimedAction_test ${catkin_LIBRARIES}
                                              FindActions
                                              Action
                                              ActionPrimitive
                                              ActionTrig
                                              ActionPinchGeneric
                                              ActionComposed
                                              gtest)
                                              
    add_executable(SendAction_test test/test_send_action.cpp)
    target_link_libraries(SendAction_test ${catkin_LIBRARIES}
                                              Action
                                              ActionGeneric
                                              ActionComposed
                                              gtest
                                              YamlWorker
                                              ROSEEParser
                                              ROSEEInterface
                                              FindActions)  
                                              
    add_executable(ServiceHandler_test test/test_service_handler.cpp)
    target_link_libraries(ServiceHandler_test ${catkin_LIBRARIES}
                                              gtest
                                              ROSEEParser
                                              ROSEEInterface
                                              MapActionHandler
                                              RosServiceHandler)  
    
    list(APPEND testsExecutable 
        EEInterface_test 
        FindPinches_test 
        FindTrigs_test 
        ComposedAction_test 
        TimedAction_test
        SendAction_test 
        ServiceHandler_test
    )
    list(APPEND handsForTests
         test_ee 
         test_ee_spread 
         two_finger 
         two_finger_mimic
    )
    
    #run each test with each hand
    foreach(hand ${handsForTests})
        foreach(testEx ${testsExecutable})
        
            add_test(
                NAME ${testEx}_${hand}
                COMMAND ${testEx} ${hand}
            )

        endforeach()    
    endforeach()
    

    #for code coverage, from Arturo xbot2
    add_custom_target(test_clean_coverage
        COMMAND /bin/sh ${CMAKE_CURRENT_SOURCE_DIR}/test/scripts/clean_gcov.sh
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    
    add_custom_target(test_coverage
        COMMAND lcov -c -d . --output-file main_coverage.info
        #remove external libraries in the coverage
        COMMAND lcov -r main_coverage.info '/usr*' 'boost*' 'eigen3*' 'c++*' '/opt*' -o main_coverage.info
        COMMAND genhtml main_coverage.info --output-directory coverage_out
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
                
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

#unset(BUILD_WITH_XBOT2 CACHE) # todo check if necesasry
