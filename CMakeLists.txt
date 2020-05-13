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
                                        rospy
                                        message_generation 
                                        moveit_ros_planning_interface
                                        rosee_msg
                                        )
                                                                               
# find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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
    src/EEHal.cpp
    src/DummyHal.cpp
)

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

add_library(MapActionHandler
    src/MapActionHandler.cpp
)

add_library(Action 
    src/Action.cpp 
)

add_library(ActionPrimitive 
    src/ActionPrimitive.cpp
)

add_library(ActionPinchTight
    src/ActionPinchTight.cpp
)
add_library(ActionPinchGeneric
    src/ActionPinchGeneric.cpp
)
add_library(ActionPinchLoose
    src/ActionPinchLoose.cpp
)
add_library(ActionTrig
    src/ActionTrig.cpp
)
add_library(ActionSingleJointMultipleTips
    src/ActionSingleJointMultipleTips.cpp
)
add_library(ActionMultiplePinchTight
    src/ActionMultiplePinchTight.cpp
)
add_library(ActionGeneric
    src/ActionGeneric.cpp 
)
add_library(ActionComposed
    src/ActionComposed.cpp 
)
add_library(ActionTimed
    src/ActionTimed.cpp 
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(UniversalRosEndEffector src/UniversalRosEndEffector.cpp)
add_executable(UniversalFindActions src/UniversalFindActions.cpp)
#add_dependencies(UniversalRosEndEffectorExecutor ${catkin_EXPORTED_TARGETS})
#add_dependencies(UniversalFindActions ${catkin_EXPORTED_TARGETS})


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
    YamlWorker
    MapActionHandler
    RosActionServer
    
    # do it automatically
    DummyHal
    
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
    
    list(APPEND testsExecutable 
        EEInterface_test FindPinches_test FindTrigs_test ComposedAction_test
    )
    list(APPEND handsForTests
         test_ee two_finger two_finger_mimic
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
