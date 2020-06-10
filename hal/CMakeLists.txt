# CMake minimum version and policies
cmake_minimum_required(VERSION 3.0.0)

# XBotEcat
project(Heri2EEHal)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)
include(MacroInstallLibrary)

# C++11 
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable Xenomai flag and if enabled find required packages
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
    find_package(Xenomai)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

# find packages 
find_package(soem${LIB_TYPE} REQUIRED)
find_package(iitecat_iface${LIB_TYPE} REQUIRED)
find_package(ecat_master${LIB_TYPE} REQUIRED)

# XBot trick
find_package(XCM REQUIRED)
find_package(XBotCoreThread REQUIRED)

set(COMMON_LIBS
    pthread
    jsoncpp
    yaml-cpp
)


# include 
include_directories(include 
                    ${soem${LIB_TYPE}_INCLUDE_DIRS}
                    ${iitecat_iface${LIB_TYPE}_INCLUDE_DIRS}
                    ${ecat_master${LIB_TYPE}_INCLUDE_DIRS}
                    )
                            
                       
# lib
add_library(${PROJECT_NAME}
    src/heri_2_ee_hal.cpp
)

# set Xenomai flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
 
endif()

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS}                                      
                                      ${soem${LIB_TYPE}_LIBRARIES} 
                                      ${iitecat_iface${LIB_TYPE}_LIBRARIES}
                                      ${ecat_master${LIB_TYPE}_LIBRARIES} 
                                      ${XBotCoreThread_LIBRARIES}
                                      EEHal
                                      )                                 

                                      
library_install(${PROJECT_NAME} 1 0 0)

# TODO automatic set set_cap -> remember to ./set_cap.sh ~/src/ROSEndEffectorPackageManager/devel/lib/ros_end_effector/UniversalRosEndEffector