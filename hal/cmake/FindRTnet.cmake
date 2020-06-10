# set the search paths
set( RTnet_SEARCH_PATH /usr/local/rtnet /usr/rtnet $ENV{RTNET_ROOT_DIR})

# find rtnet.h
find_path( RTnet_DIR
NAMES include/rtnet.h
PATHS ${RTnet_SEARCH_PATH} )

if( RTnet_DIR ) 
    MESSAGE(STATUS "rtnet found: \"${RTnet_DIR}\"")
    include_directories(${RTnet_DIR}/include)
    set(RTnet_FOUND True)
else( RTnet_DIR )
    MESSAGE(STATUS "rtnet NOT found. (${RTnet_SEARCH_PATH})")
endif( RTnet_DIR )