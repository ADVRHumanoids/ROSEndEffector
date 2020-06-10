
macro(set_cap_build target)
  message(STATUS "set_cap.sh ${target}")
  add_custom_command(TARGET ${target} POST_BUILD COMMAND ${CMAKE_SOURCE_DIR}/scripts/set_cap.sh ${target} )
endmacro(set_cap_build)

macro(set_cap_install target)
  install(CODE "message(STATUS set_cap.sh \\ ${target})" )
  install(CODE "execute_process(COMMAND ${CMAKE_SOURCE_DIR}/scripts/set_cap.sh ${target})" )
endmacro(set_cap_install)
