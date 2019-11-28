find_package(Doxygen QUIET)

if(DOXYGEN_FOUND)
    set(DOX_GENERATE_HTML YES)
    set(DOX_GENERATE_XML NO)

    add_custom_target(dox COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    set(DOX_PATTERNS "") # Will use doxygen default patterns
    set(DOX_GENERATE_MAN NO)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    add_custom_target(man COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.man)
endif()
