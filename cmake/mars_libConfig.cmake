if (NOT TARGET mars_lib)
  include(ExternalProject)
  externalproject_add(mars_lib-ext
      PREFIX ${CMAKE_BINARY_DIR}/mars_lib
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mars_lib
      CMAKE_ARGS
      ${CROSS_COMPILE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/mars_lib
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      BUILD_ALWAYS 1
      )
  add_library(mars_lib INTERFACE IMPORTED GLOBAL)
  add_dependencies(mars_lib mars_lib-ext)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/mars_lib)
  set_property(TARGET mars_lib PROPERTY INTERFACE_INCLUDE_DIRECTORIES
      ${CMAKE_BINARY_DIR}/mars_lib/include/mars
      )
endif()

#set(MARS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/mars_lib/include/mars)

#include(ExternalProject)

#ExternalProject_Add( mars_lib
#  PREFIX mars_cpp_lib
#  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mars_lib/mars_cpp
#  BUILD_IN_SOURCE
#  BUILD_COMMAND cmake --build .
#  INSTALL_COMMAND ""
#)

set(MARS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mars_lib/source/mars/include)
set(MARS_LIBRARY_RELEASE ${CMAKE_BINARY_DIR}/mars_lib/lib/libmars.a)
set(MARS_LIBRARY_DEBUG ${CMAKE_BINARY_DIR}/mars_lib/lib/libmarsd.a)
