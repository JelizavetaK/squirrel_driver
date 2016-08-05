cmake_minimum_required(VERSION 2.8.3)
project(ft17_driver)

find_package(catkin REQUIRED)

find_program(CMAKE_DPKG dpkg /bin /usr/bin /usr/local/bin) 
if(CMAKE_DPKG)
  exec_program(dpkg ARGS --print-architecture OUTPUT_VARIABLE CMAKE_DPKG_ARCH)
  if(CMAKE_DPKG_ARCH MATCHES "amd64")
    message(STATUS "++++++++++++++++++++++++++ DETECTED 64 bit +++++++++++++++++++++++++++++++++++")
    set(MYARCH "x86_64")
  else()
    message(STATUS "++++++++++++++++++++++++++ DETECTED 32 bit +++++++++++++++++++++++++++++++++++")
    set(MYARCH "i686")
  endif()
endif()

catkin_package(
    INCLUDE_DIRS common/include
    LIBRARIES FT17_driver
)

include_directories(common/include)

message(STATUS "Copying ${PROJECT_SOURCE_DIR}/common/lib/${MYARCH}/libFT17_driver.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libFT17_driver.so")

add_custom_target(
    build_ft17_driver ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/${MYARCH}/libFT17_driver.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_library(FT17_driver SHARED)
set_target_properties(FT17_driver PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(FT17_driver build_ft17_driver)

install(TARGETS FT17_driver
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS common/lib/${MYARCH}/libFT17_driver.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY common/include/FT17/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY common doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
