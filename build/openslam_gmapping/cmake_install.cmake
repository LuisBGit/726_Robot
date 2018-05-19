# Install script for directory: /home/luis/Desktop/GitWorkSpace/726_Robot/src/openslam_gmapping

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/luis/Desktop/GitWorkSpace/726_Robot/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/catkin_generated/installspace/openslam_gmapping.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/openslam_gmapping/cmake" TYPE FILE FILES
    "/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/catkin_generated/installspace/openslam_gmappingConfig.cmake"
    "/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/catkin_generated/installspace/openslam_gmappingConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/openslam_gmapping" TYPE FILE FILES "/home/luis/Desktop/GitWorkSpace/726_Robot/src/openslam_gmapping/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/luis/Desktop/GitWorkSpace/726_Robot/src/openslam_gmapping/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hxx$" REGEX "/\\.svn$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/gridfastslam/cmake_install.cmake")
  include("/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/scanmatcher/cmake_install.cmake")
  include("/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/sensor/cmake_install.cmake")
  include("/home/luis/Desktop/GitWorkSpace/726_Robot/build/openslam_gmapping/utils/cmake_install.cmake")

endif()

