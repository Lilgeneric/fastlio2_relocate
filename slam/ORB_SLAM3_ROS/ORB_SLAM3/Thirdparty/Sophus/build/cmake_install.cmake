# Install script for directory: /home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake"
         "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/CMakeFiles/Export/4110a4f8b3c48cc11f058d9be1acc9d2/SophusTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sophus/cmake" TYPE FILE FILES "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/CMakeFiles/Export/4110a4f8b3c48cc11f058d9be1acc9d2/SophusTargets.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sophus/cmake" TYPE FILE FILES
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/SophusConfig.cmake"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/SophusConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/sophus" TYPE FILE FILES
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/average.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/common.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/geometry.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/interpolate.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/interpolate_details.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/num_diff.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/rotation_matrix.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/rxso2.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/rxso3.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/se2.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/se3.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/sim2.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/sim3.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/sim_details.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/so2.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/so3.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/types.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/velocities.hpp"
    "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/sophus/formatstring.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/test/cmake_install.cmake")
  include("/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/generic/fastlio2_relocate/src/slam/ORB_SLAM3_ROS/ORB_SLAM3/Thirdparty/Sophus/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
