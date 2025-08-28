# Install script for directory: /home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/g2o/examples

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/data_fitting/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/sphere/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/target/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/ba/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/ba_anchored_inverse_depth/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/tutorial_slam2d/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/icp/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/calibration_odom_laser/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/simple_optimize/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/plane_slam/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/line_slam/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/sba/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/bal/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/data_convert/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/interactive_slam/cmake_install.cmake")
  include("/home/generic/point_lio_Ackermann/src/dependencies/g2o-20200410/build/g2o/examples/sim3/cmake_install.cmake")

endif()

