CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

PROJECT("pymtuav-competition")

IF(DEFINED PYBIND11_INCLUDE_ROOT)
  MESSAGE(STATUS "pybind11 root: ${PYBIND11_INCLUDE_ROOT}")
  INCLUDE_DIRECTORIES(
      # /home/marco/miniconda3/envs/pybind11/include/python3.10
      ${PYBIND11_INCLUDE_ROOT}/include/python3.10
      # /home/marco/miniconda3/envs/pybind11/lib/python3.10/site-packages/pybind11/include
      ${PYBIND11_INCLUDE_ROOT}/lib/python3.10/site-packages/pybind11/include
  )
  SET(PYBIND11_OUTPUT_EXTENSION ".cpython-310-x86_64-linux-gnu.so")
ELSE ()
  execute_process(COMMAND python3 -m pybind11 --includes
                  RESULT_VARIABLE PYBIND11_INCLUDE_FAIL
                  OUTPUT_VARIABLE PYBIND11_INCLUDE_ROOT
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  IF(PYBIND11_INCLUDE_FAIL)
    MESSAGE(FATAL_ERROR "fail to find PYBIND11_INCLUDE_ROOT")
  ENDIF ()
  MESSAGE(STATUS "pybind11 root: ${PYBIND11_INCLUDE_ROOT}")

  execute_process(COMMAND python3-config --extension-suffix
                  RESULT_VARIABLE PYBIND11_EXTENSION_FAIL
                  OUTPUT_VARIABLE PYBIND11_OUTPUT_EXTENSION
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  IF(PYBIND11_EXTENSION_FAIL)
    MESSAGE(FATAL_ERROR "fail to find extension-suffix")
  ENDIF ()
  MESSAGE(STATUS "pybind11 extension: ${PYBIND11_OUTPUT_EXTENSION}")

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PYBIND11_INCLUDE_ROOT}")
ENDIF ()

ADD_LIBRARY(pymtuav SHARED pymtuav.cc)

TARGET_LINK_LIBRARIES(pymtuav mtuav_sdk)

SET_TARGET_PROPERTIES(pymtuav
                      PROPERTIES
                      OUTPUT_NAME pymtuav${PYBIND11_OUTPUT_EXTENSION}
                      PREFIX "" SUFFIX "")

IF(CMAKE_BUILD_TYPE MATCHES "Release")
  MESSAGE(STATUS "----------------------------RELEASE BUILD---------------------")
  ADD_CUSTOM_COMMAND(
      TARGET pymtuav  POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
      ${CMAKE_CURRENT_BINARY_DIR}/*.so
      ${CMAKE_CURRENT_SOURCE_DIR}/../libs/)
ENDIF ()