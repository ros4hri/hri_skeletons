# Copyright (C) 2018-2019 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)

find_package(PythonInterp 3.5 REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_STRING}" EXACT REQUIRED)

execute_process(
COMMAND "${PYTHON_EXECUTABLE}" -c "import numpy; print(numpy.get_include())"
OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
OUTPUT_STRIP_TRAILING_WHITESPACE
RESULT_VARIABLE NUMPY_NOT_FOUND)
if(NUMPY_NOT_FOUND)
message(FATAL_ERROR "NumPy headers not found")
endif()

#find_package(PythonLibs 3.6 REQUIRED)
message("Found Python ${PYTHONLIBS_VERSION_STRING} dev libraries")

set(target_name pose_extractor)
add_library(${target_name} MODULE wrapper.cpp
                                  src/extract_poses.hpp src/extract_poses.cpp
                                  src/human_pose.hpp src/human_pose.cpp
                                  src/peak.hpp src/peak.cpp)
target_include_directories(${target_name} PRIVATE src/ ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
target_link_libraries(${target_name} ${PYTHON_LIBRARIES} opencv_core opencv_imgproc)
set_target_properties(${target_name} PROPERTIES PREFIX "")
if(WIN32)
    set_target_properties(${target_name} PROPERTIES SUFFIX ".pyd")
endif()

install(TARGETS ${target_name}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



