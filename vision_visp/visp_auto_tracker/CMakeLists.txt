cmake_minimum_required(VERSION 2.8.3)
project(visp_auto_tracker)

# Since kinetic to avoid warning:
#
# In file included from /home/fspindle/ros_catkin_ws/src/vision_visp/visp_auto_tracker/src/node.cpp:36:0:
# /opt/ros/kinetic/include/resource_retriever/retriever.h:81:38: warning: defaulted and deleted functions
# only available with -std=c++11 or -std=gnu++11
#   Retriever(const Retriever & ret) = delete;
#
# we add -std=c++11
# check c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=gnu++11" COMPILER_SUPPORTS_GNUXX11)

#catkin_lint: ignore endblock_args indentation
if(COMPILER_SUPPORTS_CXX11)
  set(EXTRA_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_GNUXX11)
  set(EXTRA_CXX_FLAGS "-std=gnu++11")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  visp_bridge
  visp_tracker
)
# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP)

#find_package(Boost REQUIRED
  #COMPONENTS filesystem system signals regex date_time program_options thread)
set(Boost_INCLUDE_DIRS /usr/include/boost)


include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   LIBRARIES auto_tracker

   CATKIN_DEPENDS
   geometry_msgs
   message_filters
   roscpp
   sensor_msgs
   std_msgs

   DEPENDS
   VISP
   )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/flashcode_mbt)

# Library gathering libauto_tracker used by all nodes.
add_library(${PROJECT_NAME}_cmd_line
  flashcode_mbt/cmd_line/cmd_line.cpp
  flashcode_mbt/cmd_line/cmd_line.h
)
target_link_libraries(${PROJECT_NAME}_cmd_line
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  add_library(${PROJECT_NAME}_qrcode_detector
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
    flashcode_mbt/detectors/qrcode/detector.cpp
    flashcode_mbt/detectors/qrcode/detector.h
  )
  target_link_libraries(${PROJECT_NAME}_qrcode_detector
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    zbar)

  add_library(${PROJECT_NAME}_datamatrix_detector
    flashcode_mbt/detectors/datamatrix/detector.cpp
    flashcode_mbt/detectors/datamatrix/detector.h
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
  )
  target_link_libraries(${PROJECT_NAME}_datamatrix_detector
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    dmtx)
endif()

add_library(auto_tracker
  flashcode_mbt/libauto_tracker/events.h
  flashcode_mbt/libauto_tracker/logfilewriter.hpp
  flashcode_mbt/libauto_tracker/states.hpp
  flashcode_mbt/libauto_tracker/threading.cpp
  flashcode_mbt/libauto_tracker/threading.h
  flashcode_mbt/libauto_tracker/tracking.cpp
  flashcode_mbt/libauto_tracker/tracking.h)

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  target_link_libraries(auto_tracker
      ${PROJECT_NAME}_cmd_line
      ${PROJECT_NAME}_qrcode_detector
      ${PROJECT_NAME}_datamatrix_detector
      ${resource_retriever_LIBRARIES}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
  )
else()
  target_link_libraries(auto_tracker
      ${PROJECT_NAME}_cmd_line
      ${resource_retriever_LIBRARIES}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
  )
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
  )

if(EXTRA_CXX_FLAGS)
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
endif()

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_bag)

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  target_link_libraries(${PROJECT_NAME}
    auto_tracker
    ${PROJECT_NAME}_qrcode_detector
    ${PROJECT_NAME}_datamatrix_detector
    ${PROJECT_NAME}_cmd_line
    dmtx zbar)

  target_link_libraries(${PROJECT_NAME}
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cmd_line
      ${PROJECT_NAME}_datamatrix_detector
      ${PROJECT_NAME}_qrcode_detector
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  target_link_libraries(${PROJECT_NAME}
    auto_tracker
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cmd_line
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
# Bag files #
#############

# Tutorial
if(CATKIN_ENABLE_TESTING)
  catkin_download_test_data(
    ${PROJECT_NAME}_bag
    https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-qrcode.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    MD5 0f80ceea2610b8400591ca7aff764dfa)

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()
