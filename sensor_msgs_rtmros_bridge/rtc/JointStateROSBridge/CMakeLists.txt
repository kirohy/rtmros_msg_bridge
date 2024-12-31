set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS})
add_library(JointStateROSBridge SHARED JointStateROSBridge.cpp)
target_link_libraries(JointStateROSBridge ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(JointStateROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

add_executable(JointStateROSBridgeComp JointStateROSBridgeComp.cpp)
target_link_libraries(JointStateROSBridgeComp JointStateROSBridge)

install(TARGETS JointStateROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS JointStateROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
