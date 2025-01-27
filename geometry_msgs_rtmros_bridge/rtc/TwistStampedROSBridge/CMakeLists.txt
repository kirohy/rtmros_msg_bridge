set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS})
add_library(TwistStampedROSBridge SHARED TwistStampedROSBridge.cpp)
target_link_libraries(TwistStampedROSBridge ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(TwistStampedROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

add_executable(TwistStampedROSBridgeComp TwistStampedROSBridgeComp.cpp)
target_link_libraries(TwistStampedROSBridgeComp TwistStampedROSBridge)

install(TARGETS TwistStampedROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS TwistStampedROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
