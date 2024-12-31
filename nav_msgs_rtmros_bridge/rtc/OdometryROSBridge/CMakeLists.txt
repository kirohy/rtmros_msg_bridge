set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf2
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_library(OdometryROSBridge SHARED OdometryROSBridge.cpp)
target_link_libraries(OdometryROSBridge ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(OdometryROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

add_executable(OdometryROSBridgeComp OdometryROSBridgeComp.cpp)
target_link_libraries(OdometryROSBridgeComp OdometryROSBridge)

install(TARGETS OdometryROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS OdometryROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
