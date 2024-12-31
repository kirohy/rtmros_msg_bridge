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
add_library(PoseStampedROSBridge SHARED PoseStampedROSBridge.cpp)
target_link_libraries(PoseStampedROSBridge ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(PoseStampedROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

add_executable(PoseStampedROSBridgeComp PoseStampedROSBridgeComp.cpp)
target_link_libraries(PoseStampedROSBridgeComp PoseStampedROSBridge)

install(TARGETS PoseStampedROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS PoseStampedROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
