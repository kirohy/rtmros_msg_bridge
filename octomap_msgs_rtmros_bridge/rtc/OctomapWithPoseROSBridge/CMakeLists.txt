set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_msgs
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/.. # for /idl
  ${openrtm_aist_INCLUDE_DIRS}
  )
  
add_library(OctomapWithPoseROSBridge SHARED OctomapWithPoseROSBridge.cpp)
target_link_libraries(OctomapWithPoseROSBridge
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS})
set_target_properties(OctomapWithPoseROSBridge PROPERTIES PREFIX "") # libPrimitiveStateROSBridge.so -> PrimitiveStateROSBridge.so
add_dependencies(OctomapWithPoseROSBridge RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(OctomapWithPoseROSBridgeComp OctomapWithPoseROSBridgeComp.cpp)
target_link_libraries(OctomapWithPoseROSBridgeComp OctomapWithPoseROSBridge)

install(TARGETS OctomapWithPoseROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS OctomapWithPoseROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
