set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  )

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS})

add_library(WrenchStampedROSBridge SHARED WrenchStampedROSBridge.cpp)
target_link_libraries(WrenchStampedROSBridge ${catkin_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(WrenchStampedROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

add_executable(WrenchStampedROSBridgeComp WrenchStampedROSBridgeComp.cpp)
target_link_libraries(WrenchStampedROSBridgeComp WrenchStampedROSBridge)

install(TARGETS WrenchStampedROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS WrenchStampedROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
