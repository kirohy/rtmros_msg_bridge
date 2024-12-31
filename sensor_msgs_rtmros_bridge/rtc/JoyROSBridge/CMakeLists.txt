add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
add_library(JoyROSBridge SHARED JoyROSBridge.cpp)
target_link_libraries(JoyROSBridge ${catkin_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(JoyROSBridge PROPERTIES PREFIX "") # libJoyROSBridge.so -> JoyROSBridge.so

add_executable(JoyROSBridgeComp JoyROSBridgeComp.cpp)
target_link_libraries(JoyROSBridgeComp JoyROSBridge)

install(TARGETS JoyROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS JoyROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
