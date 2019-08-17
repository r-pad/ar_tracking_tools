cmake_minimum_required(VERSION 2.8.3)
project(ar_tracking_tools)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED
    ar_track_alvar REQUIRED
    COMPONENTS 
    message_generation 
    std_msgs 
    tf
    geometry_msgs
    sensor_msgs
    message_filters
    tf2_ros
    cv_bridge
    ar_track_alvar
    ar_track_alvar_msgs
    tf2_geometry_msgs
    tf_conversions
    visualization_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_python_setup()

catkin_package(
    LIBRARIES 
    kinect_filtering
    medianFilter
    CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    tf
    geometry_msgs
    sensor_msgs
    message_filters
    image_geometry
    tf2_ros
    cv_bridge
    ar_track_alvar
    ar_track_alvar_msgs
    tf2_geometry_msgs
    tf_conversions
    visualization_msgs
  )



include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS}
                    )

add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
# This is definately the wrong way to include medianFilter and kinetic_filtering
target_link_libraries(findMarkerBundles ${catkin_LIBRARIES} 
    ${CMAKE_PREFIX_PATH}/${CATKIN_PACKAGE_LIB_DESTINATION}/libmedianFilter.so
    ${CMAKE_PREFIX_PATH}/${CATKIN_PACKAGE_LIB_DESTINATION}/libkinect_filtering.so )
add_dependencies(findMarkerBundles ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ${catkin_LIBRARIES})
add_dependencies(findMarkerBundlesNoKinect ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

add_executable(alvarBundleTracker nodes/alvar_bundle_tracker_node.cpp 
    src_cpp/alvar_bundle_tracker.cpp 
    src_cpp/consensus_bundle_refinement.cpp)
target_link_libraries(alvarBundleTracker ${catkin_LIBRARIES}
    ${CMAKE_PREFIX_PATH}/${CATKIN_PACKAGE_LIB_DESTINATION}/libmedianFilter.so
    ${CMAKE_PREFIX_PATH}/${CATKIN_PACKAGE_LIB_DESTINATION}/libkinect_filtering.so )
add_dependencies(alvarBundleTracker ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})


install(TARGETS
    alvarBundleTracker
    findMarkerBundles
    findMarkerBundlesNoKinect
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY 
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/ar_ransac.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
