# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(rtabmap_ros_CONFIG_INCLUDED)
  return()
endif()
set(rtabmap_ros_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(rtabmap_ros_SOURCE_PREFIX /tmp/binarydeb/ros-kinetic-rtabmap-ros-0.19.3)
  set(rtabmap_ros_DEVEL_PREFIX /tmp/binarydeb/ros-kinetic-rtabmap-ros-0.19.3/obj-x86_64-linux-gnu/devel)
  set(rtabmap_ros_INSTALL_PREFIX "")
  set(rtabmap_ros_PREFIX ${rtabmap_ros_DEVEL_PREFIX})
else()
  set(rtabmap_ros_SOURCE_PREFIX "")
  set(rtabmap_ros_DEVEL_PREFIX "")
  set(rtabmap_ros_INSTALL_PREFIX /opt/ros/kinetic)
  set(rtabmap_ros_PREFIX ${rtabmap_ros_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'rtabmap_ros' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(rtabmap_ros_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include;/opt/ros/kinetic/lib/x86_64-linux-gnu/rtabmap-0.19/../../../include/rtabmap-0.19;/opt/ros/kinetic/include/opencv-3.3.1-dev;/opt/ros/kinetic/include/opencv-3.3.1-dev/opencv " STREQUAL " ")
  set(rtabmap_ros_INCLUDE_DIRS "")
  set(_include_dirs "include;/opt/ros/kinetic/lib/x86_64-linux-gnu/rtabmap-0.19/../../../include/rtabmap-0.19;/opt/ros/kinetic/include/opencv-3.3.1-dev;/opt/ros/kinetic/include/opencv-3.3.1-dev/opencv")
  if(NOT "https://github.com/introlab/rtabmap_ros/issues " STREQUAL " ")
    set(_report "Check the issue tracker 'https://github.com/introlab/rtabmap_ros/issues' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Mathieu Labbe <matlabbe@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${rtabmap_ros_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'rtabmap_ros' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'rtabmap_ros' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/opt/ros/kinetic/${idir}'.  ${_report}")
    endif()
    _list_append_unique(rtabmap_ros_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "rtabmap_ros;/opt/ros/kinetic/lib/x86_64-linux-gnu/librtabmap_core.so;/opt/ros/kinetic/lib/x86_64-linux-gnu/librtabmap_utilite.so;/opt/ros/kinetic/lib/x86_64-linux-gnu/librtabmap_gui.so;/usr/lib/x86_64-linux-gnu/libz.so;/opt/ros/kinetic/lib/libg2o_core.so;/opt/ros/kinetic/lib/libg2o_types_slam2d.so;/opt/ros/kinetic/lib/libg2o_types_slam3d.so;/opt/ros/kinetic/lib/libg2o_types_sba.so;/opt/ros/kinetic/lib/libg2o_stuff.so;/opt/ros/kinetic/lib/libg2o_solver_csparse.so;/opt/ros/kinetic/lib/libg2o_csparse_extension.so;/usr/lib/x86_64-linux-gnu/libcxsparse.so;/opt/ros/kinetic/lib/libg2o_solver_cholmod.so;/usr/lib/x86_64-linux-gnu/libcholmod.so;/usr/lib/x86_64-linux-gnu/libfreenect.so;/usr/lib/x86_64-linux-gnu/libfreenect_sync.so;/opt/ros/kinetic/lib/liboctomap.so;/opt/ros/kinetic/lib/liboctomath.so;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1;/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND rtabmap_ros_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND rtabmap_ros_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND rtabmap_ros_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND rtabmap_ros_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /opt/ros/kinetic/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(rtabmap_ros_LIBRARY_DIRS ${lib_path})
      list(APPEND rtabmap_ros_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'rtabmap_ros'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND rtabmap_ros_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(rtabmap_ros_EXPORTED_TARGETS "rtabmap_ros_generate_messages_cpp;rtabmap_ros_generate_messages_eus;rtabmap_ros_generate_messages_lisp;rtabmap_ros_generate_messages_nodejs;rtabmap_ros_generate_messages_py;rtabmap_ros_gencfg")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${rtabmap_ros_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "cv_bridge;roscpp;rospy;sensor_msgs;std_msgs;std_srvs;nav_msgs;geometry_msgs;visualization_msgs;image_transport;tf;tf_conversions;tf2_ros;eigen_conversions;laser_geometry;pcl_conversions;pcl_ros;nodelet;dynamic_reconfigure;message_filters;class_loader;rosgraph_msgs;stereo_msgs;move_base_msgs;image_geometry;costmap_2d;octomap_msgs;rviz;find_object_2d")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 rtabmap_ros_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${rtabmap_ros_dep}_FOUND)
      find_package(${rtabmap_ros_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${rtabmap_ros_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(rtabmap_ros_INCLUDE_DIRS ${${rtabmap_ros_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(rtabmap_ros_LIBRARIES ${rtabmap_ros_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${rtabmap_ros_dep}_LIBRARIES})
  _list_append_deduplicate(rtabmap_ros_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(rtabmap_ros_LIBRARIES ${rtabmap_ros_LIBRARIES})

  _list_append_unique(rtabmap_ros_LIBRARY_DIRS ${${rtabmap_ros_dep}_LIBRARY_DIRS})
  list(APPEND rtabmap_ros_EXPORTED_TARGETS ${${rtabmap_ros_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "rtabmap_ros-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${rtabmap_ros_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
