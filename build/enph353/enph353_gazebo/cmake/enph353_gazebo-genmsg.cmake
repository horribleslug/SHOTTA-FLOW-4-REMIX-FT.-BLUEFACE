# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "enph353_gazebo: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(enph353_gazebo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" NAME_WE)
add_custom_target(_enph353_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "enph353_gazebo" "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" ""
)

get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" NAME_WE)
add_custom_target(_enph353_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "enph353_gazebo" "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/enph353_gazebo
)
_generate_srv_cpp(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/enph353_gazebo
)

### Generating Module File
_generate_module_cpp(enph353_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/enph353_gazebo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(enph353_gazebo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(enph353_gazebo_generate_messages enph353_gazebo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_cpp _enph353_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_cpp _enph353_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(enph353_gazebo_gencpp)
add_dependencies(enph353_gazebo_gencpp enph353_gazebo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS enph353_gazebo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/enph353_gazebo
)
_generate_srv_eus(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/enph353_gazebo
)

### Generating Module File
_generate_module_eus(enph353_gazebo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/enph353_gazebo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(enph353_gazebo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(enph353_gazebo_generate_messages enph353_gazebo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_eus _enph353_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_eus _enph353_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(enph353_gazebo_geneus)
add_dependencies(enph353_gazebo_geneus enph353_gazebo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS enph353_gazebo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/enph353_gazebo
)
_generate_srv_lisp(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/enph353_gazebo
)

### Generating Module File
_generate_module_lisp(enph353_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/enph353_gazebo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(enph353_gazebo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(enph353_gazebo_generate_messages enph353_gazebo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_lisp _enph353_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_lisp _enph353_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(enph353_gazebo_genlisp)
add_dependencies(enph353_gazebo_genlisp enph353_gazebo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS enph353_gazebo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/enph353_gazebo
)
_generate_srv_nodejs(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/enph353_gazebo
)

### Generating Module File
_generate_module_nodejs(enph353_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/enph353_gazebo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(enph353_gazebo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(enph353_gazebo_generate_messages enph353_gazebo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_nodejs _enph353_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_nodejs _enph353_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(enph353_gazebo_gennodejs)
add_dependencies(enph353_gazebo_gennodejs enph353_gazebo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS enph353_gazebo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/enph353_gazebo
)
_generate_srv_py(enph353_gazebo
  "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/enph353_gazebo
)

### Generating Module File
_generate_module_py(enph353_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/enph353_gazebo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(enph353_gazebo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(enph353_gazebo_generate_messages enph353_gazebo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/GetLegalPlates.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_py _enph353_gazebo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kausikk/enph353/SHOTTA-FLOW-4-REMIX-FT.-BLUEFACE/src/enph353/enph353_gazebo/srv/SubmitPlate.srv" NAME_WE)
add_dependencies(enph353_gazebo_generate_messages_py _enph353_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(enph353_gazebo_genpy)
add_dependencies(enph353_gazebo_genpy enph353_gazebo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS enph353_gazebo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/enph353_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/enph353_gazebo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(enph353_gazebo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(enph353_gazebo_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/enph353_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/enph353_gazebo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(enph353_gazebo_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(enph353_gazebo_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/enph353_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/enph353_gazebo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(enph353_gazebo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(enph353_gazebo_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/enph353_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/enph353_gazebo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(enph353_gazebo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(enph353_gazebo_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/enph353_gazebo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/enph353_gazebo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/enph353_gazebo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(enph353_gazebo_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(enph353_gazebo_generate_messages_py sensor_msgs_generate_messages_py)
endif()
