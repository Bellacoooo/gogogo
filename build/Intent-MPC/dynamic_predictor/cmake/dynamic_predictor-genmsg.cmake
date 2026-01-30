# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dynamic_predictor: 3 messages, 0 services")

set(MSG_I_FLAGS "-Idynamic_predictor:/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dynamic_predictor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" NAME_WE)
add_custom_target(_dynamic_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_predictor" "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" "geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" NAME_WE)
add_custom_target(_dynamic_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_predictor" "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" "dynamic_predictor/PredictedTrajectory:geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" NAME_WE)
add_custom_target(_dynamic_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dynamic_predictor" "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" "dynamic_predictor/PredictedTrajectory:geometry_msgs/Point:std_msgs/Header:dynamic_predictor/PredictedObstacle:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_cpp(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_cpp(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_predictor
)

### Generating Services

### Generating Module File
_generate_module_cpp(dynamic_predictor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_predictor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dynamic_predictor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dynamic_predictor_generate_messages dynamic_predictor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_cpp _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_cpp _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_cpp _dynamic_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_predictor_gencpp)
add_dependencies(dynamic_predictor_gencpp dynamic_predictor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_predictor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_eus(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_eus(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_predictor
)

### Generating Services

### Generating Module File
_generate_module_eus(dynamic_predictor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_predictor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dynamic_predictor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dynamic_predictor_generate_messages dynamic_predictor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_eus _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_eus _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_eus _dynamic_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_predictor_geneus)
add_dependencies(dynamic_predictor_geneus dynamic_predictor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_predictor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_lisp(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_lisp(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_predictor
)

### Generating Services

### Generating Module File
_generate_module_lisp(dynamic_predictor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_predictor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dynamic_predictor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dynamic_predictor_generate_messages dynamic_predictor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_lisp _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_lisp _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_lisp _dynamic_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_predictor_genlisp)
add_dependencies(dynamic_predictor_genlisp dynamic_predictor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_predictor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_nodejs(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_nodejs(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_predictor
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dynamic_predictor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_predictor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dynamic_predictor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dynamic_predictor_generate_messages dynamic_predictor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_nodejs _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_nodejs _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_nodejs _dynamic_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_predictor_gennodejs)
add_dependencies(dynamic_predictor_gennodejs dynamic_predictor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_predictor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_py(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor
)
_generate_msg_py(dynamic_predictor
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor
)

### Generating Services

### Generating Module File
_generate_module_py(dynamic_predictor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dynamic_predictor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dynamic_predictor_generate_messages dynamic_predictor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedTrajectory.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_py _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacle.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_py _dynamic_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ff/intent-mpc/src/Intent-MPC/dynamic_predictor/msg/PredictedObstacles.msg" NAME_WE)
add_dependencies(dynamic_predictor_generate_messages_py _dynamic_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dynamic_predictor_genpy)
add_dependencies(dynamic_predictor_genpy dynamic_predictor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dynamic_predictor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamic_predictor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dynamic_predictor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dynamic_predictor_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dynamic_predictor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dynamic_predictor_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dynamic_predictor_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamic_predictor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dynamic_predictor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dynamic_predictor_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dynamic_predictor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dynamic_predictor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dynamic_predictor_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamic_predictor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dynamic_predictor_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dynamic_predictor_generate_messages_py geometry_msgs_generate_messages_py)
endif()
