cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

# replace path to local OWL files
rosbuild_find_ros_package(knowrob_seds)
set( LOCAL_PACKAGE_PATH ${knowrob_seds_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/snakeyaml/snakeyaml-1.11.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/snakeyaml/snakeyaml-1.11.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/guava-14.0.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/guava-14.0.jar)

add_java_source_dir( ${PROJECT_SOURCE_DIR}/src )
rospack_add_java_executable( gmm_to_owl org.knowrob.imitation.GMMToOWL )
