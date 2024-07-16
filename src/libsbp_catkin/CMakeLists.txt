cmake_minimum_required(VERSION 2.8.3)
project(libsbp_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

catkin_python_setup() # Creates python2.7/dist-packages folder.

ExternalProject_Add(libsbp
  GIT_REPOSITORY https://github.com/swift-nav/libsbp.git
  GIT_TAG v2.7.4
  UPDATE_COMMAND git submodule update --init
  PATCH_COMMAND
    COMMAND patch generator/sbpg/specs/yaml2.py ${CMAKE_CURRENT_SOURCE_DIR}/patches/yaml_safe_load.patch
  SOURCE_SUBDIR ./c/
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install && cp -r ../libsbp/generator/sbpg ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
)

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          CFG_EXTRAS libsbp-extras.cmake)
