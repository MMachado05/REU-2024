cmake_minimum_required(VERSION 2.8.3)
project(libserialport_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(libserialport
        GIT_REPOSITORY https://github.com/sigrokproject/libserialport.git
        GIT_TAG 6f9b03e
        CONFIGURE_COMMAND $(CMAKE_BINARY_DIR)/libserialport-prefix/src/libserialport/autogen.sh
          COMMAND $(CMAKE_BINARY_DIR)/libserialport-prefix/src/libserialport/configure --prefix=${CATKIN_DEVEL_PREFIX}
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND $(MAKE) install
        )

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
        CFG_EXTRAS libserialport-extras.cmake)