cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS
	${CMAKE_SOURCE_DIR}/firmware
	${CMAKE_SOURCE_DIR}/components
	${CMAKE_SOURCE_DIR}/lib
)

project(Avionics)
