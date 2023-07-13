cmake_minimum_required(VERSION 2.8.12)
project(experiments)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_CXX_STANDARD 11)
# Find the ARGoS package
# find_package(PkgConfig)
# pkg_check_modules(ARGOS REQUIRED argos3_simulator)
find_package(ARGoS COMPONENTS footbot eyebot spiri)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Find the Buzz package
# find_package(PkgConfig)
# pkg_check_modules(BUZZ REQUIRED buzz)
find_package(Buzz)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${BUZZ_INCLUDE_DIR})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

# Find Lua52 ('Lua' for MacOSX)
find_package(Lua)
if(Lua_FOUND)
  include_directories(${LUA_INCLUDE_DIR})
endif(Lua_FOUND)

# Compile target
add_library(exp_lf SHARED exp_lf.h exp_lf.cpp)
target_link_libraries(exp_lf
  buzz
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_buzz)

add_library(kh_exp_lf SHARED kh_exp_lf.h kh_exp_lf.cpp)
target_link_libraries(kh_exp_lf
  buzz
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_buzz)

# Install stuff
install(TARGETS exp_lf DESTINATION ${ARGOS_PREFIX}/lib/argos3)
