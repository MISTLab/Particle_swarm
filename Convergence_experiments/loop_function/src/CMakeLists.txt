cmake_minimum_required(VERSION 2.8.12)
project(experiments)

if(APPLE)
  # MAC OSX
  set(ARGOS_SHARED_LIBRARY_EXTENSION "dylib")
  set(ARGOS_MODULE_LIBRARY_EXTENSION "so")
  set(ARGOS_DYNAMIC_LIBRARY_VARIABLE "DYLD_LIBRARY_PATH")
  # Allow for dynamic lookup of undefined symbols
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -undefined dynamic_lookup")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -undefined dynamic_lookup")
  # Add address sanitizer support for CLang
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_DEBUG} ${CMAKE_C_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} -fsanitize=address")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  set(CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${CMAKE_MODULE_LINKER_FLAGS_DEBUG}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
else(APPLE)
  # Linux
  #
  # Align doubles for higher performance
  # Also: required by the PhysX engine
  #set(ARGOS_PC_CFLAGS -malign-double)
  add_definitions(${ARGOS_PC_CFLAGS})
  # Avoid discarding unused symbols to allow plugins to work
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed")
  set(ARGOS_SHARED_LIBRARY_EXTENSION "so")
  set(ARGOS_MODULE_LIBRARY_EXTENSION "so")
  set(ARGOS_DYNAMIC_LIBRARY_VARIABLE "LD_LIBRARY_PATH")
endif(APPLE)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_CXX_STANDARD 11)
# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Find the Buzz package
find_package(PkgConfig)
pkg_check_modules(BUZZ REQUIRED buzz)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${BUZZ_INCLUDE_DIR})

# Find Lua52 ('Lua' for MacOSX)
find_package(Lua)
if(Lua_FOUND)
  include_directories(${LUA_INCLUDE_DIR})
endif(Lua_FOUND)

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${BUZZ_LIBRARY_DIRS})

# Compile target


add_library(planning_lf SHARED Planning.h Planning.cpp)
target_link_libraries(planning_lf
  buzz
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_kheperaiv
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_buzz)


# Install stuff
install(TARGETS planning_lf DESTINATION ${ARGOS_PREFIX}/lib/argos3)
