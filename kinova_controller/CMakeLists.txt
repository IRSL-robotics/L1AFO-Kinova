cmake_minimum_required(VERSION 3.5)
project(kinova_controller)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

enable_language(C)
enable_language(CXX)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)


find_package(ament_index_cpp REQUIRED)

find_package(robot_state_msgs REQUIRED)
find_package(controller_interface_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# for kortex API
set(KORTEX_INCLUDE_DIR
  "${PROJECT_SOURCE_DIR}/../third_party/kortex_api/include"
  CACHE PATH "Directory containing the Kortex C++ API headers")
set(KORTEX_LIBRARY
  "${PROJECT_SOURCE_DIR}/../third_party/kortex_api/lib/release/libKortexApiCpp.a"
  CACHE FILEPATH "Path to libKortexApiCpp.a")

if(NOT EXISTS "${KORTEX_INCLUDE_DIR}/client_stubs/BaseClientRpc.h")
  message(FATAL_ERROR
    "Kortex API headers were not found. Set KORTEX_INCLUDE_DIR to the Kortex "
    "C++ API include directory (tested with Kortex API 2.3.0).")
endif()
if(NOT EXISTS "${KORTEX_LIBRARY}")
  message(FATAL_ERROR
    "libKortexApiCpp.a was not found. Set KORTEX_LIBRARY to the Kortex C++ "
    "API static library (tested with Kortex API 2.3.0).")
endif()

include_directories(${KORTEX_INCLUDE_DIR})
include_directories(${KORTEX_INCLUDE_DIR}/client)
include_directories(${KORTEX_INCLUDE_DIR}/common)
include_directories(${KORTEX_INCLUDE_DIR}/messages)
include_directories(${KORTEX_INCLUDE_DIR}/client_stubs)
include_directories(${KORTEX_INCLUDE_DIR}/cxxopts/)
include_directories(${KORTEX_INCLUDE_DIR}/messages/)
link_libraries(${KORTEX_LIBRARY})
add_definitions(-D_OS_UNIX)

# for Mujoco start
set(MUJOCO_PATH
  "${PROJECT_SOURCE_DIR}/../third_party/mujoco-2.3.1"
  CACHE PATH "MuJoCo 2.3.1 distribution root")
set(SIMULATE_PATH "${MUJOCO_PATH}/simulate")
set(LODEPNG_PATH
  "${PROJECT_SOURCE_DIR}/../third_party/lodepng"
  CACHE PATH "Directory containing lodepng.h and lodepng.cpp")

if(NOT EXISTS "${MUJOCO_PATH}/include/mujoco/mujoco.h" OR
   NOT EXISTS "${SIMULATE_PATH}/simulate.cc")
  message(FATAL_ERROR
    "MuJoCo 2.3.1 headers and simulate sources were not found. Set "
    "MUJOCO_PATH to the extracted MuJoCo 2.3.1 distribution root.")
endif()
if(NOT EXISTS "${LODEPNG_PATH}/lodepng.h" OR
   NOT EXISTS "${LODEPNG_PATH}/lodepng.cpp")
  message(FATAL_ERROR
    "LodePNG sources were not found. Set LODEPNG_PATH to a directory "
    "containing lodepng.h and lodepng.cpp.")
endif()

find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses REQUIRED)
find_package(glfw3 REQUIRED)

find_library(MUJOCO
  NAMES libmujoco.so.2.3.1
  PATHS ${MUJOCO_PATH}/lib
  NO_DEFAULT_PATH
  REQUIRED
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${robot_state_msgs_INCLUDE_DIRS}
  ${controller_interface_msgs_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
  Eigen3::Eigen
  pinocchio::pinocchio
  ${YAML_CPP_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${robot_state_msgs_LIBRARIES}
  ${controller_interface_msgs_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)

add_library(data STATIC
  src/kinova_controller/data/controller_state.cpp
  src/kinova_controller/data/gen3_state.cpp
  src/kinova_controller/data/nominal_plant.cpp
  src/kinova_controller/data/robot_state.cpp
  src/kinova_controller/data/controller_logging_data.cpp
  )

add_library(lodepng STATIC ${LODEPNG_PATH}/lodepng.h ${LODEPNG_PATH}/lodepng.cpp)
target_include_directories(lodepng PUBLIC ${LODEPNG_PATH})
add_library(libsimulate STATIC)
target_sources(
  libsimulate
  PUBLIC  ${SIMULATE_PATH}/simulate.h
  PRIVATE ${SIMULATE_PATH}/simulate.cc
          ${SIMULATE_PATH}/array_safety.h
          ${SIMULATE_PATH}/uitools.h
          ${SIMULATE_PATH}/uitools.cc
          ${SIMULATE_PATH}/glfw_dispatch.h
          ${SIMULATE_PATH}/glfw_dispatch.cc
)
target_link_libraries(libsimulate PUBLIC lodepng ${MUJOCO} ${rclcpp_LIBRARIES})
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
target_compile_options(libsimulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_include_directories(libsimulate 
  PUBLIC 
    ${SIMULATE_PATH}
    ${MUJOCO_PATH}/include
    ${MUJOCO_PATH}/simulate
    ${MUJOCO_PATH}/sample
    ${rclcpp_INCLUDE_DIRS}
)
target_compile_features(libsimulate PRIVATE cxx_std_17)
target_compile_definitions(libsimulate PUBLIC MJSIMULATE_STATIC)
link_libraries(libsimulate
                lodepng
                glfw
                ${GLFW3_LIBRARY} 
                libsimulate
                ${CURSES_LIBRARY}
                ${MUJOCO}
                data
)
include_directories(
  ${MUJOCO_PATH}/include/mujoco
  ${MUJOCO_PATH}/include
  ${MUJOCO_PATH}/simulate
  ${MUJOCO_PATH}/sample
  ${GLFW_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${CURSES_INCLUDE_DIR}
)
# for Mujoco end

add_library(utils STATIC
  src/utils/memory_lock.cpp
  src/utils/rt_thread.cpp
)

add_library(kinova_controller STATIC
  src/kinova_controller/controller/controller.cpp
  src/kinova_controller/controller/controller_gain.cpp
  src/kinova_controller/filter/lpf.cpp
)
target_link_libraries(kinova_controller data)

add_library(robot STATIC
  src/kinova_controller/robot/kinova_gen3.cpp
  src/kinova_controller/robot/kinova_gen3_simulation.cpp
)
target_link_libraries(robot kinova_controller data)

add_library(node STATIC
  src/node/kinova_controller_node.cpp)
target_link_libraries(node robot kinova_controller data)


add_executable(run 
  src/main.cpp
)

target_link_libraries(run robot data utils node kinova_controller)

# install executables
install(TARGETS
  run
  DESTINATION lib/${PROJECT_NAME})

# install utils include
install(
  DIRECTORY include/
  DESTINATION include
)

# install utils lib
install(TARGETS utils 
  EXPORT export_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)



# install runtime resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
