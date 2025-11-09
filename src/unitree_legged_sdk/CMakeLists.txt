cmake_minimum_required(VERSION 3.10)
project(unitree_legged_sdk)

include(GNUInstallDirs)

# --- ARCHITECTURE DETECTION ---
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  # Unitree sometimes names their folders 'arm64' instead of 'aarch64'
  set(ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

include_directories(include)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# Unitree static library
set(UNITREE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a)
set(EXTRA_LIBS -pthread ${UNITREE_LIB} -ldl)

# --- PYTHON INTEGRATION (THE FIX) ---
option(PYTHON_BUILD "Build python wrapper" OFF)
if(PYTHON_BUILD)
  add_subdirectory(python_wrapper)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  
  # FORCE standard ROS 2 installation path: lib/python3.X/site-packages
  set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

  # Install routine - Looks for precompiled .so in the location you identified
  install(CODE "
    # 1. Define where to look for the source .so file
    set(SEARCH_DIR \"${CMAKE_CURRENT_SOURCE_DIR}/lib/python/${ARCH}\")
    message(STATUS \"Searching for unitree python bindings in: \${SEARCH_DIR}\")

    # 2. Find the file
    file(GLOB PYTHON_BINDING_FILES \"\${SEARCH_DIR}/*robot_interface*.so\")

    # 3. Error if not found, otherwise install to the FORCED standard path
    if(NOT PYTHON_BINDING_FILES)
        message(FATAL_ERROR \"Could not find *robot_interface*.so in \${SEARCH_DIR}\")
    else()
        message(STATUS \"Found bindings, installing to: \${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}\")
        file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}\" TYPE FILE FILES \${PYTHON_BINDING_FILES})
    endif()
  ")
endif()

# --- C++ EXAMPLES ---
add_executable(example_position example/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})
add_executable(example_velocity example/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})
add_executable(example_torque example/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})
add_executable(example_walk example/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})
add_executable(example_joystick example/example_joystick.cpp)
target_link_libraries(example_joystick ${EXTRA_LIBS})

# --- STANDARD INSTALL RULES ---
install(FILES ${UNITREE_LIB} DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS example_position example_velocity example_torque example_walk example_joystick
        DESTINATION lib/${PROJECT_NAME})