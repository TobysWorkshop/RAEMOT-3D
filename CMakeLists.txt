cmake_minimum_required(VERSION 3.16)
project(raemot_3d CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to an optimized build unless the caller asked for something else -
# without this, plain `cmake -B build` gives you an unoptimized binary, which
# has bad performance for real-time event rates.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# --- Dependencies ---------------------------------------------------------

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp) # fallback for other versions (this was needed in the past on some devices)
    add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
    target_include_directories(yaml-cpp::yaml-cpp INTERFACE ${YAML_CPP_INCLUDE_DIR})
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE ${YAML_CPP_LIBRARIES})
endif()

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)


#find_package(Boost 1.70 REQUIRED)
#target_link_libraries(raemot_3d PRIVATE Boost::boost)

# --- Executable ------------------------------------------------------------

set(SHARED_SOURCES
    src/processing.cpp
    src/TrackManager.cpp
    src/kalman.cpp
    src/SAEdetector.cpp
    src/parameters.cpp
    src/SAE.cpp
    src/track_logger.cpp
    src/track_summary_logger.cpp
    src/raw_event_logger.cpp
)

add_executable(aemot_realtime
    src/camera_driver.cpp
    ${SHARED_SOURCES}
)

add_executable(raemot_3d
    src/stereo_driver.cpp
    ${SHARED_SOURCES}
    src/threadC_params.cpp
    src/threadC.cpp
    src/writer_queue.cpp
)

# Tie the raemot_3d binary to this computer's specific setup for even more optimisation - NOTE: this does mean that it will only work on the computer you build it on
target_compile_options(raemot_3d PRIVATE -march=native)

# gen4/common is a sibling of this project's own root (raemot_3d/), holding
# the sepia/psee/evk4 camera headers. Giving it as a proper include path
# (rather than relying on a "../gen4/common/x.hpp" relative include inside
# each source file) means moving files around within src/ later can't silently break the camera includes
target_include_directories(aemot_realtime PRIVATE
    ${CMAKE_SOURCE_DIR}/../gen4/common
)
target_include_directories(raemot_3d PRIVATE
    ${CMAKE_SOURCE_DIR}/../gen4/common
)

target_link_libraries(aemot_realtime PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    PkgConfig::LIBUSB
    Threads::Threads
)

target_link_libraries(raemot_3d PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    PkgConfig::LIBUSB
    Threads::Threads
)

# processing.cpp resolves configs/<name>.yaml relative to the running
# executable's own location via /proc/self/exe (see executable_directory()
# in processing.cpp), assuming configs/ sits exactly one directory above
# wherever the binary lands. Putting the binary straight in the build
# directory root (the default here - no subdirectories) keeps that true as
# long as you build from the raemot_3d/ project root, e.g. `cmake -B build`
# run from here puts the binary at raemot_3d/build/raemot_3d, and
# raemot_3d/build/../configs correctly resolves to raemot_3d/configs.
