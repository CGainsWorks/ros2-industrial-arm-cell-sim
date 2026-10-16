cmake_minimum_required(VERSION 3.16)
project(cell_io_scanner CXX)

# EIPScanner installs its headers under include/EIPScanner and a shared
# library. C++20 because EIPScanner's headers need it.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_library(EIPSCANNER_LIBRARY NAMES EIPScanner REQUIRED)
find_path(EIPSCANNER_INCLUDE_DIR NAMES ConnectionManager.h PATH_SUFFIXES EIPScanner REQUIRED)

add_executable(io_scanner io_scanner.cpp)
target_include_directories(io_scanner PRIVATE ${EIPSCANNER_INCLUDE_DIR})
target_link_libraries(io_scanner PRIVATE ${EIPSCANNER_LIBRARY})
target_compile_options(io_scanner PRIVATE -Wall -Wextra)
install(TARGETS io_scanner RUNTIME DESTINATION bin)
