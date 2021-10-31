cmake_minimum_required(VERSION 3.10)

project(k_minimum_spanning_tree VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#set (PROJECT_SOURCES
#  src/main.cpp
#  )
#add_executable(kmst ${PROJECT_SOURCES}
add_executable(kmst ${PROJECT_SOURCES}
  src/main.cpp
  src/kmst.cpp
  src/io.cpp
  src/vertex_impl.cpp
  )

target_include_directories(kmst PRIVATE src)

target_include_directories(kmst PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/lib/boost/boost/
  src/
  )

set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -O3")

