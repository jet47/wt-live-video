cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

project(wt-live-video)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Wt REQUIRED)
include_directories(${Wt_INCLUDE_DIR})

set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS thread signals date_time system chrono regex filesystem random program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(FFMPEG REQUIRED)
include_directories(${FFMPEG_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

file(GLOB HEADER_FILES "*.h" "*.hpp")
file(GLOB SOURCE_FILES "*.c" "*.cpp")

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} debug ${Wt_DEBUG_LIBRARY} optimized ${Wt_LIBRARY} 
                                      debug ${Wt_HTTP_DEBUG_LIBRARY} optimized ${Wt_HTTP_LIBRARY} 
                                      debug ${Wt_EXT_DEBUG_LIBRARY} optimized ${Wt_EXT_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})