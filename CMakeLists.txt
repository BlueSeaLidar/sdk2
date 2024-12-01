cmake_minimum_required(VERSION 3.10)

#设置生成的VS项目文件结构和本地文件目录一样
function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else()
            set(_source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)
 
function(my_add_executable)
    foreach(_source IN ITEMS ${ARGN})
        assign_source_group(${_source})
    endforeach()
    add_executable(${ARGV})
endfunction(my_add_executable)

#工程名称
project(bluesea)
#C++98  C99
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
IF (UNIX)
# 添加对gdb的支持
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

ELSEIF (WIN32)
#多线程
set(CMAKE_CXX_FLAGS_DEBUG "/MT")
#强制文件输出
#set(CMAKE_EXE_LINKER_FLAGS_DEBUG       "/FORCE")
ENDIF ()
#指定生成debug模式
set(CMAKE_CONFIGURATION_TYPES Debug)
#所有的头文件目录
include_directories ("${PROJECT_SOURCE_DIR}/sdk")
include_directories ("${PROJECT_SOURCE_DIR}/sdk/service")
#加载指定目录下的指定格式文件集
file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/sdk/*.cpp  ${PROJECT_SOURCE_DIR}/sdk/*.c  ${PROJECT_SOURCE_DIR}/example/main.cpp)
file(GLOB_RECURSE HEADER_FILES  ${PROJECT_SOURCE_DIR}/sdk/*.h  ${PROJECT_SOURCE_DIR}/sdk/*.hpp)

IF (UNIX)
  #加载指定目录下需要删除的指定格式文件集
  #file(GLOB_RECURSE REMOVE_SOURCES  "${PROJECT_SOURCE_DIR}/sdk/service/LidarCheckService_win32.cpp")
  #file(GLOB_RECURSE REMOVE_HEADERS  "${PROJECT_SOURCE_DIR}/sdk/win32/*")
  #获取最终编译需要的文件集
  #list(REMOVE_ITEM SOURCE_FILES ${REMOVE_SOURCES})
  #list(REMOVE_ITEM HEADER_FILES ${REMOVE_HEADERS})
  find_package(Threads REQUIRED)
  source_group("Include" FILES ${HEADER_FILES})
  #编译生成可执行文件demo
  my_add_executable(demo ${SOURCE_FILES} ${HEADER_FILES})
  #添加链接库pthread
  target_link_libraries(demo m Threads::Threads)
ELSEIF (WIN32)
  #同上
  file(GLOB_RECURSE REMOVE_SOURCES  "${PROJECT_SOURCE_DIR}/sdk/uart.c")
  #file(GLOB_RECURSE REMOVE_HEADERS  "${PROJECT_SOURCE_DIR}/sdk/linux/*")
  
  list(REMOVE_ITEM SOURCE_FILES ${REMOVE_SOURCES})
  #list(REMOVE_ITEM HEADER_FILES ${REMOVE_HEADERS})
  
  source_group("Include" FILES ${HEADER_FILES})
  my_add_executable(demo ${SOURCE_FILES} ${HEADER_FILES})
ENDIF ()

#make install   将可执行文件放到指定位置
#install(TARGETS demo RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/tools/)