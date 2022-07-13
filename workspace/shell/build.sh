#!/bin/sh
#设置头文件路径
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$PWD/../../sdk:$PWD/../../sdk/linux
#编译单独的接口(调用了同名系统接口)
gcc -std=c99 -c -o ../../tools/linux/uart.o ../../sdk/linux/uart.c
gcc  -std=c99  -c ../../sdk/third_party/mongoose/mongoose.c   -o   ../../sdk/third_party/mongoose.o
gcc  -std=c99  -c  ../../sdk/third_party/cJson/cJSON.c  -o   ../../sdk/third_party/cJSON.o
g++  -std=c++98  -c  ../../sdk/service/LidarCheckService_linux.cpp  -o   ../../sdk/service/LidarCheckService.o
g++  -std=c++98  -c  ../../sdk/service/LidarWebService.cpp  -o   ../../sdk/service/LidarWebService.o
g++  -std=c++98  -o ../../tools/linux/demo \
                    ../../sdk/parser.cpp \
                    ../../sdk/user.cpp \
                    ../../sdk/standard_interface.cpp \
                     ../../sdk/linux/udp_linux.cpp  \
                      ../../sdk/linux/uart_linux.cpp \
                      ../../sdk/third_party/mongoose.o \
                       ../../sdk/third_party/cJSON.o \
                        ../../demo/main.cpp \
                         ../../tools/linux/uart.o  \
                         ../../sdk/service/LidarCheckService.o\
                          ../../sdk/service/LidarWebService.o \
                           -lpthread