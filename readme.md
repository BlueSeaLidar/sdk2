# USB-LIDAR DESCRIPTION#
windows/linux SDK and Demo programs for lanhai 2d lidar

#HOW TO BUILD AND USE#

## LINUX ##
Prerequisite: g++ and gcc must be installed

    cd  ./workspace/shell
    ./build.sh

Generate executable directory

plug lidar usb port, make sure /dev/ttyUSBx existed, add read / write attribution

	
    cd  ./tools/linux
	//uart
	sudo chmod 666 /dev/ttyUSB0
	
    ./demo   ../../config/xxx.txt  (specified model)


## WINDOWS ##
windows powershell

    cd  workspace/vc10
    ./demo.sln
    
cmd

open project by  visual stdio 2010  or greater than this version 
Generate a solution

Generate executable directory
    
    cd  ./tools/win32-x86
	//For specific parameter description, please refer to main.cpp
    demo.exe  arg1 arg2...  
**Special instructions: Please refer to the corresponding product's documentation to select the specified configuration file, and ensure that the software can be called normally only after the hardware works normally.**


#FILE DIRECTORY STRUCTURE#
<pre>
lanhai-driver-master/
├── config
│   ├── LDS-15D-B25R.txt
│   ├── LDS-50C-2.txt
│   ├── LDS-50C-C20E.txt
│   ├── LDS-50C-C30E.txt
│   └── LSS-40S-B20E44.txt
├── demo
│   └── main.cpp
├── docs
│   └── SDK开发手册.docx
├── sdk
│   ├── data.h
│   ├── error.h
│   ├── linux
│   │   ├── uart.c
│   │   ├── uart_linux.cpp
│   │   ├── uart_linux.h
│   │   ├── udp_linux.cpp
│   │   └── udp_linux.h
│   ├── parser.cpp
│   ├── standard_interface.cpp
│   ├── standard_interface.h
│   ├── user.cpp
│   └── win32
│       ├── uart_win32.cpp
│       ├── uart_win32.h
│       ├── udp_win32.cpp
│       └── udp_win32.h
├── tools
│   ├── linux
│   │   ├── demo
│   │   └── uart.o
│   └── win32_x86
│       └── demo.exe
├── workspace
│   ├── shell
│   │   └── build.sh
│   └── vc10
│       ├── demo
│       │   ├── demo.vcxproj
│       │   ├── demo.vcxproj.filters
│       │   └── demo.vcxproj.user
│       └── demo.sln
└── 详细说明.md



<pre>


#  Related address links #

[downland ros drive](https://github.com/BlueSeaLidar/bluesea2)




