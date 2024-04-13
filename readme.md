# USB-LIDAR DESCRIPTION#
windows/linux SDK and Demo programs for lanhai 2d lidar

#HOW TO BUILD AND USE#

## LINUX ##
Prerequisite: g++ and gcc must be installed

	mkdir build
	cd build
	cmake ..
	make
	./demo ../config/xxxx.txt

## WINDOWS ##
windows powershell

    use cmake_gui  build ,open project and compile
    need mfc support
	Detailed operation reference  docs/SDK开发手册.docx
open project by  visual stdio 2013  or greater than this version 
Generate a solution

**Special instructions: Please refer to the corresponding product's documentation to select the specified configuration file, and ensure that the software can be called normally only after the hardware works normally.**

## config file from windows to linux ##
	vim xxxx.txt
	set ff=unix

#  Related address links #

[downland ros drive](https://github.com/BlueSeaLidar/sdk2)




