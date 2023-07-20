set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_FLAGS "-Wno-psabi" CACHE STRING "" FORCE )
set(CMAKE_CXX_FLAGS "-Wno-psabi"  CACHE STRING "" FORCE )

set(CMAKE_C_FLAGS "-L${CMAKE_SYSROOT}/usr/lib")
set(CCMAKE_CXX_FLAGS "-L${CMAKE_SYSROOT}/usr/lib")

if(EXISTS ${CMAKE_SYSROOT} AND IS_DIRECTORY ${CMAKE_SYSROOT})
  message(STATUS "SYSROOT found")
else()
  message(FATAL_ERROR "ERROR: SYSROOT not found!!!")
endif()

set(NODEJS_INCLUDE_DIR /usr/include/node) # make sure that nodejs is installed. If not, sudo apt-get install nodejs-dev

set(PYTHON_INCLUDE_DIRS "${CMAKE_SYSROOT}/usr/include/python3.10")
set(PYTHON_LIBRARIES "${CMAKE_SYSROOT}/usr/lib/libpython3.10.so")

set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
